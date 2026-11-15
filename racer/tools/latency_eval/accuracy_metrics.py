"""Audit missed-window selection separately from the shared trajectory outcomes."""
import csv
import json
import uuid
from pathlib import Path

from .core import write_json
from . import intensity_metrics

WAITS = dict(G1=0., G2=1., G3=2., G4=3., G5=5.)


def window_evidence(events, plan, injections):
    windows = {}; errors = []; active = {}
    decisions = {e['decision_ns']: e for e in events if e['kind'] == 'intensity_rule_decision'}
    tx = {(e['id'], e['seq'], e['action']): e for e in events if e['kind'] == 'intensity_tx'}
    publications = [e for e in events if e['kind'] == 'accuracy_publication']
    releases = {(r['sender_id'], r['seq']): r for r in injections}
    for e in events:
        if e['kind'] != 'accuracy_window':
            continue
        key = (e['id'], e['window_id']); action = e['action']
        if action == 'triggered':
            if key in windows or e['id'] in active:
                errors.append('duplicate_or_overlapping_window:' + str(key))
            windows[key] = {'trigger': e}; active[e['id']] = key
        elif key not in windows:
            errors.append('window_event_without_trigger:' + str(key))
        else:
            slot = 'selection' if action == 'selected' else 'terminal'
            if slot in windows[key]:
                errors.append('duplicate_window_transition:' + str(key))
            windows[key][slot] = e
            if slot == 'terminal':
                active.pop(e['id'], None)
    rows = []
    for (sender, wid), entries in windows.items():
        t = entries['trigger']; s = entries.get('selection'); end = entries.get('terminal')
        decision = decisions.get(t['decision_ns']); local = []
        origin_valid = bool(decision and decision['source_id'] == sender and
                            decision['decision_ns'] <= t['trigger_ns'] <= decision['expires_ns'])
        if not origin_valid:
            local.append('missing_or_expired_origin_rule_evidence')
        if t['wait_ns'] != int(WAITS[plan['group']] * 1e9):
            local.append('configured_wait_mismatch')
        skipped = t['wait_ns'] > 0
        forward = tx.get((sender, t['original_seq'], 'forward'))
        if skipped and not (forward and forward['generated_ns'] == t['trigger_ns'] and
                            (sender, t['original_seq'], 'scheduled') not in tx):
            local.append('original_message_was_not_forwarded_without_buffering')
        release = None; selected_ns = None
        if s:
            selected_ns = s['action_ns']; seq = s['selected_seq']
            scheduled = tx.get((sender, seq, 'scheduled'))
            if not scheduled or scheduled['generated_ns'] != selected_ns:
                local.append('selection_without_matching_source_schedule')
            if s['decision_ns'] != t['decision_ns']:
                local.append('selection_replaced_original_rule_token')
            if selected_ns < t['trigger_ns'] + t['wait_ns']:
                local.append('selected_before_wait_deadline')
            if (skipped and seq <= t['original_seq']) or (not skipped and seq != t['original_seq']):
                local.append('wrong_original_or_later_message_identity')
            candidates = [e for e in publications if e['id'] == sender and e['new_trajectory'] and
                          e['eligible'] and e['generated_ns'] >= t['trigger_ns'] + t['wait_ns']]
            first = min(candidates, key=lambda e: e['generated_ns']) if candidates else None
            if not first or first['seq'] != seq:
                local.append('not_first_eligible_new_publication_after_deadline')
            release = releases.get((sender, seq))
            if release and release['completed_injection'] and release['reason'] != 'successor_G5':
                local.append('nonstandard_packet_delay_strategy')
        if not end:
            local.append('missing_terminal_window_event')
        elif end['action'] == 'completed' and not (s and release and release['completed_injection']):
            local.append('completed_window_without_completed_source_delay')
        elif end['action'] not in ('completed', 'cancelled'):
            local.append('unknown_terminal_window_action')
        errors.extend(str((sender, wid)) + ':' + reason for reason in local)
        rows.append(dict(sender_id=sender, window_id=wid, trigger_ns=t['trigger_ns'],
                         decision_ns=t['decision_ns'], matched_rule_ids=json.dumps(decision.get('matched_rule_ids', []) if decision else []),
                         trigger_fsm=t.get('fsm'), selection_fsm=s.get('fsm') if s else None,
                         skipped_original_window=skipped, configured_wait_s=t['wait_ns']/1e9,
                         original_seq=t['original_seq'], original_forward_ns=forward['publish_before_ns'] if forward else None,
                         selected_seq=s['selected_seq'] if s else None, actual_injection_ns=selected_ns,
                         actual_wait_s=(selected_ns-t['trigger_ns'])/1e9 if s else None,
                         rule_evidence_valid_at_origin=origin_valid,
                         original_token_fresh_at_selection=bool(s and selected_ns <= t['expires_ns']),
                         rule_required_at_selection=False, release_ns=release['release_ns'] if release else None,
                         actual_packet_delay_s=release['actual_delay_s'] if release else None,
                         completed_injection=bool(release and release['completed_injection']),
                         terminal_action=end['action'] if end else None,
                         cancellation_reason=end.get('reason') if end and end['action']=='cancelled' else None,
                         window_contract_valid=not local, errors=json.dumps(local)))
    scheduled_keys = {(e['id'], e['seq']) for e in events if e['kind']=='intensity_tx' and e['action']=='scheduled'}
    selected_keys = {(r['sender_id'], r['selected_seq']) for r in rows if r['selected_seq'] is not None}
    if scheduled_keys != selected_keys:
        errors.append('scheduled_messages_and_selected_windows_do_not_match')
    return rows, errors


def analyze(directory, cfg, baseline):
    d = Path(directory); plan = json.loads((d/'plan.json').read_text())
    result = intensity_metrics.analyze(d, cfg, baseline)
    # Preserve a raw outcome result, but do not expose it as a finished accuracy
    # trial until the window audit is also complete.
    (d/'intensity_result.json').replace(d/('unverified_outcomes_'+uuid.uuid4().hex+'.json'))
    windows, errors = window_evidence(intensity_metrics.events_for(d), plan, result['injections'])
    by_seq = {(r['sender_id'], r['selected_seq']): r for r in windows}
    for r in result['injections']:
        w = by_seq.get((r['sender_id'], r['seq']))
        r['window_id'] = w['window_id'] if w else None
        r['rule_required_at_selection'] = False
        r['window_contract_valid'] = bool(w and w['window_contract_valid'])
        r['rule_evidence_valid_at_origin'] = bool(w and w['rule_evidence_valid_at_origin'])
    result.update(accuracy_mode=True, windows=windows, window_errors=errors,
                  window_trigger_n=len(windows), selected_window_n=sum(r['selected_seq'] is not None for r in windows),
                  cancelled_window_n=sum(r['terminal_action']=='cancelled' for r in windows),
                  standard_packet_strategy='release before next source broadcast; all accuracy groups identical',
                  result_path=str(d/'accuracy_result.json'))
    drift_file = d/'drift_samples.csv'
    samples = list(csv.DictReader(drift_file.open())) if drift_file.exists() else []
    # Keep pre/post scopes explicit; the shared CSV contains both.
    result['drift']['max_post_injection_error_m'] = max((float(r['error_m']) for r in samples if r['post_injection']=='True'), default=None)
    if errors:
        result['unfiltered_labels'] = result['labels']
        result['labels'] = {k: dict(v, valid=False, missing_reason='accuracy_window_contract_not_verified')
                            for k, v in result['labels'].items()}
    for filename in ('accuracy_result.json', 'intensity_result.json'):
        write_json(d/filename, result)
    return result
