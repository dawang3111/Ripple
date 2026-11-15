"""Miss an attack window without holding its message. Simulation clock only."""
from .intensity_transport import IntensityQueue

WAIT_SECONDS={'G1':0.,'G2':1.,'G3':2.,'G4':3.,'G5':5.}


class AccuracyQueue(IntensityQueue):
    """One active attack globally; all groups use next-same-source release.

    The t0 message in G2-G5 is normal. After the deadline, select the first
    eligible NEW publication from that sender without rechecking its rules.
    The release-boundary message is normal, then monitoring resumes.
    """
    def __init__(self,group,enabled=True):
        if group not in WAIT_SECONDS:raise ValueError('unknown accuracy group')
        super().__init__('G5',enabled=enabled)
        self.accuracy_group=group;self.wait_s=WAIT_SECONDS[group]
        self.windows=[];self.active_window=None

    def publish(self,sender,seq,payload,now,inject=False,rule_ids=(),state='RUNNING'):
        self.advance(now)
        if seq<=self.last_seq.get(sender,-1):raise ValueError('duplicate or out-of-order publication')
        boundary=bool(self.pending)
        window=self.active_window
        selected=False
        if self.enabled and state=='RUNNING' and not boundary:
            if window is None and inject:
                window=dict(window_id=len(self.windows)+1,group=self.accuracy_group,
                    sender_id=sender,trigger_seq=seq,rule_trigger_time=now,rule_ids=list(rule_ids),
                    actively_skipped=self.wait_s>0,configured_wait_s=self.wait_s,
                    wait_deadline_s=now+self.wait_s,actual_injection_time=None,actual_wait_s=None,
                    selected_seq=None,operation_id=None,actual_delay_s=None,
                    actual_release_time=None,released=False,status='waiting',skip_reason='')
                self.windows.append(window);self.active_window=window
                self.events.append(dict(event='accuracy_window',**window))
            if (window is not None and window['status']=='waiting' and sender==window['sender_id']
                    and now>=window['wait_deadline_s']):
                selected=True
        before=len(self.operations)
        super().publish(sender,seq,payload,now,inject=selected,
            rule_ids=window['rule_ids'] if selected else (),state=state)
        if len(self.operations)>before:
            operation=self.operations[-1]
            window.update(status='pending_release',actual_injection_time=now,
                actual_wait_s=now-window['rule_trigger_time'],selected_seq=seq,
                operation_id=operation['operation_id'])
            operation.update(group=self.accuracy_group,standard_policy='next_same_sender_publication',
                window_id=window['window_id'],rule_trigger_time=window['rule_trigger_time'],
                configured_wait_s=self.wait_s,actively_skipped=window['actively_skipped'],
                wait_deadline_s=window['wait_deadline_s'],actual_wait_s=window['actual_wait_s'],
                actual_injection_time=now)
        if window is not None and window['status']=='pending_release':
            op=self.operations[window['operation_id']-1]
            if op['status']=='released':
                window.update(status='released',released=True,actual_delay_s=op['actual_delay_s'],
                    actual_release_time=op['actual_release_time'])
                self.active_window=None

    def finish(self,now):
        super().finish(now)
        if self.active_window is not None:
            w=self.active_window
            reason='task_ended_during_wait' if w['status']=='waiting' else 'task_ended_before_release'
            w.update(status='cancelled',skip_reason=reason)
            self.events.append(dict(event='accuracy_window_cancel',cancel_time=now,**w))
            if not self.injection['actually_injected']:self.injection['skip_reason']=reason
            self.active_window=None
        self.injection.update(rule_window_count=len(self.windows),accuracy_group=self.accuracy_group,
            configured_wait_s=self.wait_s,standard_policy='next_same_sender_publication')
