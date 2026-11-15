"""Simulation-only multi-injection TX queue. G5 waits for actual next publication."""
from copy import deepcopy
import hashlib
import json
import math

GROUP_DELAYS={'G1':.2,'G2':.4,'G3':.6,'G4':.8,'G5':None}


def payload_hash(payload):
    def default(value):
        if hasattr(value,'tolist'):return value.tolist()
        raise TypeError('payload must be JSON-compatible')
    raw=json.dumps(payload,default=default,sort_keys=True,separators=(',',':'),allow_nan=False)
    return hashlib.sha256(raw.encode()).hexdigest()


class IntensityQueue:
    """One pending operation per sender, release B before C, then publish C normally.

    Timer execution uses the caller's actual simulation time (including polling
    lateness). No sleeps, one-second assumption, payload edits, or invented TX.
    """
    def __init__(self,group,enabled=True,delays=None):
        values=dict(GROUP_DELAYS)
        if delays:values.update(delays)
        if group not in values:raise ValueError('unknown intensity group')
        if group!='G5' and (not math.isfinite(values[group]) or values[group]<=0):raise ValueError('positive target delay required')
        if values['G5'] is not None:raise ValueError('G5 must be next-publication based, never fixed delay')
        self.group=group;self.delay=values[group];self.enabled=enabled
        self.cache={};self.pending={};self.last_seq={};self.events=[];self.operations=[];self.now=-math.inf
        self.rule_triggers=0;self.injection=dict(attempted=False,actually_injected=False,skip_reason='no_rule_match',used_stale_count=0)

    def initialize(self,positions):
        for sender,payload in positions.items():
            self.cache[sender]=dict(sender_id=sender,seq=-1,payload=deepcopy(payload),generated_time=0.,release_time=0.)

    def _release(self,message,now,reason,operation=None):
        sender=message['sender_id']
        if message['seq']<=self.cache[sender]['seq']:raise ValueError('duplicate or out-of-order release')
        if payload_hash(message['payload'])!=message['payload_sha256']:raise ValueError('queued payload modified')
        item=dict(message,release_time=now,event='release',release_reason=reason)
        self.cache[sender]=item;self.events.append(item)
        if operation is not None:
            operation.update(status='released',actual_release_time=now,actual_delay_s=now-message['generated_time'],
                release_reason=reason,truncated=reason=='next_message_preempt',timer_lateness_s=max(0.,now-operation['planned_release_time']) if operation['planned_release_time'] is not None else None)
            if not self.injection['actually_injected']:
                self.injection.update(actually_injected=True,nominal_tx_time=message['generated_time'],sender_id=sender,seq=message['seq'],actual_release_time=now,actual_tx_time=now,actual_delay_s=operation['actual_delay_s'],planned_delay_s=self.delay,skip_reason='')

    def advance(self,now):
        if not math.isfinite(now) or now<self.now:raise ValueError('simulation clock must be finite and monotonic')
        self.now=now
        due=[(sender,value) for sender,value in self.pending.items() if value[1]['planned_release_time'] is not None and value[1]['planned_release_time']<=now]
        for sender,(message,operation) in sorted(due,key=lambda item:(item[1][1]['planned_release_time'],item[0])):
            del self.pending[sender];self._release(message,now,'timer_due',operation)

    def publish(self,sender,seq,payload,now,inject=False,rule_ids=(),state='RUNNING'):
        self.advance(now)
        if seq<=self.last_seq.get(sender,-1):raise ValueError('duplicate or out-of-order publication')
        self.last_seq[sender]=seq
        matched=bool(inject and state=='RUNNING')
        if matched:self.rule_triggers+=1
        # C is an order-preserving boundary, not a second overlapping delay.
        boundary=sender in self.pending
        if boundary:
            message,operation=self.pending.pop(sender)
            operation.update(next_message_generated_time=now,next_message_seq=seq,next_message_generated_before_release=True)
            self._release(message,now,'next_publication' if self.group=='G5' else 'next_message_preempt',operation)
        message=dict(sender_id=sender,seq=seq,payload=deepcopy(payload),generated_time=now,nominal_tx_time=now,
                     payload_sha256=payload_hash(payload),delayed=False)
        if matched:
            self.events.append(dict(event='rule_trigger',sender_id=sender,seq=seq,generated_time=now,rule_ids=list(rule_ids),
                accepted=bool(self.enabled and not boundary),skip_reason='experiment_disabled' if not self.enabled else ('boundary_message_published_normally' if boundary else '')))
        if matched and self.enabled and not boundary:
            operation=dict(operation_id=len(self.operations)+1,sender_id=sender,seq=seq,group=self.group,rule_ids=list(rule_ids),
                rule_trigger_time=now,nominal_tx_time=now,target_delay_s=self.delay,
                planned_release_time=now+self.delay if self.delay is not None else None,
                actual_release_time=None,actual_delay_s=None,next_message_generated_time=None,next_message_seq=None,
                next_message_release_time=None,next_message_generated_before_release=False,
                payload_sha256=message['payload_sha256'],status='pending',truncated=False,release_reason=None,timer_lateness_s=None,
                out_of_order=False,dropped=False)
            self.operations.append(operation);message['delayed']=True;message['operation_id']=operation['operation_id']
            self.pending[sender]=(message,operation);self.events.append(dict(message,event='enqueue',operation_id=operation['operation_id']))
            self.injection['attempted']=True
            return
        self._release(message,now,'normal')
        if boundary:operation['next_message_release_time']=now

    def receive(self,sender,receiver,now):
        self.advance(now);message=self.cache[sender];stale=sender in self.pending
        if stale:self.injection['used_stale_count']+=1
        self.events.append(dict(message,event='receive',receiver_id=receiver,receive_time=now,pending_injection=stale))
        return deepcopy(message['payload'])

    def finish(self,now):
        self.advance(now)
        for message,operation in self.pending.values():
            operation.update(status='cancelled',release_reason='task_ended_before_release',dropped=True)
            self.events.append(dict(message,event='cancel',cancel_time=now,operation_id=operation['operation_id']))
        self.pending.clear()
        if not self.injection['actually_injected']:
            self.injection['skip_reason']='task_ended_before_release' if self.operations else ('experiment_disabled' if not self.enabled else 'no_rule_match')
        self.injection.update(completed_injections=sum(o['status']=='released' for o in self.operations),
            cancelled_injections=sum(o['status']=='cancelled' for o in self.operations),rule_trigger_count=self.rule_triggers)
