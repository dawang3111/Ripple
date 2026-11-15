#ifndef RACER_INTENSITY_DELAY_GATE_H
#define RACER_INTENSITY_DELAY_GATE_H
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
namespace racer_eval {
// One outstanding trajectory. All successor publication events, including heartbeat
// retransmissions, release the predecessor first and are themselves forwarded.
template<class M> class IntensityDelayGate {
 public:
  struct Action {std::string kind,reason; M message; uint64_t seq=0; int64_t generated_ns=0,target_ns=0,decision_ns=0; bool selected=false;};
  bool enabled=false; int group=1; int64_t fixed_ns=200000000;
  uint64_t next_seq() const {return seq_+1;}
  std::vector<Action> message(const M& msg,int64_t now,bool select,int64_t decision_ns=0){
    std::vector<Action> out; const bool occupied=bool(pending_);
    if(occupied) release(out,group==5?"successor_G5":"early_successor");
    Action a;a.message=msg;a.seq=++seq_;a.generated_ns=now;a.selected=select;a.decision_ns=decision_ns;a.target_ns=group==5?0:fixed_ns;
    if(enabled&&select&&!occupied){a.kind="scheduled";pending_.reset(new Action(a));due_=now+fixed_ns;out.push_back(a);}
    else {a.kind="forward";a.reason=occupied?"successor_forwarded_no_overlap":"normal";out.push_back(a);}
    return out;
  }
  std::vector<Action> timer(int64_t now){std::vector<Action> out;if(pending_&&group!=5&&now>=due_)release(out,"fixed_timer");return out;}
  std::vector<Action> flush(){std::vector<Action> out;if(pending_)release(out,"task_end_flush");return out;}
 private:
  uint64_t seq_=0;int64_t due_=0;std::unique_ptr<Action> pending_;
  void release(std::vector<Action>& out,const std::string& reason){Action a=*pending_;a.kind="release";a.reason=reason;out.push_back(a);pending_.reset();}
};}
#endif
