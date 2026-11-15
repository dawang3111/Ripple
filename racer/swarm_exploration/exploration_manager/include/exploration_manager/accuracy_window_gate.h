#ifndef RACER_ACCURACY_WINDOW_GATE_H
#define RACER_ACCURACY_WINDOW_GATE_H
#include <cstdint>
namespace racer_eval {
// Controls selection only. It never buffers a ROS message or sleeps a callback.
class AccuracyWindowGate {
 public:
  enum State { MONITORING, WAITING, IN_FLIGHT };
  struct Window {
    bool triggered=false, selected=false;
    uint64_t id=0, original_seq=0, selected_seq=0;
    int64_t trigger_ns=0, decision_ns=0, expires_ns=0, wait_ns=0;
  };
  int64_t wait_ns=0;
  State state() const { return state_; }
  const Window& current() const { return window_; }
  Window message(int64_t now, uint64_t seq, bool new_trajectory,
                 bool eligible, bool rule_fresh, int64_t decision, int64_t expires) {
    Window event;
    if (!new_trajectory || !eligible || state_==IN_FLIGHT) return event;
    if (state_==MONITORING) {
      if (!rule_fresh) return event;
      window_=Window();window_.id=++counter_;window_.original_seq=seq;
      window_.trigger_ns=now;window_.decision_ns=decision;window_.expires_ns=expires;
      window_.wait_ns=wait_ns;state_=WAITING;event=window_;event.triggered=true;
    }
    // A stored window needs no fresh token or rule match after its wait.
    if (state_==WAITING && now>=window_.trigger_ns+window_.wait_ns) {
      window_.selected_seq=seq;state_=IN_FLIGHT;
      const bool triggered=event.triggered;event=window_;
      event.triggered=triggered;event.selected=true;
    }
    return event;
  }
  void reset() {state_=MONITORING;window_=Window();}
 private:
  State state_=MONITORING;Window window_;uint64_t counter_=0;
};
}
#endif
