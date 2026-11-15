#ifndef RACER_TX_DELAY_GATE_H
#define RACER_TX_DELAY_GATE_H

#include <algorithm>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace racer_eval {
// Source-side, single-channel gate. Never sleeps or changes message contents.
// The caller must publish returned actions in order, on the same callback queue.
template <class Message> class TxDelayGate {
public:
  struct Action {
    std::string kind, reason;
    Message message;
    uint64_t seq = 0;
    int64_t generated_ns = 0, planned_delay_ns = 0, budget_ns = 0, period_ns = 0;
    bool injected = false;
  };
  bool enabled = false;
  bool wait_for_arm = false, armed = false;
  double offset_s = 0, fraction = 0.5;
  int64_t protocol_ns = 200000000;

  std::vector<Action> message(const Message& msg, int64_t now, bool task_started,
                              bool active, double elapsed_s, int64_t remaining_ns,
                              int64_t message_age_ns = 0) {
    std::vector<Action> out;
    // Release first, even when a callback arrives earlier than predicted.
    if (pending_) release(out, "early_successor");
    if (last_ns_ && now > last_ns_) {
      periods_.push_back(now - last_ns_);
      if (periods_.size() > 8) periods_.pop_front();
    }
    last_ns_ = now;
    Action a;
    a.message = msg;
    a.seq = ++seq_;
    a.generated_ns = now;
    a.kind = "forward";
    if (enabled && (!wait_for_arm || armed) && !attempted_ && task_started && elapsed_s >= offset_s) {
      attempted_ = true;
      std::string why;
      if (!active) why = "inactive_fsm_at_selected_time";
      else if (periods_.size() < 3) why = "insufficient_period_history";
      else if (!(fraction > 0 && fraction < 1)) why = "invalid_fraction";
      else {
        auto sorted = std::vector<int64_t>(periods_.begin(), periods_.end());
        std::sort(sorted.begin(), sorted.end());
        a.period_ns = sorted[sorted.size() / 2];
        // Strictly less than a measured cycle and the remaining protocol/run budget.
        a.budget_ns = std::min(protocol_ns - std::max<int64_t>(0, message_age_ns), remaining_ns);
        a.planned_delay_ns = std::min<int64_t>(a.period_ns * fraction, a.budget_ns - 1);
        if (a.planned_delay_ns <= 0) why = "no_protocol_or_simulation_budget";
      }
      if (why.empty()) {
        a.kind = "scheduled";
        a.injected = true;
        pending_.reset(new Action(a));
        due_ns_ = now + a.planned_delay_ns;
        out.push_back(a);
        return out;
      }
      Action skipped = a;
      skipped.kind = "skip";
      skipped.reason = why;
      out.push_back(skipped);
    }
    out.push_back(a);
    return out;
  }

  std::vector<Action> timer(int64_t now) {
    std::vector<Action> out;
    if (pending_ && now >= due_ns_) release(out, "scheduled_timer");
    return out;
  }

private:
  void release(std::vector<Action>& out, const std::string& reason) {
    Action a = *pending_;
    a.kind = "release";
    a.reason = reason;
    out.push_back(a);
    pending_.reset();
  }
  bool attempted_ = false;
  uint64_t seq_ = 0;
  int64_t last_ns_ = 0, due_ns_ = 0;
  std::deque<int64_t> periods_;
  std::unique_ptr<Action> pending_;
};
}  // namespace racer_eval
#endif
