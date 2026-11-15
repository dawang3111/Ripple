#include <exploration_manager/tx_delay_gate.h>
#include <cassert>
#include <string>
#include <vector>

using Gate = racer_eval::TxDelayGate<std::string>;
int main() {
  Gate gate;
  gate.enabled = true;
  gate.offset_s = 1;
  gate.fraction = 0.75;
  // WAIT_TRIGGER must never consume the injection opportunity.
  for (int i = 1; i <= 4; ++i) {
    auto a = gate.message("duplicate", i * 40000000LL, false, false, 2, 1000000000);
    assert(a.size() == 1 && a[0].kind == "forward");
  }
  auto planned = gate.message("duplicate", 200000000, true, true, 1.0, 1000000000);
  assert(planned.size() == 1 && planned[0].kind == "scheduled");
  assert(planned[0].planned_delay_ns == 30000000 && planned[0].message == "duplicate");
  assert(gate.timer(220000000).empty());
  // An unexpectedly early successor causes release before forwarding it.
  auto early = gate.message("duplicate", 221000000, true, true, 1.021, 1000000000);
  assert(early.size() == 2 && early[0].kind == "release" && early[1].kind == "forward");
  assert(early[0].seq + 1 == early[1].seq && early[0].reason == "early_successor");
  assert(early[0].message == early[1].message); // repeated payloads retain distinct sequences
  assert(gate.timer(300000000).empty());
  for (int i = 7; i <= 12; ++i)
    assert(gate.message("next", i * 40000000LL, true, true, 2, 1000000000)[0].kind == "forward");

  Gate inactive;
  inactive.enabled = true;
  auto skipped = inactive.message("x", 100, true, false, 1, 1000000);
  assert(skipped.size() == 2 && skipped[0].kind == "skip" && !skipped[0].injected);
  assert(skipped[0].reason == "inactive_fsm_at_selected_time");

  Gate no_history;
  no_history.enabled = true;
  assert(no_history.message("x", 100, true, true, 1, 1000000)[0].reason == "insufficient_period_history");

  Gate timed;
  timed.enabled = true;
  timed.offset_s = 1;
  for (int i = 1; i <= 4; ++i) timed.message("p", i * 800000000LL, true, true, 0, 1000000000);
  auto bounded = timed.message("p", 4000000000LL, true, true, 1, 1000000000);
  // No fixed 250 ms clipping: protocol remaining budget is the actual constraint.
  assert(bounded[0].period_ns == 800000000 && bounded[0].planned_delay_ns == 199999999);
  assert(timed.timer(4200000000LL)[0].kind == "release");

  Gate exhausted;
  exhausted.enabled = true;
  exhausted.offset_s = 1;
  for (int i = 1; i <= 4; ++i) exhausted.message("p", i * 40000000LL, true, true, 0, 100);
  assert(exhausted.message("p", 200000000, true, true, 1, 0)[0].reason == "no_protocol_or_simulation_budget");
}
