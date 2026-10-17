# Approaches considered (arena c4)

- **Full on-target build of each option.** Rejected for this pass. Option 2 is a sketch that does not build (the
  sketch's `RalSessionSt` is not wired into the F´ build). A bench build needs the shared pcr-588 workspace or a
  worktree plus the Zephyr SDK, and the dev-board lock would be held for long periods. It is the right next step
  for claims G and the drills, not for the ordering question.
- **Host model of the whole RAC engine.** Rejected. A model of the engine would test my model, not the USP code.
  Instead I compiled the real `ProfilePolicy.cpp` and modelled only the event ordering, which the code fixes.
- **Host simulation of kick latency with threads.** Rejected. macOS scheduling is not the RP2350 Zephyr scheduler,
  so a number from it would not inform the decision. Used airtime arithmetic for the G budget instead.
- **Reading usp v1.2.1 only.** Rejected as the sole source. The pinned fork's usp_zephyr loop is what runs on the
  board today, so the F claim needs both; I cited both and marked the v1.2.1 usp_zephyr loop as unchecked.
- **Counting lines with cloc.** Not installed. A grep-based count is rough, and I used it only to show the table's
  numbers do not reproduce on any simple basis.
