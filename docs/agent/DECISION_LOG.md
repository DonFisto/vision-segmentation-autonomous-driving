# Decision Log

Use this file for durable project decisions. Append new entries; do not rewrite earlier decisions merely because the design evolves. When a decision is superseded, link the new entry and mark the old one accordingly.

## Entry template

### ADR-XXX — Decision title

- **Date:** YYYY-MM-DD
- **Status:** proposed / accepted / superseded / rejected
- **Context:** What problem or constraint required a decision?
- **Options considered:**
  1. Option and trade-offs.
  2. Option and trade-offs.
- **Decision:** What was chosen?
- **Rationale:** Why is it the best choice for the current educational objective and constraints?
- **Consequences:** What becomes easier, harder, or deferred?
- **Validation:** How will the decision be tested?
- **Supersedes / superseded by:** ADR reference, when applicable.

---

## ADR-001 — Keep the first prototype deliberately simple

- **Date:** 2026-07-30
- **Status:** accepted
- **Context:** The repository is intended to teach the complete autonomous-driving stack while the developer also manages a double degree, Formula Student, and potential professional work.
- **Options considered:**
  1. Attempt production-like breadth and infrastructure.
  2. Build the simplest coherent closed loop, then deepen one subsystem.
- **Decision:** Build a simple, modular, measurable closed-loop prototype before adding advanced planning, control, localization, or infrastructure.
- **Rationale:** This maximizes conceptual coverage and learning per unit of limited time while preserving a path toward deeper work.
- **Consequences:** Simulator odometry, basic planning, and basic controllers are acceptable. Full SLAM, HD maps, complex prediction, and production deployment are deferred.
- **Validation:** The first system milestone must complete repeatable CARLA scenarios and expose measurable perception-to-actuation behavior.

## ADR-002 — Use CARLA odometry for the first planning-and-control milestone

- **Date:** 2026-07-30
- **Status:** accepted provisionally
- **Context:** The current accumulated mapping stage uses hero odometry, while independent ego-motion estimation is a future direction.
- **Options considered:**
  1. Block planning and control until visual odometry or SLAM is implemented.
  2. Continue using simulator-provided odometry for the first closed loop.
- **Decision:** Keep CARLA odometry as an accepted simplification unless the first full review finds an unusable interface or inconsistency.
- **Rationale:** Replacing odometry is not on the critical path to learning planning and control.
- **Consequences:** Localization realism remains limited, but planning, control, interfaces, and evaluation can progress.
- **Validation:** Confirm frame, timestamp, and transform consistency across odometry, occupancy, mapping, planner, and controller.

## ADR-003 — Target engineering understanding across the stack

- **Date:** 2026-07-30
- **Status:** accepted
- **Context:** AI has materially assisted implementation, and independently rewriting every component would be incompatible with the project's breadth and available time.
- **Options considered:**
  1. Require independent reimplementation of all components.
  2. Require operational understanding only.
  3. Require engineering understanding across the stack and reimplementation depth in selected areas.
- **Decision:** Target Level 2 engineering understanding across all major components and Level 3 reimplementation depth only in one or two high-value areas.
- **Rationale:** This supports responsible ownership of the code without turning every dependency into a separate course.
- **Consequences:** Planning, optimization, or control are likely depth targets; adapters and mature external libraries may remain at Level 1 or Level 2.
- **Validation:** Use targeted understanding questions and focused exercises during project reviews.
