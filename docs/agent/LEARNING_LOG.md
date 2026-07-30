# Learning Log

Use this file to record durable understanding gained from the project. This is not a daily diary and should not duplicate the development timeline.

Append entries when a concept becomes clearer, an assumption is disproved, or a subsystem reaches a new understanding level.

## Entry template

### YYYY-MM-DD — Topic

- **Component:**
- **Previous understanding level:** 1 / 2 / 3
- **Current understanding level:** 1 / 2 / 3
- **What became clear:**
- **Evidence:** code change, experiment, explanation, derivation, or debugging result
- **Remaining confusion:**
- **Next exercise:**
- **Connection to mathematics or systems coursework:**

---

## Initial learning targets

These are hypotheses to validate during the first full review, not claims about current understanding.

| Area | Initial target | Why it matters | Suggested evidence of understanding |
| --- | --- | --- | --- |
| Semantic segmentation | Level 2 | Current perception foundation | Explain dataset labels, model choice, preprocessing, metrics, domain shift, and failure cases |
| Depth and fusion | Level 2 | Bridges image output to geometry | Explain relative versus metric depth, calibration assumptions, synchronization, and fusion limitations |
| Tracking | Level 2 | Introduces temporal state and association | Explain state model, data association, occlusion behavior, and stale tracks |
| Occupancy and mapping | Level 2 | Direct input candidate for planning | Explain projection, frames, grid resolution, accumulation, static/dynamic semantics, and drift |
| Planning | Level 3 candidate | Strong connection to algorithms and optimization | Implement and justify a simplified planner; define costs, constraints, and failure modes |
| Control | Level 3 candidate | Strong connection to ODEs, numerics, and optimization | Derive or reimplement a baseline controller and compare it experimentally |
| ROS2 integration | Level 2 | Essential production-style systems skill | Trace topics, timing, parameters, launch behavior, and debugging workflow |
| CARLA actuation | Level 1–2 | Simulator-specific adapter | Explain scaling, saturation, command ownership, and feedback loop |

## Review questions to revisit

1. Can every module's inputs, outputs, frames, units, and consumers be described without reading the implementation?
2. Can the dominant failure mode of every major subsystem be predicted?
3. Can a component be modified without asking AI to regenerate the surrounding architecture?
4. Which part of the system can currently be reimplemented in simplified form?
5. Which mathematical idea became tangible because of a real debugging or design problem?
