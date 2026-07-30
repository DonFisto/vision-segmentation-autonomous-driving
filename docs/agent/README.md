# Codex Project Review Framework

This folder gives Codex persistent context and a repeatable process for reviewing the autonomous-driving prototype.

## Files

| File | Purpose |
| --- | --- |
| `PROJECT_CONTEXT.md` | Stable developer constraints, project purpose, scope, and success criteria |
| `CURRENT_STATE.md` | Latest evidence-based summary of implemented, partial, absent, and unclear subsystems |
| `ARCHITECTURE.md` | Current data-flow map, interface registry, and architecture invariants |
| `ROADMAP.md` | Small dependency-aware sequence of active and future milestones |
| `DECISION_LOG.md` | Durable architecture and scope decisions |
| `LEARNING_LOG.md` | Evidence of developer understanding and selected depth targets |
| `REVIEW_PLAYBOOK.md` | Procedure Codex follows for repository, subsystem, and learning reviews |
| `REPORT_TEMPLATE.md` | Standard output structure for substantial reviews |
| `reports/` | Dated review reports; historical reports should not be silently rewritten |

The repository root `AGENTS.md` defines the agent's mission and mandatory behavior.

## First review

After this framework is merged, run Codex with:

> Perform a full repository review according to `AGENTS.md` and `docs/agent/REVIEW_PLAYBOOK.md`. Do not modify implementation code. Validate the initial project state and architecture, create a dated report, and update the agent state files only where supported by repository evidence. End with no more than three active tasks for the next development cycle.

## Incremental review

After a development cycle, use:

> Review changes since `<commit-or-report>`. Follow `AGENTS.md` and `docs/agent/REVIEW_PLAYBOOK.md`. Compare the implementation and available runtime evidence with the previous state, create a dated report, and update only the state, roadmap, decisions, and learning entries justified by evidence.

## Subsystem review

Use:

> Review `<subsystem>` as an autonomous-systems engineer, mathematical reviewer, software engineer, and learning mentor. Ground every finding in repository evidence. Do not change code. Identify blockers, suitable metrics, current understanding level, and the smallest next experiment.

## Implementation handoff

A review does not authorize implementation. After accepting a recommendation, use:

> Implement `<specific task>` from `<report path>`. Preserve the agreed interfaces, state acceptance criteria before editing, make the smallest coherent change, run the relevant checks, and report what remains unverified.

## Maintenance rule

- Update `CURRENT_STATE.md` and `ARCHITECTURE.md` when evidence changes.
- Keep only a few tasks active in `ROADMAP.md`.
- Append to `DECISION_LOG.md` and `LEARNING_LOG.md`; do not erase history.
- Store substantial reviews under `reports/YYYY-MM-DD-<scope>.md`.
