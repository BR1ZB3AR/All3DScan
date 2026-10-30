# Memory index (copy)

A copy of what Claude keeps in its own memory for this project, so the
same notes are in the repository. The working copy lives outside it, in
`~/.claude/projects/-home-br1zb3ar-All3DScan/memory/`; this is a copy, so
it can fall behind.

The record of the work itself is [WORK_LOG.md](WORK_LOG.md).

- [CachyOS, sudo needs password](machine-cachyos-sudo-password.md) — Arch/pacman box; hand install commands to the user, use Arch package names
- [REA + Ghidra setup](rea-ghidra-setup.md) — /opt/ghidra, JDK 21, Node 24 LTS; CLAUDE.md's ~/ghidra paths are stale here; refresh command
- [POP 4 lights + blue laser in the app](pop4-lights-blue-laser-in-app.md) — user requirement; reuse RE_NOTES/legacy commands; blue laser can't scan yet; REA as fallback
- [POP 4 in the new app](pop4-in-new-app.md) — wired in 2026-10-27; projector needs streaming first, V4L2 gives stale frames, depth yield unresolved
- [Hybrid stack plan status](hybrid-stack-plan-status.md) — phases 1–4 done 2026-10-27, POP 4 wired in after; agreed deviations (Qt GL view, OCP, RANSAC in Rust, planes as faces)
- [Commit straight to main](git-commit-straight-to-main.md) — All3DScan: push to main, no branch/PR; main unprotected; repo-local git identity set
- [Third-party vendoring](third-party-vendoring.md) — PoissonRecon is BSD-3-Clause despite an MIT LICENSE file; check per-file headers; name our wrappers for what they do
- [Measure before claiming](measure-before-claiming.md) — several wrong claims came from wrong probes, not wrong reasoning; suspect the probe, prefer differential checks
