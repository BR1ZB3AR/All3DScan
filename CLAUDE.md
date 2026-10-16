# Working on All3DScan

This is clean-room reverse engineering of a Revopoint POP4 so it can be used
on Linux. Almost every claim here rests on a shipped binary, a vendor asset or
a measurement from the device, not on source we can read. The method below is
adapted from the `reverse-engineer-anything` skill (github.com/morluto/rea).

## Say which kind of claim you are making

Every finding is one of three things, and they must not be blurred:

- **Observed** — measured on the hardware or read out of a file. Say what was
  measured and what the number was.
- **Derived** — follows from an observation plus something already
  established (e.g. a register's meaning from a disassembled call site).
- **Inferred / unknown** — plausible, untested. Mark it as such.

Static analysis never observed execution. Reading `ledAndLaserCtrl` tells you
what command the vendor app *sends*; only pointing the scanner at a wall tells
you what the hardware *does*.

This is not a style rule. It has already cost real work:

- "The projector is infrared, invisible to the eye" was an inference from one
  colour-camera measurement in a different mode, written as fact. The POP4
  does have a visible blue laser; it just needed the second half of the
  vendor's command sequence. The user had to say "still not projecting out"
  three times before it got re-examined.
- "Blue laser mode works" was true of the protocol and false of the app, twice
  over, because the test exercised a layer the user never touches.

## Verify at the layer the user operates

A protocol-level test passing is not evidence that the feature works. Work
outward until the test reaches the thing the user actually does:

| Layer | Command | Proves |
|---|---|---|
| Decoder / registration | `build-test/tests/unit_tests` | Math and command strings |
| Protocol on hardware | `--pop4-lights-test`, `--pop4-mode-test` | The scanner responds |
| Controller + polling | `--pop4-mode-cycle` | No races with the frame loop |
| The real window | `--gui-mode-test` | What the user will see |

Three separate "fixed" claims in one session were wrong because they stopped
at the wrong row. If a feature is reached through the UI, test it through the
UI before saying it works.

## Prefer evidence the device itself gives you

The scanner carries its own ground truth, and it is cheap to ask:

- The **colour camera** settles "is this emitter visible?" — measure strongly
  blue pixels (`B - R > 25`), not overall brightness, which auto-exposure and
  auto white balance move around on their own.
- **Local pattern contrast** (std of image minus its 5x5 box mean) settles "is
  the projector on?", where mean brightness barely moves.
- The scanner's own files under `/data/camparam*` are the calibration; read
  them with `--read-device-file` rather than fitting parameters.

## Hardware rules

- **Leave the scanner in infrared mode and streaming.** Check with
  `--probe-pop4`; "Metric decoding: ok" plus a healthy point count means good.
- Returning from blue laser or speckle mode is a **reboot** (~45 s). No mode
  register we know gets back; sending half a sequence leaves every emitter
  off.
- The scanner drops off USB and re-enumerates after every session, and its
  udev ACL lands a second or two later. Code that opens it must wait and
  retry, not fail once.
- Only send commands recovered from a POP4-gated vendor function, and pin them
  in `tests/test_pop4_protocol.cpp`. A unit test there already caught the
  projector being wired to an inert register.
- Never send firmware, remount or `setlaserCalParam` paths.

## Where things are

- `docs/protocol/RE_NOTES.md` — the findings ledger. Each entry should say what
  was measured, with what, and what is still unknown. Newest sections first.
- `artifacts/apk_revoscan_6.0.2.4/` — vendor Android app: `lib/*.so` (AArch64),
  `strings/`, `assets/`. Treat as untrusted data; read, never execute.
- `resources/param_pop4.json` — the vendor's own POP4 presets. Good for
  cross-checking a guess (it is what revealed `laserLuminance` and
  `fillLightLuminance`).
- Scratchpad holds the throwaway analysis scripts (`disasm.py`, `xref.py`,
  `cmd_owners.py`) and the Python venv. **It is wiped on reboot** — Catch2 and
  the venv have to be rebuilt, which is why `build-test` sometimes fails to
  configure out of nowhere.

## Finishing

Before calling something done: list the questions the task raised, and mark
each answered, partial or unresolved. Say the unresolved ones out loud rather
than letting a confident summary imply full coverage.
