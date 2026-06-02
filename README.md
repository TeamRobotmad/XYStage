# XYStage app

XYStage is a Tildagon app for controlling a 2-axis microscope stage through external DRV8825 stepper drivers and one endstop per axis.

## Hardware and slots

- XY stage hexpansion: VID: 0xCBCB, PID: 0x6000
- Optional joystick hexpansion: VID: 0xCBCB, PID: 0x6001
- One PWM channel is used per axis

## Main menu

- XYStage: manual jog mode
- Home XY: runs the built-in two-stage homing sequence
- Run NC: browse and execute `.nc` files from `/gcode`
- Record NC: jog and capture waypoints, then save a replay script
- Settings: edit/persist settings
- About
- Exit

## Controls by mode

- XYStage mode:
	Left/Right/Up/Down (or joystick) jog axes.
	Confirm performs go-to target behavior.
	Cancel exits to menu.
	Confirm+Cancel starts homing sequence.
- NC Replay mode:
	Cancel aborts replay.
	Confirm continues on `M0` or `M1` pauses.
- NC Record mode:
	Move with buttons or joystick.
	Confirm short press captures waypoint.
	Confirm long press opens save dialog.
	Cancel discards recording and exits.
	Confirm+Cancel starts homing sequence.

## Full settings reference

All settings are visible in the Settings menu and can be persisted.

| Setting | Meaning | Units | Default | Min | Max |
|---|---|---|---:|---:|---:|
| `logging` | Enable console logs | bool | `True` | `False` | `True` |
| `width` | Rendered stage width | microsteps | `64000` (`2000*32`) | `10` | `100000` |
| `height` | Rendered stage height | microsteps | `64000` (`2000*32`) | `10` | `100000` |
| `XRange` | Travel limit on X | microsteps | `70400` (`2200*32`) | `10` | `100000` |
| `YRange` | Travel limit on Y | microsteps | `64000` (`2000*32`) | `10` | `100000` |
| `min_speed` | Minimum axis speed | microsteps/s | `320` (`10*32`) | `10` | `10000` |
| `max_speed` | Maximum axis speed | microsteps/s | `32000` (`1000*32`) | `10` | `100000` |
| `acceleration` | Maximum speed change per control update | microsteps/s per update | `3200` (`100*32`) | `10` | `10000` |
| `x_usteps_per_mm` | X conversion scale | microsteps/mm | `1280` | `1` | `100000` |
| `y_usteps_per_mm` | Y conversion scale | microsteps/mm | `1280` | `1` | `100000` |
| `gcode_feed` | Default feed for generated/parsed motion when `F` not updated | mm/min | `300` | `1` | `60000` |
| `gcode_accel` | Stored acceleration value used by `M204` logic | mm/s^2 | `20` | `1` | `10000` |

## GCODE support

### File location and extension

- Directory: `/gcode`
- Extension: `.nc`

### Parser behavior

- Command letters are case-insensitive.
- `G00` maps to `G0`, `G01` maps to `G1`, `M00` maps to `M0`, `M01` maps to `M1`.
- `;` comments are supported.
- Parenthesized comments `( ... )` are supported.
- Words are parsed as `<LETTER><VALUE>` and may be separated by spaces or commas.
- `G28 X`, `G28 Y`, and `G28 O` flag syntax (no numeric value) is accepted.
- `G28 O1` style numeric flag is also accepted (`O != 0` means enabled).
- Unsupported commands or invalid parameters fail load/parse.

### Units

- `X`, `Y`: mm
- `F`: mm/min
- `M203 X/Y`: mm/min (axis speed limits)
- `M204 S`: mm/s^2
- `G4 P`: milliseconds
- `G4 S`: seconds
- `M114` reports positions in mm

### Supported commands

| Command | Format | Parameters | Behavior |
|---|---|---|---|
| Rapid/linear move | `G0 ...` / `G1 ...` | `X`, `Y`, optional `F` | Moves to target (absolute or relative per mode). At least one axis required. |
| Dwell | `G4 ...` | `P` and/or `S` | Waits for specified time before continuing. |
| Home | `G28` | optional `X`, `Y`, `O` | Homes selected axes (or both if none provided). |
| Conditional home | `G28 O` | optional `X`, `Y` | Home only axes not already calibrated. |
| Absolute mode | `G90` | none | Future `G0/G1` coordinates interpreted as absolute. |
| Relative mode | `G91` | none | Future `G0/G1` coordinates interpreted as relative. |
| Set position | `G92 ...` | `X`, `Y` | Sets current logical position in mm for provided axes. |
| Program stop | `M0` / `M1` | none | Pauses replay until Confirm is pressed. |
| Enable motors | `M17` | none | Enables both steppers. |
| Disable motors | `M18` | none | Stops and disables both steppers. |
| Emergency stop | `M112` | none | Immediate stop, disable, and error state. |
| Report position | `M114` | none | Logs current X/Y in mm. |
| Axis speed limits | `M203 ...` | `X`, `Y` | Sets per-axis replay speed caps. Values must be > 0. |
| Acceleration | `M204 S...` | `S` | Updates replay acceleration behavior. `S` must be > 0. |
| Sync/wait complete | `M400` | none | Accepted and completes immediately. |

### Parameter rules and ranges

- `G0/G1` must include at least one of `X` or `Y`.
- `G4` must include at least one of `P` or `S`; both must be >= 0.
- `M203` must include at least one of `X` or `Y`; provided values must be > 0.
- `M204` requires `S`; value must be > 0.
- Duplicate parameters in one line are rejected.
- Unknown parameters for a command are rejected.
- Non-numeric or non-finite values are rejected.

### Important execution details

- Move targets are clamped to `[0, XRange]` and `[0, YRange]` (internal microstep limits).
- Homing moves toward negative direction until endstop/contact latch.
- Homing timeout is computed dynamically from range and homing speed.

## Default homing preamble used in recorded scripts

Recorded `.nc` files include a two-stage homing preamble generated by the app:

1. Fast home (`G28`) at current `gcode_feed`-based axis limits.
2. Back off a short distance in relative mode.
3. Slower home (`G28`) using reduced `M203` limits.
4. Restore fast limits and absolute mode.

This uses existing commands only, no custom GCODE opcodes.

Current tuning constants in code:

- Backoff distance: `5.0` mm
- Fine homing feed ratio: `0.25` of fast feed
- Fine homing minimum: `20` mm/min

## Installation

Stable versions are published in the [Tildagon App Directory](https://apps.badge.emfcamp.org/).

For direct deployment, the core app files are:

- `app.py`
- `tildagon.toml`
- `metadata.json`

## Running tests

```bash
pytest -q
```
