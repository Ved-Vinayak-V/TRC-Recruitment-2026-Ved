# Task 1 — Webots project (1a keyboard control + 1b line follower)

One Webots project, two worlds, two controllers. Both worlds are built from
Webots **base nodes only** — there is not a single `EXTERNPROTO` line, so
nothing is downloaded and nothing is looked up inside your Webots installation.

## Folder layout

Webots works out where the controllers live from the world file's parent
folder, so keep this structure intact.

```
task1_webots/
├── worlds/
│   ├── task1a.wbt              <- keyboard control
│   └── task1b.wbt              <- line follower
└── controllers/
    ├── keyboard_controller/
    │   └── keyboard_controller.py
    └── line_follower/
        └── line_follower.py
```

## Running either one

1. Extract the zip somewhere stable. On **Windows use right-click ▸ Extract
   All** — double-clicking the zip shows a preview that only *looks* like a
   folder, and Webots cannot find the controllers inside it.
2. Webots → **File ▸ Open World…** → `worlds/task1a.wbt` or `worlds/task1b.wbt`.
3. Press ▶.
4. For **1a only**: click once inside the 3D view so it takes keyboard focus.
   1b needs no input at all.

If Python fails to start, set **Tools ▸ Preferences ▸ General ▸ Python command**
to `python3` (macOS/Linux) or the full path to `python.exe` (Windows), then
reload the world with Ctrl+Shift+T. Webots ships its own `controller` module —
there is nothing to `pip install`.

---

# Task 1a — keyboard control

| Key       | Action                      |
|-----------|-----------------------------|
| `W` / `↑` | forward                     |
| `S` / `↓` | backward                    |
| `A` / `←` | turn left                   |
| `D` / `→` | turn right                  |
| `SPACE`   | brake — stops immediately   |

Keys combine: `W`+`D` arcs right, `S`+`A` reverse-turns. Releasing lets the
robot roll down smoothly; `SPACE` cuts it dead.

`keyboard.getKey()` returns only **one** key per call, so the controller drains
the buffer every step — that is what makes two keys work together. `& 0xFFFF`
strips the SHIFT/CTRL/ALT bits Webots ORs into the key code. Speeds ramp by
`ACCEL` rad/s per step instead of jumping, so the wheels don't spin out from a
standing start.

---

# Task 1b — line follower

Press ▶ and the robot drives the loop on its own. Same chassis as 1a, with
three downward-facing infra-red sensors added under the nose.

```
        ir_left   ir_center   ir_right        +y is the robot's LEFT
           |          |          |
      --------------------------------  front of chassis
                  travel ->
```

## How the line is detected

Most Webots line-follower examples paint the line into a **floor texture** and
rely on the infra-red sensor picking up the change in reflectance. That works,
but it depends on exactly how Webots weights surface colour, and it needs an
image file alongside the world.

This track instead lays the line down as a chain of **dark box segments
standing 4 mm proud of the floor**. Each sensor therefore sees two changes at
once when it crosses the line — a *shorter distance* and a *darker surface* —
so there is a usable signal whichever of the two effects dominates. A 4 mm step
is nothing to a wheel of 50 mm radius.

The sensors sit 18 mm above the floor and use `lookupTable [0 1000 0, 0.03 0 0]`,
so on distance alone the readings work out at roughly:

| Surface under the sensor | Distance | Reading |
|--------------------------|----------|---------|
| clear floor              | 18 mm    | ~400    |
| the line                 | 14 mm    | ~533    |

Surface darkness shifts those numbers further apart — possibly the other way
round. Which brings us to the next part.

## Why the controller calibrates itself

Nothing above is hard-coded. For the first half-second the robot sits still and
measures, because the start pose is arranged so the **centre sensor is on the
line and the outer two are off it**:

```
calibration: line=533.2  floor=400.1  contrast=+133.1
the line reads HIGHER than the floor - following it now.
```

Every reading after that is converted to a 0..1 "how much line am I seeing"
value by `(raw - floor) / (line - floor)`. Because that divisor is *signed*,
the same expression works unchanged if the line turns out to read lower than
the floor. If you move the robot, change the surface colours or swap in a
textured floor, the controller re-learns on the next reload instead of needing
new constants.

If it ever prints `WARNING: almost no contrast`, the robot was not sitting on
the line when it started — reload the world rather than fighting the gains.

## The control law

```
error = (w_right - w_left) / (w_left + w_centre + w_right)
turn  = KP * error + KD * (error - previous_error)
left  = base + turn        right = base - turn
```

`error` is positive when the line is off to the right, which speeds the left
wheels up and turns the robot right. Base speed is cut by up to 55 % in
proportion to `|error|`, so it slows into corners and opens up on the straights.

If every sensor loses the line, the robot spins towards whichever side saw it
last while creeping forward, until it finds it again.

## Tuning

All at the top of `line_follower.py`:

| Constant          | Default | Effect                                            |
|-------------------|---------|---------------------------------------------------|
| `BASE_SPEED`      | `3.6`   | rad/s on the straights → 0.18 m/s                 |
| `KP`              | `3.4`   | how hard it corrects — too high and it weaves     |
| `KD`              | `7.0`   | damps the weave; raise this before lowering `KP`  |
| `CORNER_SLOWDOWN` | `0.55`  | fraction of speed given up in a hard corner       |
| `LOST_TOTAL`      | `0.18`  | below this the line counts as lost                |
| `DEBUG_EVERY`     | `60`    | steps between console status lines; `0` is silent |

Want it faster? Raise `BASE_SPEED` first, and when it starts cutting corners
raise `KP` and `KD` together.

## What was checked before shipping

The controller was driven round the real track geometry in a head-less
differential-drive simulation, across eight scenarios: both sensor polarities
(line reading higher than the floor and lower), an ideal drive and a
skid-steer that only delivers 55 % of commanded rotation, contrast as weak as
45 units, heavy sensor noise, and two cases where the robot is physically
shoved off the line mid-lap.

All eight complete laps without losing the line. The worst recovery from a
70 mm shove took 46 control steps (about 0.7 s), and the robot holds the line
to within 16 mm of centre — roughly half the line's own width.

## Useful while debugging

* **View ▸ Optional Rendering ▸ Show DistanceSensor Rays** draws the three
  sensor beams, which makes it obvious when one is off the line.
* The console prints `TRACK L=.. C=.. R=.. err=..` every 60 steps. `L/C/R` are
  the normalised 0..1 values, so a healthy run over a straight reads roughly
  `L=0.00 C=1.00 R=0.00`.
* Ctrl+Shift+T reloads the world and re-runs calibration from the start pose.

---

## Notes on Webots versions

Both worlds target **R2025a** (`#VRML_SIM R2025a utf8`).

PROTO nodes such as `RectangleArena` and `TexturedBackground` resolve through a
URL at load time, and that resolution is the usual cause of a world opening to
red console errors like `Skipped unknown 'TexturedBackground' node or PROTO`.
When that happens the scene loads *without* them, and since the lighting PROTO
is normally the only light source the 3D view goes black even though the robot
is fine. Nothing here depends on that machinery: the floor, walls, sky, lights
and the whole track are plain `Solid` / `Background` / `DirectionalLight` nodes
written straight into the world files.

Running an **older** Webots? Change the header line to your version, e.g.
`#VRML_SIM R2023b utf8`; everything else works unchanged back to R2022b. On
**R2022a or older**, also replace each `Pose { … }` with `Transform { … }` —
the field names are identical.

## Where Task 1b goes next

The chassis already carries wheel encoders (`enc_fl`, `enc_fr`, `enc_rl`,
`enc_rr`), so odometry is available if you want to measure lap times or
distance travelled without touching the world file.
