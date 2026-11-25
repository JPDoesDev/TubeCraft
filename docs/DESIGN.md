# TubeCraft design notes

Target: SolidCraft 0.48.1 or later, plugin WIT `craft:plugin@0.2.0`, capability `commands@0.2`.

## Approach

SolidCraft has no plugin-owned parametric feature type yet, so TubeCraft is a command plugin: it
runs once and leaves ordinary features that refer to the sketch curves. The tubes rebuild when
the wireframe moves, without the plugin. The perpendicular cope is the exception
([below](#perpendicular-cope)).

One run makes one physical tube and copes it against the tubes already in the design. The
design itself is the record: no plugin storage is needed, and tubes made by hand count too.

## Commands used

| Step | Command |
|---|---|
| Read the picks and earlier paths | `sketch.inspect` |
| Find earlier tubes | `document.inspect`, then `timeline.inspect` per pipe feature |
| Tube, run-on, flush cope, miter cut | `solid.pipe` (new body, join or cut) |
| Corner helper lines, cope cut lines | `sketch.create`, `sketch.options`, `sketch.line3d`, `sketch.finish` |
| Perpendicular cope | `surface.loft`, `solid.split_body`, `inspect.measure`, `solid.remove` |
| Tidy | `timeline.group` (one folder per tube), `browser.visibility` (hide the helper sketches) |

Expression fields read a bare number in the design's unit, so every size is sent as text with
`mm`, and lengths read back are parsed with their unit. Point coordinates are always
millimetres.

TubeCraft's own cuts are named `Cope ...` and `Miter ...` and are skipped when looking for
tubes. A tube name that starts with either word is therefore refused.

## Rules

- The picked curves must join end to end (within 0.01 mm) and run on tangent (within 0.5°), in
  one sketch, in any pick order and direction. A closed loop must be tangent at its seam too.
- An earlier tube is a pipe feature that makes a new body. Pipes joined onto its body are its
  run-ons. Its bodies are those with its body name or `<name> (n)`.
- Two tubes meet when their centrelines are closer than the sum of their radii less 0.05 mm.
- Where they meet:
  1. The new tube's end on the earlier tube's end (within 0.5 mm), with no third tube there: a
     **free corner**. Miter when asked and both are the same OD (within 0.01 mm), else butt
     with a warning.
  2. Else the new tube ends on the earlier tube: the new tube is coped.
  3. Else the earlier tube ends on the new tube: the earlier tube's end is coped. A through
     tube is never cut in its middle.
  4. Else they cross: the new tube is cut in two, with a warning.
- A new tube that ends on an earlier tube and meets it again further along is coped at the end
  and cut in two at the crossing.
- **Miter**: each tube runs on past the corner by `r tan(turn / 2) + 1 mm`, then a solid
  cylinder whose flat end lies on the miter plane cuts each tube's far side.
- **Butt**: the earlier tube runs on by the furthest reach of the new tube's outside inside it
  (one radius of the smaller tube at a square corner), and the new tube is coped to it.
- A tube that continues an earlier one in a straight line, or turns back on it, gets no joint
  and a warning. A miter over 120°, a butt over 150° and a large tube coped to a smaller one
  are made with a warning.
- A bend whose radius is not more than the tube's radius is refused. One that differs from the
  die CLR by more than 0.5 % is a warning.

## Limits of one run

One run may make 256 SolidCraft calls in 30 s: 1 to read the design, one per pipe feature, one
per sketch read, then 3 to about 16 to build with flush copes, plus 8 per perpendicular cope.
The run counts its calls and stops at 250 with its own message before it changes anything.

## Perpendicular cope

A tube laser cuts square to the tube wall: the cut face is ruled radially from the tube's axis.
A flush cope is cut by the other tube's cylinder, so on the inside of the wall it reaches
further than a laser cut would.

The cut is a surface lofted through open radial lines, which then splits the tube:

- 48 lines round the tube (49 loft sections; the last repeats the first), drawn as one zigzag
  polyline whose odd segments are the lines. They overshoot the wall by 1.5 mm outside and up
  to 1.5 mm inside.
- Each line sits where the wall (five fibres from inside to outside) stops reaching into any
  tube the end lands on, plus the weld gap and 0.02 mm. One cut serves every tube of a node.
- The trim is worked out at 720 angles. The lines are spread evenly along the cut's own length,
  because lines bunched at a crease make the kernel refuse the surface.
- The loft is smooth, so beside a crease it swings past the lines; the lines there are lifted
  by 0.12 of the crease's step. The end clears the other tubes by a few hundredths of a
  millimetre on a plain joint and a few tenths at the point of a crease.
- The cope reaches along the tube only as far as the stretch, starting at the end, where the
  axis stays within reach of the other tubes. A second meeting further along is a crossing.
- Of the two pieces the split leaves, the larger is the tube. The cope is never longer than
  the straight end it is cut from, so the off-cut is always the smaller.
- It falls back to the flush cope, with a warning: an end in a bend or shorter than the cope, a
  joint too shallow, an earlier tube that is a solid bar, in several pieces or not one chain.

**It does not follow the sketch.** The cut lines are fixed world points. When the wireframe
moves, a tube coped this way keeps its old length, or its split fails. Make the tube again.

**The helper sketches reappear.** The run's last call hides them, but SolidCraft 0.48.1 to
0.49.1 show them again when a plugin's run ends. The same commands sent as a plain batch keep
them hidden, so the loss is in the host's plugin run.

Other ways to build the cut were refused by the kernel for a same-size tee: a loft between
closed 3D splines, a ruled strip from the flush cope's outside edge (as a split tool, thickened
as a cut, or thickened to a kerf).

## Cut length

`tube.cut_length_in` is the stock to cut: the centreline, less what each coped end loses at its
long point, on to the long point of each miter (`r tan(turn / 2)` past the corner). A flush
cope's long point is the least that any fibre of the wall is cut back by the other tube's
outside: for a 1.5 in tube square on a 1.75 in one, `sqrt(22.225² - 19.05²)` = 11.447 mm, and
nothing for two tubes of one size. An end coped in a bend is counted to the centreline's end.

## Roadmap

- Cut list and bend sheet (CSV): cut length, bend angles, rotation between bends, cope angles.
- Printable cope wrap templates.
- Per-joint overrides in the dialog.
- More materials: DOM, 4130 chromoly, square and rectangular tube.
- New sizes are one line each in `tubecraft-plugin/src/profiles.rs` and `tubecraft/plugin.json`.
