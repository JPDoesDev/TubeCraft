# Changelog

## 0.4.0 (2026-11-25)

First public release.

- Fixed: a perpendicular cope on a tube bent back past its own coped end deleted the tube and
  kept the off-cut. The larger piece of the split is now kept.
- Fixed: with the perpendicular cope, a tube that ends on an earlier tube and crosses it again
  further along was cut back to the far side of the crossing. It is now coped at the end and
  cut in two at the crossing, with a warning.
- Fixed: a closed loop with a sharp corner at its seam was accepted.
- Fixed: a tube named `Cope ...` or `Miter ...` was not found by later runs. Such names are now
  refused.
- Fixed: the run could reach SolidCraft's 256-call limit part-way through. It now counts every
  call and stops with its own message before it changes anything.
- Fixed: a pipe whose path is not sketch curves stopped the run. It is skipped with a warning.
- Fixed: the name's 40 limit counted bytes, not characters.
- Changed: `tube.cut_length_in` is the stock to cut, for flush copes and miters too.
  `tube.centreline_in` is the centreline as drawn.
- Added: each `perpendicular_copes` entry says which end it is at (`end_mm`).
- Changed: the helper sketches are hidden by one call at the end of the run, one host call less
  per perpendicular cope.

## 0.3.0 (2026-11-24)

- Added the **Cope** option: flush (as before) or perpendicular (tube laser), falling back to
  flush with a warning where a perpendicular cut cannot be made.

## 0.2.0 (2026-11-24)

- One physical tube per run, each in its own size, coped against the tubes already in the
  design. An earlier tube that ends on the new tube is coped instead of cutting the new tube.
- Free corners with an earlier tube: miter (same OD) or butt.

## 0.1.0 (2026-11-24)

- First version: a whole frame in one run and one size.
