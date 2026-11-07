# Resort the contents of recycling bins

## Status

This document records the task idea for handoff. A draft scene, scoring function and candidate scripted demonstration are in `resort_trash_bins.py`; they have not been validated in the simulator. The object list, scoring rules and run limit remain provisional.

The initial novelty check found closely related work. [PR #244](https://github.com/robocurve/benchmark-tasks/pull/244) moves misplaced items between existing emergency kits while leaving correctly placed items alone; [PR #417](https://github.com/robocurve/benchmark-tasks/pull/417) rearranges flasks already in the wrong holders; and [PR #549](https://github.com/robocurve/benchmark-tasks/pull/549) sorts waste into bins. The proposed task combines inspection and correction with waste categorization. Whether that combination is sufficiently distinct for review is unresolved; the team workflow stops at the novelty gate when an existing task tests the same ability.

## Idea and motivation

Recycling bins can become contaminated when people put an item in the wrong container. A robot is asked to inspect several bins that already contain waste, identify items that do not belong in their current bin, and move those items to the appropriate destination. Items that already belong in their bin should stay there. Some objects do not belong in any of the available recycling categories and must be set aside instead of being forced into an inappropriate bin.

The task combines these abilities:

1. Inspect the existing contents of multiple containers.
2. Recognize each item and decide which of the available categories it belongs to.
3. Notice which items are already correctly placed and which need to be moved.
4. Transfer the misplaced items to the correct destinations.
5. Set aside items for which none of the three recycling categories applies.

For example, the robot should recognize that a metal can found in the glass bin belongs in the metal bin. An apple found among the recyclables belongs in the separate set-aside area, rather than in any of the three recycling bins.

## Proposed setup

Use three distinguishable recycling bins and a separate, clearly marked plate, bowl or tray:

| Destination | Intended contents | Examples |
| --- | --- | --- |
| Glass bin | Objects designated as glass | Glass bottles |
| Paper and cardboard bin | Objects designated as paper or cardboard | A cardboard box or paper item |
| Metal bin | Objects designated as metal | Metal cans |
| Set-aside plate or tray | Items outside the three recycling categories | An apple or another organic item |

Each bin initially contains objects belonging to its own category as well as objects that do not. A possible arrangement is:

- **Glass bin:** several glass bottles, plus a metal item and a cardboard item.
- **Paper and cardboard bin:** several paper or cardboard items, plus a glass item and a metal item.
- **Metal bin:** several metal items, plus a glass item and a cardboard item.
- **Set-aside items:** at least one organic item starts in one or two of the three bins. The set-aside plate starts empty.

This is a conceptual arrangement, not a final inventory. The number, type, size and material of every object, the exact layout, and the number and position of the organic items remain to be chosen. Misplaced objects could initially sit on top of the other contents so they can be seen and reached; the final setup must define their positions and match a setup image.

The set-aside area represents items that cannot be classified into the three available bins. It is not itself an organic-waste bin: organic items are the initial examples, and the final task must specify which objects belong there.

## Intended behaviour

For each bin, the robot should inspect its contents and determine which objects belong there. It should leave correctly placed objects in their proper bin, move misplaced glass, paper/cardboard and metal objects into the matching bin, and place all specified out-of-category objects in the set-aside area.

The task should be phrased as a request to correct the sorting. The precise instruction, bin labels, how the items' categories can be recognized from camera images, and whether the relevant sorting rules are given explicitly remain open decisions. A camera image alone does not necessarily establish an object's material, so the final scene must make the categories identifiable without relying on unseen information.

The user's wording includes “only the ones that are already outside.” The working interpretation is **only move items that are outside their correct destination**, including items currently inside the wrong bin. Whether objects should also start outside all bins needs clarification before finalizing the setup.

## Proposed scoring

The initial idea is four equally weighted checks, each worth **0.25**, for a total from 0 to 1:

| Check | Credit when the check is satisfied |
| --- | ---: |
| Glass | 0.25 |
| Paper and cardboard | 0.25 |
| Metal | 0.25 |
| Set-aside objects | 0.25 |

The intended final state is that all glass objects are in the glass bin, all paper and cardboard objects are in their bin, all metal objects are in the metal bin, and all designated set-aside objects are on or inside the set-aside plate or tray. A check earns its 0.25 only if its eventual, precisely specified conditions are satisfied; otherwise it earns 0. The total would be the sum of the four checks.

The user suggested a margin of error for the set-aside objects. The actual tolerance, including how to tell whether an item is on the plate, must be defined numerically after the geometry is chosen. The same applies to deciding whether an item is inside a bin.

The scoring rules still need to resolve:

- Whether a category check requires its destination to contain **only** correct items as well as all of its own items, and how to score an item in the wrong destination. This prevents accidentally awarding full credit to a contaminated bin.
- Whether an object outside all destinations, a knocked-over bin, damaged or removed objects, or a later failure cancels a category's credit or the entire score.
- Whether credit is based on the final state or must also account for violations during the trial. Any global failure rule would be an additional decision, not part of the current four-check proposal.
- Whether each category contains at least one initially misplaced item, so leaving the scene untouched does not receive unintended credit for a category.
- Whether smaller partial steps within a category deserve credit, or each category remains an all-or-nothing 0.25 check.

The final scorer should cover every outcome unambiguously and be checkable using the stated positions and tolerances. Scoring and shortcut cases remain untested.

## Details to settle before a task can be built

- Establish whether the task is sufficiently distinct from the closest existing work.
- Confirm the intended meaning of “only the ones that are already outside” and the exact set-aside category.
- Specify the complete fixed inventory, bin and object dimensions and materials, starting poses, labels and reproducible layout.
- Define the scoring conditions, tolerances, failure cases, run limit and reset procedure.
- Check whether the camera can see the relevant objects and whether the robot can reach and grasp them in the proposed arrangement.
- Check the idea against the team's simulator capabilities and the competition's review criteria before any instance run.

No feasibility test, GPU probe, Dream-1 run or verified full-score demonstration has been performed for this idea. Offline tests use stand-ins for the simulator API and check only the scoring logic and candidate move sequence.
