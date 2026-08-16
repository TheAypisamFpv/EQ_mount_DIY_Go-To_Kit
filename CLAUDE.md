# EQ Mount DIY Go-To Kit - working agreement

This repo lost real work once already because changes sat uncommitted in the working tree,
got silently bundled into an unrelated commit, and were then wiped out by reverting that
commit. The rules below exist specifically to make that impossible to repeat.

## Always commit

- **Commit at the end of every change**, not just when explicitly asked to. "Implement X" or
  "fix Y" includes committing the result - don't leave finished work sitting uncommitted.
- Commit **each logically distinct change separately** rather than batching unrelated work into
  one commit. If a commit's message needs "and" to describe it, it's probably two commits.
- Before starting new work, check `git status`/`git diff` for anything already sitting
  uncommitted that isn't yours. If found, commit or otherwise resolve it *first*, on its own,
  before adding more changes on top - never let unrelated pre-existing work ride along inside a
  commit for something else. That mixing is exactly what caused the lost-work incident above:
  reverting one feature reverted an unrelated one bundled into the same commit.
- Never revert a commit to undo one change if that commit also contains unrelated changes -
  separate them first (e.g. restore the file(s) to the target commit's content for review, or
  cherry-pick/diff surgically) rather than reverting the whole thing.
- Stay on `main` unless the user asks for a branch. Don't create branches unprompted.

## Bump the version on every functional change

- **Python GUI** (`tracker_gui.py`): bump `GUI_VERSION` and add a one-line changelog entry in
  the comment block right above it, same format as the entries already there.
- **Arduino firmware** (`EQMountTracker/EQMountTracker.ino`): bump `FIRMWARE_VERSION` and add a
  changelog entry in the versioned comment block above its `#define`, same format as the
  existing `1.8.x` entries (what changed, why, and any behavioral consequence).
- Bump **only** the side(s) that actually changed - a GUI-only change bumps `GUI_VERSION` alone;
  a firmware-only change bumps `FIRMWARE_VERSION` alone. Don't bump the untouched side.
- The version bump is part of the same commit as the change it describes, not a follow-up.

## Why this matters here

This is a hobby telescope mount project with real hardware in the loop - a lost or silently
reverted change isn't just wasted GUI work, it can also mean firmware behavior differs from what
the user thinks is flashed, or a Python/Arduino protocol mismatch nobody notices until the mount
moves wrong. Committing promptly and bumping versions is the cheap insurance against that.
