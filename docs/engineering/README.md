# Engineering records

The working record of the project. Design lives in [requirements](../requirements.md) and
[architecture](../architecture.md); this folder holds what happened, what is broken, what is next,
and the evidence behind decisions.

| File | Contents | Convention |
|---|---|---|
| [log.md](log.md) | What was done, decided or learned | Dated `##` headings, newest first. A few lines per entry; link to spikes and commits rather than repeating them. |
| [defects.md](defects.md) | Known defects and limitations of what exists now | `DEF-n` IDs are never reused. Fixed entries move to the *Fixed* section with the fix noted. |
| [todo.md](todo.md) | Work queue | Grouped by phase, highest priority first. Done items are deleted; the log records them. |
| [handoff.md](handoff.md) | Where the work stands when a session ends: bring-up on another machine, the fleet, open threads, working agreements, recipes | Overwritten at each handoff; the log keeps the history. |
| [spikes/](spikes/) | One file per investigation, `YYYY-MM-DD-topic.md` | Question → method → how to reproduce → results → conclusion. Results are not edited after the fact; a re-run gets a new section or a new spike. |
