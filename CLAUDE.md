# navgrid repo rules

## Commits

Messages are terse: 1–2 sentences saying what changed. No `Co-Authored-By` or other trailers, no
conventional-commit prefixes (`feat:`, `chore:`), no emoji or boilerplate.

## Engineering records

`docs/engineering/` is the working record (conventions in its README):

- `log.md`: dated entries, newest first. What was done, decided or learned.
- `defects.md`: known defects and limitations of what exists now.
- `todo.md`: the work queue, by phase.
- `spikes/`: one file per investigation, `YYYY-MM-DD-topic.md`.

Update the matching file when work lands, a defect is found or fixed, or an investigation
concludes. Design stays in `docs/requirements.md` and `docs/architecture.md`; evidence goes in
spikes.

## Build and check

- Host (repo root): `cargo test --workspace`, `cargo clippy --workspace --all-targets`,
  `cargo fmt --all --check`.
- Firmware (`cd firmware`, separate workspace): run `cargo clippy --release --features board-tag`
  and the same with `board-dk`. Flash with `cargo tag` / `cargo dk`.
