# Project Memory Index

Updated: 2026-07-02

This repository now stores project memory in `omx_wiki/`. Keep these files under git so Isaac Sim hand/arm decisions survive context resets and future sessions.

Current pages:

- `amazinghand-isaacsim.md`: Isaac Sim AmazingHand conversion decisions, validation evidence, and known limits.
- `log.md`: chronological project memory log.

Memory policy:

- Add important debugging conclusions, validation artifact paths, and user-facing decisions here before committing related code.
- Keep transient Isaac outputs out of memory unless their paths are needed as evidence.
- Do not treat report `PASS` alone as enough for visual tasks; record image/contact-sheet inspection results.
