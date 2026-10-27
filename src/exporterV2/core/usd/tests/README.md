# USD core tests

Unit tests for the USD authoring primitives in `src/exporterV2/core/usd/`:
sphere rigid bodies, terminal-body collision filters, collision validation,
the rigid (welded) trunk and the runtime physics settings.

Run them on their own from the project root (their imports clash with the
optimizer tests when collected in the same pytest run):

```bash
uv run pytest -q src/exporterV2/core/usd/tests
```

Tests that need Isaac Sim are skipped without it.
