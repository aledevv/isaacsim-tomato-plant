# Collision Checks

The exporter controls collisions in two layers: `FilteredPairs` authored together with the PhysX bodies, and an overlap audit of the finished stage (`collisions.py`).

## Authored Filtered Pairs

While authoring the articulated structure, the exporter adds `FilteredPairs` relationships for:

- parent-child link pairs;
- branch attachments that start close to the parent surface;
- pedicel/truss connections;
- detachable tomato to pedicel and to truss rachis (only with `fruits.mode: physical`).

The detachable tomato filters matter because the tomato is outside the articulation. Without them, contact resolution near the breakable FixedJoint can dominate the intended detachment behavior.

## Rest-Pose Overlap Audit

The skeleton is used as given; the exporter does not move branches to avoid collisions. After the stage is built, `collider_overlaps` tests every pair of colliders on different rigid bodies (AABB pre-check, then capsule segment distance against the summed radii). Colliders that already touch at rest would push each other apart when the simulation starts.

`collisions.initial_overlaps` in `config/physics.yaml` decides what happens to overlaps that are not already filtered:

- `filter` (default): `auto_filter_initial_overlaps` adds one permanent filter per overlapping body pair and records each pair, its depth and contacts in the manifest;
- `error`: no extra filters are added, and the export fails the audit with "N collider overlaps at rest are not filtered".

## Validation

The overlap audit runs during every export in the regular `uv` environment (it only needs `pxr`). Runtime contact behavior can only be checked in Isaac Sim, for example with `./run_main.sh --headless`.
