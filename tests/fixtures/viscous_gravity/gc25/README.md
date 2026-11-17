# Archived 25% gravity-feedforward incident

These are offline evidence from the dry-air baseline `rehearse-live-20261117T024005Z`
and compensated run `rehearse-live-20261117T090749Z` (source b9dccb60).
`sources.json` records SHA256s of the full original tick files and archived fit.

The JSONL files retain every approach/hold row's original timestamp/phase and
seven motors' `pos`, `cmd`, `torque`, plus compensated `ff`/`w`, without rounding.
Temperatures, stop rows and per-joint weights are omitted. The gripper position
is retained because its configuration differed between runs. The original
fit is copied verbatim, intentionally unsafe for commanding: it must be refused
by the command loader. The regression constructs it directly only for an offline
fake-bus replay.

See [the incident analysis](../../../../docs/gravity-gc25.md) and
`tests/test_viscous_gravity_gc25.py`. Tests need neither a Mac path nor the full
run archive. No fixture represents a new hardware measurement or a validated fit.
