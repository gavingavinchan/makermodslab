New waterproof Viscous Arm: viscous_waterproof_01

Completed: CAN IDs 1-7 saved; folded/CLOSED mechanical zero saved; seven manual ROM captures; 0.05 rad stop backoff; 200ms watchdog persisted; MIT verified after power cycle. No powered teleoperation has been performed on this arm.

Lab backend: http://100.64.91.4:8000
Select robot viscous_waterproof_01. Same purple Star leader, copied calibration ID star_viscous_waterproof_01, with its existing origins unchanged and new range metadata. Both USB ports detected and folded leader mapped-pose check passed.

Hardware profiles: /home/gavin/.makermods/makermodslab-viscous/viscous_profiles/viscous_waterproof_01.json and star_viscous_waterproof_01.json
Lab loads these by calibration ID. They carry this follower's measured limits and new leader mapping, with all additive offsets explicitly zero. Calibration JSON alone does not override preset joint limits. Never select old viscous_01 for this new arm.

Runtime follower calibration: /home/gavin/.cache/huggingface/lerobot/calibration/robots/viscous_follower/viscous_waterproof_01.json
Runtime leader calibration: /home/gavin/.cache/huggingface/lerobot/calibration/teleoperators/rebot_102_leader/star_viscous_waterproof_01.json
Backups in lerobot-calibration/ under this folder; old unit-1 calibration files unchanged.

J2 forward clearance enlarged; rear folded stop unchanged (Gavin).
J4 logo restricts rearward motion; see j4-logo-clearance-note.txt. Remeasure J4 and update limits/mapping if logo clearance changes.

Lab retains the installed driver's startup sync (1 degree target increment per joint per loop, 3 degree tolerance) and 85C stop. It is not a degrees-per-second speed limit. The driver releases stalled joints from startup sync; ordinary tracking afterward uses the existing preset. Gains remain untuned. Assemble/mount/support the arm, fold both arms and clear hands/tools before powered testing. Lab stop releases torque: support/fold first.

Do not rerun follower calibration/zero on this commissioned arm. ID/zero/watchdog persisted to actuator flash; software profiles live on disk. SDK YAML has kp/kd=0 for capture and is not the powered runtime profile. This Lab change is deployed on Linux only; the Mac installation has not been updated.

Policy rollout remains unverified: its subprocess CLI path does not consume Lab's hardware profile loader; provide explicit matching driver limits before considering policy execution.
