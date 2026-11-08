# Sim run details

Evidence: [sim_summary.json](sim_summary.json). Both virtual arms are replayed independently. O/C/R means ok/clamped/rejected. Policy rows exclude the synthetic approach. Joint error is the maximum absolute error across seven motors for each row. Error cells show median / maximum. Time cells show mean / p99 / maximum. A jump refusal has no binding face; see the JSON reasons and clipped-joint counts.

| Run / side | O / C / R | Binding link / face | Longest non-ok rows (s) | Joint error (deg) | Probe error (mm) | Step time (ms) |
| --- | --- | --- | --- | --- | --- | --- |
| flange/dry/B0/flange/ep_00 / left | 0 / 0 / 192 | jump filter | 192 (9.60) | 120.71 / 120.71 | 283.93 / 283.93 | 0.0052 / 0.0070 / 0.0086 |
| flange/dry/B0/flange/ep_00 / right | 0 / 0 / 192 | jump filter | 192 (9.60) | 120.71 / 120.71 | 283.93 / 283.93 | 0.0054 / 0.0080 / 0.0098 |
| flange_dream/dry_pregrasp/B0/flange/ep_00 / left | 0 / 0 / 192 | jump filter | 192 (9.60) | 51.47 / 51.47 | 272.77 / 274.34 | 0.0052 / 0.0068 / 0.0078 |
| flange_dream/dry_pregrasp/B0/flange/ep_00 / right | 0 / 0 / 192 | jump filter | 192 (9.60) | 53.56 / 53.83 | 256.06 / 256.53 | 0.0052 / 0.0062 / 0.0081 |
| flange_dream2/dry/B0/flange/G1/ep_00 / left | 0 / 0 / 192 | jump filter | 192 (9.60) | 51.49 / 51.49 | 273.57 / 275.53 | 0.0050 / 0.0068 / 0.0073 |
| flange_dream2/dry/B0/flange/G1/ep_00 / right | 0 / 0 / 192 | jump filter | 192 (9.60) | 54.06 / 54.33 | 256.56 / 256.85 | 0.0052 / 0.0079 / 0.0082 |
| flange_dream2/dry/B0/flange/G2/ep_00 / left | 0 / 0 / 192 | jump filter | 192 (9.60) | 51.49 / 51.49 | 273.57 / 275.53 | 0.0052 / 0.0067 / 0.0087 |
| flange_dream2/dry/B0/flange/G2/ep_00 / right | 0 / 0 / 192 | jump filter | 192 (9.60) | 54.06 / 54.33 | 256.56 / 256.85 | 0.0074 / 0.0120 / 0.3258 |
| flange_dream2/dry/B0/flange/G3/ep_00 / left | 0 / 0 / 192 | jump filter | 192 (9.60) | 120.71 / 120.71 | 283.93 / 283.93 | 0.0051 / 0.0064 / 0.0073 |
| flange_dream2/dry/B0/flange/G3/ep_00 / right | 0 / 0 / 192 | jump filter | 192 (9.60) | 54.15 / 54.42 | 257.67 / 258.03 | 0.0051 / 0.0073 / 0.0082 |
| flange_dream2/dry/B0/flange/T/ep_00 / left | 0 / 0 / 192 | jump filter | 192 (9.60) | 120.71 / 120.71 | 283.93 / 283.93 | 0.0055 / 0.0100 / 0.0119 |
| flange_dream2/dry/B0/flange/T/ep_00 / right | 0 / 0 / 192 | jump filter | 192 (9.60) | 120.71 / 120.71 | 283.93 / 283.93 | 0.0049 / 0.0058 / 0.0066 |
| flange_dream2/episodes/B0/flange/G1/ep_00 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 86.30 / 93.60 | 273.13 / 310.13 | 0.0052 / 0.0083 / 0.0694 |
| flange_dream2/episodes/B0/flange/G1/ep_00 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 72.34 / 74.99 | 463.63 / 483.73 | 0.0053 / 0.0062 / 0.0766 |
| flange_dream2/episodes/B0/flange/G1/ep_01 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 52.28 / 59.62 | 212.98 / 277.34 | 0.0054 / 0.0080 / 0.0742 |
| flange_dream2/episodes/B0/flange/G1/ep_01 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 59.24 / 72.10 | 345.07 / 363.44 | 0.0053 / 0.0073 / 0.0843 |
| flange_dream2/episodes/B0/flange/G2/ep_00 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 58.23 / 105.46 | 253.04 / 275.29 | 0.0052 / 0.0061 / 0.0848 |
| flange_dream2/episodes/B0/flange/G2/ep_00 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 57.98 / 64.07 | 208.51 / 261.97 | 0.0050 / 0.0087 / 0.0832 |
| flange_dream2/episodes/B0/flange/G2/ep_01 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 57.43 / 104.33 | 248.37 / 299.03 | 0.0079 / 0.0075 / 3.7314 |
| flange_dream2/episodes/B0/flange/G2/ep_01 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 53.12 / 60.97 | 205.59 / 259.25 | 0.0052 / 0.0079 / 0.0765 |
| flange_dream2/episodes/B0/flange/G3/ep_00 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 103.25 / 120.71 | 256.26 / 284.58 | 0.0073 / 0.0070 / 3.2465 |
| flange_dream2/episodes/B0/flange/G3/ep_00 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 54.43 / 69.97 | 176.84 / 259.87 | 0.0053 / 0.0074 / 0.0800 |
| flange_dream2/episodes/B0/flange/G3/ep_01 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 111.81 / 120.71 | 267.74 / 284.66 | 0.0053 / 0.0065 / 0.0863 |
| flange_dream2/episodes/B0/flange/G3/ep_01 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 58.12 / 76.61 | 160.58 / 250.89 | 0.0054 / 0.0077 / 0.0787 |
| flange_dream2/episodes/B0/flange/G3/ep_02 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 100.51 / 120.71 | 252.20 / 284.15 | 0.0049 / 0.0061 / 0.0669 |
| flange_dream2/episodes/B0/flange/G3/ep_02 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 53.60 / 79.97 | 172.55 / 264.00 | 0.0050 / 0.0062 / 0.0681 |
| flange_dream2/episodes/B0/flange/G3/ep_03 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 97.99 / 120.71 | 243.33 / 284.26 | 0.0051 / 0.0071 / 0.0544 |
| flange_dream2/episodes/B0/flange/G3/ep_03 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 55.20 / 63.21 | 181.63 / 263.64 | 0.0048 / 0.0064 / 0.0846 |
| flange_dream2/episodes/B0/flange/T/ep_00 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 88.69 / 130.97 | 371.44 / 418.12 | 0.0156 / 0.0076 / 6.0014 |
| flange_dream2/episodes/B0/flange/T/ep_00 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 93.15 / 120.72 | 248.53 / 283.92 | 0.0050 / 0.0059 / 0.0731 |
| flange_dream2/episodes/B0/flange/T/ep_01 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 98.63 / 120.71 | 327.54 / 387.48 | 0.0054 / 0.0083 / 0.0652 |
| flange_dream2/episodes/B0/flange/T/ep_01 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 94.71 / 120.71 | 250.32 / 382.00 | 0.0047 / 0.0061 / 0.0724 |
| flange_v2/dry/B0/flange/ep_00 / left | 0 / 0 / 192 | jump filter | 192 (9.60) | 120.71 / 120.71 | 283.93 / 283.93 | 0.0049 / 0.0064 / 0.0067 |
| flange_v2/dry/B0/flange/ep_00 / right | 0 / 0 / 192 | jump filter | 192 (9.60) | 120.71 / 120.71 | 283.93 / 283.93 | 0.0054 / 0.0095 / 0.0229 |
| propeller/dry/B0/propeller/P1/ep_00 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 120.71 / 120.71 | 283.93 / 283.93 | 0.0077 / 0.0067 / 3.6881 |
| propeller/dry/B0/propeller/P1/ep_00 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 120.71 / 120.71 | 283.93 / 283.93 | 0.0051 / 0.0060 / 0.0696 |
| propeller/dry/B0/propeller/P2/ep_00 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 120.71 / 120.71 | 283.93 / 283.93 | 0.0052 / 0.0062 / 0.0774 |
| propeller/dry/B0/propeller/P2/ep_00 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 120.71 / 120.71 | 283.93 / 283.93 | 0.0053 / 0.0074 / 0.0765 |
| propeller/dry/B0/propeller/P3/ep_00 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 120.71 / 120.71 | 283.93 / 283.93 | 0.0054 / 0.0075 / 0.0696 |
| propeller/dry/B0/propeller/P3/ep_00 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 120.71 / 120.71 | 283.93 / 283.93 | 0.0050 / 0.0063 / 0.0808 |
| propeller/dry/B0/propeller/P4/ep_00 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 120.71 / 120.71 | 283.93 / 283.93 | 0.0054 / 0.0072 / 0.0985 |
| propeller/dry/B0/propeller/P4/ep_00 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 120.71 / 120.71 | 283.93 / 283.93 | 0.0054 / 0.0088 / 0.1373 |
| replay_ik_smoke_fixed / left | 0 / 0 / 48 | jump filter | 48 (2.40) | 119.73 / 120.71 | 283.80 / 283.94 | 0.0052 / 0.0072 / 0.0075 |
| replay_ik_smoke_fixed / right | 0 / 0 / 48 | jump filter | 48 (2.40) | 119.73 / 120.71 | 283.80 / 283.94 | 0.0051 / 0.0061 / 0.0062 |
| replay_yam_path_on_viscous / left | 0 / 0 / 720 | jump filter | 720 (36.00) | 89.48 / 120.81 | 463.61 / 584.50 | 0.0053 / 0.0071 / 0.1129 |
| replay_yam_path_on_viscous / right | 0 / 0 / 720 | jump filter | 720 (36.00) | 99.66 / 120.80 | 243.66 / 283.81 | 0.0086 / 0.0079 / 2.3295 |
| replay_yam_path_on_viscous_contact_check / left | 0 / 0 / 720 | jump filter | 720 (36.00) | 89.49 / 120.81 | 463.61 / 584.51 | 0.0052 / 0.0080 / 0.0842 |
| replay_yam_path_on_viscous_contact_check / right | 0 / 0 / 720 | jump filter | 720 (36.00) | 99.67 / 120.80 | 243.67 / 283.81 | 0.0052 / 0.0070 / 0.0948 |
| shackle_scene/dream/ep_00 / left | 0 / 19 / 1421 | forearm_link/rim: 103 | 1440 (72.00) | 68.01 / 75.27 | 79.57 / 134.58 | 0.9418 / 14.2901 / 16.7130 |
| shackle_scene/dream/ep_00 / right | 0 / 25 / 1415 | forearm_link/rim: 1440 | 1440 (72.00) | 13.42 / 13.53 | 76.06 / 76.67 | 9.5063 / 14.0308 / 17.2399 |
| shackle_scene/dream/ep_01 / left | 0 / 21 / 1419 | forearm_link/rim: 712 | 1440 (72.00) | 63.50 / 76.56 | 89.30 / 116.33 | 4.8644 / 14.0885 / 16.6542 |
| shackle_scene/dream/ep_01 / right | 0 / 25 / 1415 | forearm_link/rim: 1440 | 1440 (72.00) | 13.42 / 13.53 | 76.06 / 76.67 | 9.3661 / 14.6570 / 28.7255 |
| shackle_scene/dream/ep_02 / left | 0 / 21 / 1419 | forearm_link/rim: 67 | 1440 (72.00) | 62.19 / 78.05 | 93.11 / 127.62 | 0.6215 / 13.3081 / 16.7332 |
| shackle_scene/dream/ep_02 / right | 0 / 25 / 1415 | forearm_link/rim: 1440 | 1440 (72.00) | 13.42 / 13.53 | 76.06 / 76.67 | 9.4122 / 15.6630 / 29.0745 |
| shackle_scene/dry/ep_00 / left | 0 / 25 / 71 | forearm_link/rim: 96 | 96 (4.80) | 13.42 / 13.53 | 76.06 / 76.67 | 11.9889 / 13.1421 / 14.5909 |
| shackle_scene/dry/ep_00 / right | 0 / 25 / 71 | forearm_link/rim: 96 | 96 (4.80) | 13.42 / 13.53 | 76.06 / 76.67 | 13.6190 / 16.3776 / 16.5043 |
| sweep/B0/apple/ep_00 / left | 0 / 0 / 528 | jump filter | 528 (26.40) | 92.11 / 120.69 | 453.81 / 557.62 | 0.0054 / 0.0066 / 0.0320 |
| sweep/B0/apple/ep_00 / right | 0 / 0 / 528 | jump filter | 528 (26.40) | 120.10 / 120.69 | 288.41 / 292.51 | 0.0050 / 0.0062 / 0.0297 |
| sweep/B0/apple/ep_01 / left | 0 / 0 / 672 | jump filter | 672 (33.60) | 72.09 / 120.69 | 455.16 / 532.43 | 0.0051 / 0.0064 / 0.0083 |
| sweep/B0/apple/ep_01 / right | 0 / 0 / 672 | jump filter | 672 (33.60) | 110.38 / 120.70 | 268.12 / 289.52 | 0.0051 / 0.0061 / 0.0063 |
| sweep/B0/apple/ep_02 / left | 0 / 0 / 624 | jump filter | 624 (31.20) | 69.54 / 120.67 | 472.21 / 559.26 | 0.0027 / 0.0050 / 0.0163 |
| sweep/B0/apple/ep_02 / right | 0 / 0 / 624 | jump filter | 624 (31.20) | 116.08 / 120.69 | 277.50 / 288.98 | 0.0026 / 0.0056 / 0.0150 |
| sweep/B0/apple/ep_03 / left | 0 / 0 / 336 | jump filter | 336 (16.80) | 94.02 / 120.69 | 432.46 / 491.90 | 0.0026 / 0.0034 / 0.0047 |
| sweep/B0/apple/ep_03 / right | 0 / 0 / 336 | jump filter | 336 (16.80) | 113.97 / 120.69 | 278.99 / 291.28 | 0.0028 / 0.0065 / 0.0084 |
| sweep/B0/apple/ep_04 / left | 0 / 0 / 720 | jump filter | 720 (36.00) | 74.20 / 120.71 | 437.72 / 561.16 | 0.0029 / 0.0070 / 0.0673 |
| sweep/B0/apple/ep_04 / right | 0 / 0 / 720 | jump filter | 720 (36.00) | 118.04 / 120.69 | 278.50 / 291.30 | 0.0026 / 0.0048 / 0.0679 |
| sweep/B0/apple/ep_05 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 85.56 / 120.68 | 457.53 / 515.13 | 0.0026 / 0.0035 / 0.0566 |
| sweep/B0/apple/ep_05 / right | 0 / 0 / 864 | jump filter | 864 (43.20) | 117.39 / 120.70 | 275.42 / 289.43 | 0.0025 / 0.0049 / 0.0369 |
| sweep/B0/box/ep_00 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 119.02 / 144.22 | 476.85 / 512.90 | 0.0026 / 0.0052 / 0.0355 |
| sweep/B0/box/ep_00 / right | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 89.69 / 120.70 | 255.34 / 286.90 | 0.0025 / 0.0036 / 0.0374 |
| sweep/B0/box/ep_01 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 93.88 / 120.69 | 436.82 / 494.85 | 0.0026 / 0.0036 / 0.0607 |
| sweep/B0/box/ep_01 / right | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 99.69 / 120.70 | 254.90 / 286.84 | 0.0024 / 0.0032 / 0.0278 |
| sweep/B0/box/ep_02 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 76.34 / 120.69 | 463.36 / 486.90 | 0.0027 / 0.0055 / 0.0360 |
| sweep/B0/box/ep_02 / right | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 92.31 / 120.70 | 261.36 / 286.81 | 0.0026 / 0.0041 / 0.0399 |
| sweep/B0/box/ep_03 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 78.74 / 120.69 | 440.23 / 481.07 | 0.0026 / 0.0040 / 0.0520 |
| sweep/B0/box/ep_03 / right | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 91.14 / 120.70 | 272.39 / 286.76 | 0.0024 / 0.0031 / 0.0248 |
| sweep/B0/box/ep_04 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 128.26 / 147.12 | 473.65 / 507.23 | 0.0025 / 0.0048 / 0.0410 |
| sweep/B0/box/ep_04 / right | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 85.95 / 120.70 | 291.08 / 300.74 | 0.0026 / 0.0049 / 0.0486 |
| sweep/B0/box/ep_05 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 124.85 / 137.42 | 457.87 / 490.06 | 0.0026 / 0.0035 / 0.0564 |
| sweep/B0/box/ep_05 / right | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 77.43 / 120.70 | 282.97 / 299.62 | 0.0025 / 0.0050 / 0.0282 |
| sweep/B0/propeller/P1/ep_00 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 110.75 / 124.72 | 283.62 / 444.60 | 0.0025 / 0.0032 / 0.0406 |
| sweep/B0/propeller/P1/ep_00 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 90.89 / 120.67 | 240.78 / 288.40 | 0.0026 / 0.0053 / 0.0428 |
| sweep/B0/propeller/P1/ep_01 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 91.24 / 120.71 | 395.12 / 434.13 | 0.0026 / 0.0034 / 0.0413 |
| sweep/B0/propeller/P1/ep_01 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 98.16 / 120.67 | 257.76 / 288.09 | 0.0025 / 0.0035 / 0.0452 |
| sweep/B0/propeller/P2/ep_00 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 72.04 / 126.56 | 253.78 / 529.99 | 0.0026 / 0.0049 / 0.0383 |
| sweep/B0/propeller/P2/ep_00 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 103.78 / 120.64 | 270.52 / 295.49 | 0.0025 / 0.0030 / 0.0312 |
| sweep/B0/propeller/P2/ep_01 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 113.73 / 130.40 | 394.16 / 476.22 | 0.0026 / 0.0051 / 0.0452 |
| sweep/B0/propeller/P2/ep_01 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 97.99 / 120.64 | 265.96 / 292.89 | 0.0025 / 0.0048 / 0.0391 |
| sweep/B0/propeller/P3/ep_00 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 60.08 / 120.69 | 318.17 / 442.89 | 0.0025 / 0.0038 / 0.0455 |
| sweep/B0/propeller/P3/ep_00 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 102.12 / 120.69 | 269.82 / 294.03 | 0.0026 / 0.0042 / 0.0541 |
| sweep/B0/propeller/P3/ep_01 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 118.30 / 133.81 | 418.08 / 457.09 | 0.0025 / 0.0030 / 0.0536 |
| sweep/B0/propeller/P3/ep_01 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 92.96 / 120.69 | 265.21 / 288.49 | 0.0025 / 0.0031 / 0.0422 |
| sweep/B0/propeller/P3/ep_02 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 121.11 / 132.57 | 437.40 / 493.74 | 0.0026 / 0.0050 / 0.0356 |
| sweep/B0/propeller/P3/ep_02 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 107.08 / 120.69 | 274.94 / 289.05 | 0.0025 / 0.0037 / 0.0414 |
| sweep/B0/propeller/P3/ep_03 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 121.66 / 145.01 | 289.50 / 446.95 | 0.0025 / 0.0031 / 0.0297 |
| sweep/B0/propeller/P3/ep_03 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 102.11 / 120.69 | 260.90 / 289.32 | 0.0025 / 0.0030 / 0.0350 |
| sweep/B0/propeller/P3/ep_04 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 91.98 / 121.83 | 296.18 / 436.09 | 0.0026 / 0.0032 / 0.0401 |
| sweep/B0/propeller/P3/ep_04 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 103.78 / 120.69 | 260.99 / 288.57 | 0.0025 / 0.0030 / 0.0345 |
| sweep/B0/propeller/P4/ep_00 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 121.93 / 134.17 | 416.55 / 463.74 | 0.0025 / 0.0033 / 0.0391 |
| sweep/B0/propeller/P4/ep_00 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 93.11 / 120.69 | 255.00 / 290.00 | 0.0028 / 0.0068 / 0.0574 |
| sweep/B0/propeller/P4/ep_01 / left | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 63.62 / 120.71 | 406.50 / 441.04 | 0.0026 / 0.0066 / 0.0855 |
| sweep/B0/propeller/P4/ep_01 / right | 0 / 0 / 1440 | jump filter | 1440 (72.00) | 101.85 / 120.68 | 256.54 / 286.56 | 0.0026 / 0.0041 / 0.0666 |
| sweep/parked/apple/ep_00 / left | 0 / 0 / 528 | jump filter | 528 (26.40) | 81.42 / 120.68 | 452.57 / 526.08 | 0.0026 / 0.0043 / 0.0198 |
| sweep/parked/apple/ep_00 / right | 0 / 25 / 503 | forearm_link/rim: 528 | 528 (26.40) | 13.42 / 13.53 | 76.06 / 76.67 | 8.8585 / 10.3548 / 11.8820 |
| sweep/parked/apple/ep_01 / left | 0 / 0 / 480 | jump filter | 480 (24.00) | 79.60 / 120.69 | 425.05 / 558.31 | 0.0028 / 0.0060 / 0.0274 |
| sweep/parked/apple/ep_01 / right | 0 / 25 / 455 | forearm_link/rim: 480 | 480 (24.00) | 13.42 / 13.53 | 76.06 / 76.67 | 8.8387 / 10.3301 / 12.5934 |
| sweep/parked/apple/ep_02 / left | 0 / 0 / 384 | jump filter | 384 (19.20) | 88.58 / 120.69 | 447.54 / 587.07 | 0.0027 / 0.0050 / 0.0340 |
| sweep/parked/apple/ep_02 / right | 0 / 25 / 359 | forearm_link/rim: 384 | 384 (19.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.8361 / 11.3038 / 12.0619 |
| sweep/parked/apple/ep_03 / left | 0 / 0 / 432 | jump filter | 432 (21.60) | 81.39 / 120.68 | 430.88 / 576.23 | 0.0027 / 0.0052 / 0.0282 |
| sweep/parked/apple/ep_03 / right | 0 / 25 / 407 | forearm_link/rim: 432 | 432 (21.60) | 13.42 / 13.53 | 76.06 / 76.67 | 8.8718 / 10.5392 / 12.0780 |
| sweep/parked/apple/ep_04 / left | 0 / 0 / 432 | jump filter | 432 (21.60) | 82.18 / 120.68 | 448.96 / 570.01 | 0.0027 / 0.0055 / 0.0271 |
| sweep/parked/apple/ep_04 / right | 0 / 25 / 407 | forearm_link/rim: 432 | 432 (21.60) | 13.42 / 13.53 | 76.06 / 76.67 | 8.7668 / 10.2367 / 11.3902 |
| sweep/parked/apple/ep_05 / left | 0 / 0 / 384 | jump filter | 384 (19.20) | 79.85 / 120.70 | 440.61 / 568.06 | 0.0025 / 0.0034 / 0.0244 |
| sweep/parked/apple/ep_05 / right | 0 / 25 / 359 | forearm_link/rim: 384 | 384 (19.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.5926 / 11.4620 / 12.9782 |
| sweep/parked_blind/apple/ep_00 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 72.87 / 120.70 | 483.25 / 615.61 | 0.0029 / 0.0069 / 0.0874 |
| sweep/parked_blind/apple/ep_00 / right | 0 / 25 / 839 | forearm_link/rim: 864 | 864 (43.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.7849 / 10.3073 / 12.3166 |
| sweep/parked_blind/apple/ep_01 / left | 0 / 0 / 576 | jump filter | 576 (28.80) | 67.18 / 120.68 | 437.62 / 461.54 | 0.0026 / 0.0033 / 0.0185 |
| sweep/parked_blind/apple/ep_01 / right | 0 / 25 / 551 | forearm_link/rim: 576 | 576 (28.80) | 13.42 / 13.53 | 76.06 / 76.67 | 8.6554 / 9.9855 / 12.6102 |
| sweep/parked_blind/apple/ep_02 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 111.34 / 123.82 | 436.45 / 478.14 | 0.0026 / 0.0040 / 0.0396 |
| sweep/parked_blind/apple/ep_02 / right | 0 / 25 / 839 | forearm_link/rim: 864 | 864 (43.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.7623 / 10.4701 / 12.0439 |
| sweep/parked_blind/apple/ep_03 / left | 0 / 0 / 432 | jump filter | 432 (21.60) | 85.90 / 120.70 | 430.68 / 555.09 | 0.0026 / 0.0038 / 0.0256 |
| sweep/parked_blind/apple/ep_03 / right | 0 / 25 / 407 | forearm_link/rim: 432 | 432 (21.60) | 13.42 / 13.53 | 76.06 / 76.67 | 8.7706 / 10.2577 / 12.4030 |
| sweep/parked_blind/apple/ep_04 / left | 0 / 0 / 384 | jump filter | 384 (19.20) | 77.28 / 120.69 | 417.88 / 567.07 | 0.0026 / 0.0041 / 0.0239 |
| sweep/parked_blind/apple/ep_04 / right | 0 / 25 / 359 | forearm_link/rim: 384 | 384 (19.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.6534 / 10.4601 / 11.5567 |
| sweep/parked_blind/apple/ep_05 / left | 0 / 0 / 480 | jump filter | 480 (24.00) | 86.14 / 120.70 | 436.01 / 564.58 | 0.0028 / 0.0082 / 0.0235 |
| sweep/parked_blind/apple/ep_05 / right | 0 / 25 / 455 | forearm_link/rim: 480 | 480 (24.00) | 13.42 / 13.53 | 76.06 / 76.67 | 8.6059 / 10.6271 / 12.3353 |
| sweep/r24/box/ep_00 / left | 0 / 0 / 576 | jump filter | 576 (28.80) | 98.16 / 129.71 | 556.50 / 604.96 | 0.0026 / 0.0052 / 0.0180 |
| sweep/r24/box/ep_00 / right | 0 / 0 / 576 | jump filter | 576 (28.80) | 110.55 / 120.70 | 282.53 / 290.86 | 0.0026 / 0.0035 / 0.0187 |
| sweep/r24/box/ep_01 / left | 0 / 0 / 576 | jump filter | 576 (28.80) | 119.07 / 141.45 | 452.10 / 507.66 | 0.0024 / 0.0035 / 0.0144 |
| sweep/r24/box/ep_01 / right | 0 / 0 / 576 | jump filter | 576 (28.80) | 109.42 / 120.70 | 280.15 / 292.28 | 0.0025 / 0.0034 / 0.0160 |
| sweep/r24/box/ep_02 / left | 0 / 0 / 576 | jump filter | 576 (28.80) | 114.06 / 126.96 | 438.34 / 510.83 | 0.0025 / 0.0037 / 0.0045 |
| sweep/r24/box/ep_02 / right | 0 / 0 / 576 | jump filter | 576 (28.80) | 112.73 / 120.70 | 277.64 / 289.46 | 0.0026 / 0.0034 / 0.0181 |
| sweep/r24/box/ep_03 / left | 0 / 0 / 576 | jump filter | 576 (28.80) | 115.14 / 147.12 | 442.84 / 511.91 | 0.0024 / 0.0032 / 0.0038 |
| sweep/r24/box/ep_03 / right | 0 / 0 / 576 | jump filter | 576 (28.80) | 110.97 / 120.70 | 281.57 / 291.25 | 0.0024 / 0.0033 / 0.0137 |
| sweep/single/apple/ep_00 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 98.03 / 120.66 | 315.61 / 353.48 | 0.0024 / 0.0031 / 0.0280 |
| sweep/single/apple/ep_00 / right | 0 / 25 / 839 | forearm_link/rim: 864 | 864 (43.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.6992 / 12.2538 / 14.0615 |
| sweep/single/apple/ep_01 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 104.26 / 120.66 | 321.14 / 350.78 | 0.0024 / 0.0030 / 0.0282 |
| sweep/single/apple/ep_01 / right | 0 / 25 / 839 | forearm_link/rim: 864 | 864 (43.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.8796 / 11.8285 / 12.7351 |
| sweep/single/apple/ep_02 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 75.10 / 120.66 | 420.28 / 472.53 | 0.0025 / 0.0033 / 0.0072 |
| sweep/single/apple/ep_02 / right | 0 / 25 / 839 | forearm_link/rim: 864 | 864 (43.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.7147 / 12.3535 / 13.5080 |
| sweep/single/apple/ep_03 / left | 0 / 0 / 768 | jump filter | 768 (38.40) | 94.07 / 128.49 | 425.93 / 484.41 | 0.0026 / 0.0041 / 0.0395 |
| sweep/single/apple/ep_03 / right | 0 / 25 / 743 | forearm_link/rim: 768 | 768 (38.40) | 13.42 / 13.53 | 76.06 / 76.67 | 8.9079 / 12.0571 / 13.3675 |
| sweep/single/apple/ep_04 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 85.35 / 120.66 | 456.74 / 514.55 | 0.0026 / 0.0031 / 0.0260 |
| sweep/single/apple/ep_04 / right | 0 / 25 / 839 | forearm_link/rim: 864 | 864 (43.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.9382 / 12.0682 / 12.7598 |
| sweep/single/apple/ep_05 / left | 0 / 0 / 576 | jump filter | 576 (28.80) | 73.96 / 120.66 | 408.67 / 563.00 | 0.0038 / 0.0074 / 0.0364 |
| sweep/single/apple/ep_05 / right | 0 / 25 / 551 | forearm_link/rim: 576 | 576 (28.80) | 13.42 / 13.53 | 76.06 / 76.67 | 9.0114 / 12.3594 / 13.3276 |
| sweep/single/box/ep_00 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 80.47 / 120.72 | 438.24 / 471.93 | 0.0024 / 0.0028 / 0.0226 |
| sweep/single/box/ep_00 / right | 0 / 25 / 1127 | forearm_link/rim: 1152 | 1152 (57.60) | 13.42 / 13.53 | 76.06 / 76.67 | 8.8449 / 12.5951 / 13.7665 |
| sweep/single/box/ep_01 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 71.69 / 120.71 | 468.03 / 498.16 | 0.0024 / 0.0028 / 0.0218 |
| sweep/single/box/ep_01 / right | 0 / 25 / 1127 | forearm_link/rim: 1152 | 1152 (57.60) | 13.42 / 13.53 | 76.06 / 76.67 | 8.8318 / 12.1340 / 13.7160 |
| sweep/single/box/ep_02 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 98.72 / 145.02 | 340.70 / 402.98 | 0.0025 / 0.0032 / 0.0605 |
| sweep/single/box/ep_02 / right | 0 / 25 / 1127 | forearm_link/rim: 1152 | 1152 (57.60) | 13.42 / 13.53 | 76.06 / 76.67 | 8.8271 / 11.5314 / 12.2519 |
| sweep/single/box/ep_03 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 100.70 / 120.71 | 429.27 / 469.20 | 0.0039 / 0.0108 / 0.0629 |
| sweep/single/box/ep_03 / right | 0 / 25 / 1127 | forearm_link/rim: 1152 | 1152 (57.60) | 13.42 / 13.53 | 76.06 / 76.67 | 8.7425 / 10.8476 / 12.6070 |
| sweep/single/box/ep_04 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 99.84 / 120.71 | 300.42 / 496.80 | 0.0025 / 0.0048 / 0.0350 |
| sweep/single/box/ep_04 / right | 0 / 25 / 1127 | forearm_link/rim: 1152 | 1152 (57.60) | 13.42 / 13.53 | 76.06 / 76.67 | 8.6338 / 11.3282 / 15.8114 |
| sweep/single/box/ep_05 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 83.58 / 145.94 | 446.45 / 466.35 | 0.0026 / 0.0042 / 0.0372 |
| sweep/single/box/ep_05 / right | 0 / 25 / 1127 | forearm_link/rim: 1152 | 1152 (57.60) | 13.42 / 13.53 | 76.06 / 76.67 | 8.7723 / 10.7609 / 13.0396 |
| sweep/single_copy/apple/ep_00 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 92.49 / 117.68 | 395.49 / 443.48 | 0.0026 / 0.0054 / 0.0362 |
| sweep/single_copy/apple/ep_00 / right | 0 / 25 / 839 | forearm_link/rim: 864 | 864 (43.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.7791 / 11.0786 / 13.5674 |
| sweep/single_copy/apple/ep_01 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 140.00 / 147.01 | 423.61 / 457.91 | 0.0026 / 0.0052 / 0.0397 |
| sweep/single_copy/apple/ep_01 / right | 0 / 25 / 839 | forearm_link/rim: 864 | 864 (43.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.7864 / 10.3161 / 11.9643 |
| sweep/single_copy/apple/ep_02 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 79.94 / 120.71 | 407.91 / 430.66 | 0.0024 / 0.0026 / 0.0261 |
| sweep/single_copy/apple/ep_02 / right | 0 / 25 / 839 | forearm_link/rim: 864 | 864 (43.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.7491 / 10.2242 / 12.7330 |
| sweep/single_copy/apple/ep_03 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 70.41 / 120.72 | 459.26 / 565.07 | 0.0026 / 0.0038 / 0.0473 |
| sweep/single_copy/apple/ep_03 / right | 0 / 25 / 839 | forearm_link/rim: 864 | 864 (43.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.5959 / 10.4455 / 13.8238 |
| sweep/single_copy/apple/ep_04 / left | 0 / 0 / 576 | jump filter | 576 (28.80) | 98.21 / 120.71 | 389.67 / 473.55 | 0.0025 / 0.0030 / 0.0162 |
| sweep/single_copy/apple/ep_04 / right | 0 / 25 / 551 | forearm_link/rim: 576 | 576 (28.80) | 13.42 / 13.53 | 76.06 / 76.67 | 8.7730 / 10.2732 / 11.6590 |
| sweep/single_copy/apple/ep_05 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 92.66 / 120.72 | 425.97 / 454.94 | 0.0024 / 0.0032 / 0.0054 |
| sweep/single_copy/apple/ep_05 / right | 0 / 25 / 839 | forearm_link/rim: 864 | 864 (43.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.7595 / 10.3788 / 12.4726 |
| sweep/single_fixedcam/apple/ep_00 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 117.45 / 118.62 | 279.42 / 311.84 | 0.0025 / 0.0042 / 0.0391 |
| sweep/single_fixedcam/apple/ep_00 / right | 0 / 25 / 839 | forearm_link/rim: 864 | 864 (43.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.7932 / 10.3697 / 12.2003 |
| sweep/single_fixedcam/apple/ep_01 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 116.37 / 120.71 | 283.00 / 305.48 | 0.0024 / 0.0032 / 0.0255 |
| sweep/single_fixedcam/apple/ep_01 / right | 0 / 25 / 839 | forearm_link/rim: 864 | 864 (43.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.5888 / 10.6582 / 14.6389 |
| sweep/single_fixedcam/apple/ep_02 / left | 0 / 0 / 576 | jump filter | 576 (28.80) | 95.68 / 120.71 | 398.35 / 513.62 | 0.0025 / 0.0031 / 0.0159 |
| sweep/single_fixedcam/apple/ep_02 / right | 0 / 25 / 551 | forearm_link/rim: 576 | 576 (28.80) | 13.42 / 13.53 | 76.06 / 76.67 | 8.8024 / 10.4412 / 12.1735 |
| sweep/single_fixedcam/apple/ep_03 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 71.21 / 120.71 | 455.27 / 511.32 | 0.0023 / 0.0027 / 0.0284 |
| sweep/single_fixedcam/apple/ep_03 / right | 0 / 25 / 839 | forearm_link/rim: 864 | 864 (43.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.7841 / 10.7050 / 12.8246 |
| sweep/single_fixedcam/apple/ep_04 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 76.78 / 120.71 | 409.96 / 459.82 | 0.0024 / 0.0028 / 0.0260 |
| sweep/single_fixedcam/apple/ep_04 / right | 0 / 25 / 839 | forearm_link/rim: 864 | 864 (43.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.7554 / 10.3540 / 12.3029 |
| sweep/single_fixedcam/apple/ep_05 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 111.67 / 120.71 | 277.12 / 307.45 | 0.0025 / 0.0033 / 0.0386 |
| sweep/single_fixedcam/apple/ep_05 / right | 0 / 25 / 839 | forearm_link/rim: 864 | 864 (43.20) | 13.42 / 13.53 | 76.06 / 76.67 | 8.7076 / 10.2347 / 13.9130 |
| sweep/table/box/ep_00 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 102.51 / 114.53 | 338.00 / 348.21 | 0.0025 / 0.0033 / 0.0392 |
| sweep/table/box/ep_00 / right | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 97.62 / 118.35 | 250.37 / 285.64 | 0.0025 / 0.0037 / 0.0407 |
| sweep/table/box/ep_01 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 101.60 / 111.71 | 311.34 / 348.96 | 0.0024 / 0.0034 / 0.0255 |
| sweep/table/box/ep_01 / right | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 97.59 / 120.70 | 264.56 / 286.84 | 0.0025 / 0.0036 / 0.0403 |
| sweep/table/box/ep_02 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 102.07 / 141.27 | 311.29 / 386.92 | 0.0025 / 0.0031 / 0.0455 |
| sweep/table/box/ep_02 / right | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 94.61 / 120.70 | 247.51 / 286.81 | 0.0025 / 0.0033 / 0.0182 |
| sweep/table/box/ep_03 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 146.30 / 146.33 | 418.09 / 437.03 | 0.0025 / 0.0030 / 0.0248 |
| sweep/table/box/ep_03 / right | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 86.85 / 120.70 | 227.03 / 286.76 | 0.0024 / 0.0031 / 0.0233 |
| sweep/tip/apple/ep_00 / left | 0 / 0 / 576 | jump filter | 576 (28.80) | 72.78 / 117.51 | 442.30 / 489.49 | 0.0025 / 0.0033 / 0.0160 |
| sweep/tip/apple/ep_00 / right | 0 / 0 / 576 | jump filter | 576 (28.80) | 112.11 / 117.48 | 276.11 / 289.11 | 0.0024 / 0.0029 / 0.0131 |
| sweep/tip/apple/ep_01 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 73.16 / 120.68 | 436.98 / 503.17 | 0.0024 / 0.0031 / 0.0395 |
| sweep/tip/apple/ep_01 / right | 0 / 0 / 864 | jump filter | 864 (43.20) | 110.29 / 120.69 | 267.64 / 289.24 | 0.0024 / 0.0030 / 0.0258 |
| sweep/tip/apple/ep_02 / left | 0 / 0 / 672 | jump filter | 672 (33.60) | 81.99 / 120.67 | 447.75 / 523.19 | 0.0025 / 0.0032 / 0.0043 |
| sweep/tip/apple/ep_02 / right | 0 / 0 / 672 | jump filter | 672 (33.60) | 109.66 / 120.69 | 270.47 / 288.90 | 0.0026 / 0.0067 / 0.0093 |
| sweep/tip/apple/ep_03 / left | 0 / 0 / 864 | jump filter | 864 (43.20) | 77.72 / 120.69 | 457.13 / 505.05 | 0.0026 / 0.0070 / 0.0419 |
| sweep/tip/apple/ep_03 / right | 0 / 0 / 864 | jump filter | 864 (43.20) | 110.82 / 120.69 | 269.23 / 291.21 | 0.0026 / 0.0033 / 0.0047 |
| sweep/tip/apple/ep_04 / left | 0 / 0 / 336 | jump filter | 336 (16.80) | 81.99 / 120.71 | 428.46 / 474.09 | 0.0025 / 0.0031 / 0.0039 |
| sweep/tip/apple/ep_04 / right | 0 / 0 / 336 | jump filter | 336 (16.80) | 119.68 / 120.69 | 285.74 / 290.89 | 0.0026 / 0.0033 / 0.0059 |
| sweep/tip/apple/ep_05 / left | 0 / 0 / 528 | jump filter | 528 (26.40) | 90.09 / 120.68 | 433.79 / 563.75 | 0.0025 / 0.0035 / 0.0167 |
| sweep/tip/apple/ep_05 / right | 0 / 0 / 528 | jump filter | 528 (26.40) | 110.51 / 120.69 | 267.88 / 288.92 | 0.0024 / 0.0029 / 0.0171 |
| sweep/tip/box/ep_00 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 119.18 / 133.16 | 444.79 / 483.91 | 0.0025 / 0.0031 / 0.0257 |
| sweep/tip/box/ep_00 / right | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 82.74 / 120.70 | 281.68 / 306.86 | 0.0025 / 0.0032 / 0.0724 |
| sweep/tip/box/ep_01 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 131.82 / 147.43 | 492.24 / 558.87 | 0.0026 / 0.0049 / 0.0587 |
| sweep/tip/box/ep_01 / right | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 109.37 / 120.70 | 259.09 / 286.77 | 0.0025 / 0.0033 / 0.0290 |
| sweep/tip/box/ep_02 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 127.34 / 148.01 | 495.98 / 567.46 | 0.0025 / 0.0033 / 0.0302 |
| sweep/tip/box/ep_02 / right | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 107.08 / 120.70 | 266.32 / 286.76 | 0.0024 / 0.0030 / 0.0190 |
| sweep/tip/box/ep_03 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 102.07 / 138.21 | 448.88 / 472.37 | 0.0025 / 0.0032 / 0.0491 |
| sweep/tip/box/ep_03 / right | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 112.64 / 120.70 | 271.43 / 338.28 | 0.0025 / 0.0033 / 0.0254 |
| sweep/tip/box/ep_04 / left | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 120.77 / 147.12 | 487.82 / 532.50 | 0.0025 / 0.0038 / 0.0402 |
| sweep/tip/box/ep_04 / right | 0 / 0 / 1152 | jump filter | 1152 (57.60) | 76.79 / 120.70 | 278.49 / 309.09 | 0.0026 / 0.0066 / 0.0379 |
| sweep/tip/box/ep_05 / left | 0 / 0 / 672 | jump filter | 672 (33.60) | 88.09 / 121.06 | 438.71 / 493.83 | 0.0028 / 0.0079 / 0.0090 |
| sweep/tip/box/ep_05 / right | 0 / 0 / 672 | jump filter | 672 (33.60) | 98.62 / 120.70 | 270.23 / 286.66 | 0.0025 / 0.0033 / 0.0045 |
| viscous_apple_makerwrist_live1 / left | 0 / 0 / 960 | jump filter | 960 (48.00) | 92.10 / 117.43 | 469.94 / 593.93 | 0.0026 / 0.0036 / 0.0288 |
| viscous_apple_makerwrist_live1 / right | 0 / 0 / 960 | jump filter | 960 (48.00) | 103.61 / 117.54 | 265.82 / 325.93 | 0.0026 / 0.0047 / 0.0641 |
| viscous_apple_vision_live2 / left | 0 / 0 / 720 | jump filter | 720 (36.00) | 108.44 / 133.46 | 253.07 / 317.75 | 0.0025 / 0.0036 / 0.0343 |
| viscous_apple_vision_live2 / right | 0 / 0 / 720 | jump filter | 720 (36.00) | 98.34 / 120.72 | 288.64 / 308.28 | 0.0026 / 0.0035 / 0.0384 |
| viscous_box_makerwrist_live1 / left | 0 / 0 / 960 | jump filter | 960 (48.00) | 113.77 / 135.48 | 477.95 / 544.07 | 0.0025 / 0.0032 / 0.0049 |
| viscous_box_makerwrist_live1 / right | 0 / 0 / 960 | jump filter | 960 (48.00) | 103.89 / 117.85 | 256.54 / 283.45 | 0.0024 / 0.0028 / 0.0263 |

# Start gates and paths

Extents list x; y; z, in mm, after the assumed virtual-base rotation. Probe means FK of the exact executed targets. Raw EEF means Dream’s saved Cartesian request; its TCP differs from the physical fingertip probe. Missing raw EEF values belong to local hold stubs. The JSON also gives each plane’s worst slack.

| Run / side | First gate: link / face (slack mm) | Approach O / C / R | Requested probe xyz extents (mm) | Raw Dream EEF xyz extents (mm) |
| --- | --- | --- | --- | --- |
| flange/dry/B0/flange/ep_00 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-256.2; 0.0..0.0; 124.5..124.5 | -188.2..-188.2; 117.2..117.2; 10.9..10.9 |
| flange/dry/B0/flange/ep_00 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-256.2; 0.0..0.0; 124.5..124.5 | -188.2..-188.2; -492.8..-492.8; 10.9..10.9 |
| flange_dream/dry_pregrasp/B0/flange/ep_00 / left | forearm_link / near (-105.38) | 72 / 32 / 32 | -183.8..-174.9; 112.0..133.7; 137.6..182.1 | -185.4..-176.2; 114.3..136.4; 135.0..180.0 |
| flange_dream/dry_pregrasp/B0/flange/ep_00 / right | forearm_link / near (-108.85) | 0 / 98 / 34 | -182.0..-181.5; -140.2..-135.2; 183.1..194.7 | -185.4..-176.2; -495.7..-473.6; 135.0..180.0 |
| flange_dream2/dry/B0/flange/G1/ep_00 / left | forearm_link / near (-108.10) | 72 / 32 / 32 | -182.3..-173.3; 112.5..134.2; 135.4..180.0 | -183.9..-174.6; 114.8..136.8; 132.8..177.9 |
| flange_dream2/dry/B0/flange/G1/ep_00 / right | forearm_link / near (-111.91) | 0 / 98 / 35 | -182.2..-181.7; -141.5..-136.5; 179.5..191.2 | -183.9..-174.6; -495.2..-473.2; 132.8..177.9 |
| flange_dream2/dry/B0/flange/G2/ep_00 / left | forearm_link / near (-108.10) | 72 / 32 / 32 | -182.3..-173.3; 112.5..134.2; 135.4..180.0 | -183.9..-174.6; 114.8..136.8; 132.8..177.9 |
| flange_dream2/dry/B0/flange/G2/ep_00 / right | forearm_link / near (-111.91) | 0 / 98 / 35 | -182.2..-181.7; -141.5..-136.5; 179.5..191.2 | -183.9..-174.6; -495.2..-473.2; 132.8..177.9 |
| flange_dream2/dry/B0/flange/G3/ep_00 / left | forearm_link / near (-209.98) | 0 / 101 / 101 | -256.2..-256.2; -0.0..-0.0; 124.5..124.5 | -364.5..-364.5; 225.0..225.0; 145.2..145.2 |
| flange_dream2/dry/B0/flange/G3/ep_00 / right | forearm_link / near (-110.91) | 0 / 98 / 35 | -182.8..-182.3; -141.8..-136.8; 180.6..192.3 | -364.5..-364.5; -385.0..-385.0; 145.2..145.2 |
| flange_dream2/dry/B0/flange/T/ep_00 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-256.2; 0.0..0.0; 124.5..124.5 | -293.6..-293.5; 176.8..176.8; 10.9..10.9 |
| flange_dream2/dry/B0/flange/T/ep_00 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-256.2; 0.0..0.0; 124.5..124.5 | -293.6..-293.5; -433.2..-433.2; 10.9..10.9 |
| flange_dream2/episodes/B0/flange/G1/ep_00 / left | forearm_link / near (-108.10) | 16 / 88 / 32 | -189.9..-147.0; -14.6..134.1; 76.8..183.0 | -182.1..-137.4; -47.0..112.5; 94.0..206.9 |
| flange_dream2/episodes/B0/flange/G1/ep_00 / right | forearm_link / near (-111.92) | 0 / 98 / 35 | -242.1..-182.2; -509.6..-141.5; 65.1..208.6 | -233.8..-161.0; -486.7..-110.8; 71.6..229.0 |
| flange_dream2/episodes/B0/flange/G1/ep_01 / left | forearm_link / near (-108.88) | 43 / 59 / 32 | -273.9..-183.8; 104.8..163.8; 59.1..185.1 | -271.6..-164.1; 68.2..144.2; 71.0..208.2 |
| flange_dream2/episodes/B0/flange/G1/ep_01 / right | forearm_link / near (-109.03) | 0 / 98 / 34 | -324.6..-184.5; -386.1..-141.4; 22.9..203.1 | -314.8..-164.8; -367.9..-110.0; 36.4..225.0 |
| flange_dream2/episodes/B0/flange/G2/ep_00 / left | forearm_link / near (-108.10) | 0 / 104 / 32 | -285.6..-181.2; 74.3..190.1; 3.3..179.9 | -282.2..-161.4; 56.1..197.0; 31.4..204.3 |
| flange_dream2/episodes/B0/flange/G2/ep_00 / right | forearm_link / near (-111.92) | 0 / 98 / 35 | -284.3..-177.6; -144.5..-77.0; 189.0..253.7 | -264.0..-158.5; -126.4..-66.3; 203.0..279.2 |
| flange_dream2/episodes/B0/flange/G2/ep_01 / left | forearm_link / near (-108.88) | 0 / 102 / 32 | -288.8..-183.7; -11.7..169.7; -23.4..179.6 | -280.3..-165.1; -7.2..149.7; 7.1..203.4 |
| flange_dream2/episodes/B0/flange/G2/ep_01 / right | forearm_link / near (-109.03) | 0 / 98 / 34 | -281.8..-181.9; -163.5..-132.9; 193.2..272.4 | -262.8..-162.1; -150.9..-107.0; 208.3..295.3 |
| flange_dream2/episodes/B0/flange/G3/ep_00 / left | forearm_link / near (-209.99) | 0 / 101 / 101 | -284.6..-256.1; -27.9..3.1; 122.8..171.2 | -256.9..-214.0; -26.0..12.5; 122.3..179.1 |
| flange_dream2/episodes/B0/flange/G3/ep_00 / right | forearm_link / near (-110.91) | 0 / 98 / 35 | -342.7..-181.8; -256.5..-141.8; 192.4..241.8 | -333.5..-162.7; -233.4..-108.6; 202.5..263.1 |
| flange_dream2/episodes/B0/flange/G3/ep_01 / left | forearm_link / near (-209.99) | 0 / 101 / 101 | -293.0..-256.0; -12.7..4.4; 123.2..171.7 | -266.3..-214.0; -14.5..15.4; 120.8..179.8 |
| flange_dream2/episodes/B0/flange/G3/ep_01 / right | forearm_link / near (-108.40) | 0 / 99 / 33 | -365.1..-183.0; -248.9..-139.4; 173.2..208.1 | -354.4..-162.1; -226.0..-106.5; 183.6..230.0 |
| flange_dream2/episodes/B0/flange/G3/ep_02 / left | forearm_link / near (-209.99) | 0 / 101 / 101 | -288.8..-256.2; -22.1..4.1; 124.1..157.4 | -261.4..-215.4; -20.8..12.2; 123.4..164.3 |
| flange_dream2/episodes/B0/flange/G3/ep_02 / right | forearm_link / near (-109.78) | 0 / 97 / 36 | -359.8..-180.6; -251.6..-141.9; 153.9..211.1 | -353.4..-160.6; -226.9..-111.7; 162.9..233.0 |
| flange_dream2/episodes/B0/flange/G3/ep_03 / left | forearm_link / near (-209.98) | 0 / 101 / 101 | -292.3..-256.2; -23.8..4.7; 124.0..166.3 | -263.9..-214.9; -22.9..12.8; 123.3..175.0 |
| flange_dream2/episodes/B0/flange/G3/ep_03 / right | forearm_link / near (-113.56) | 0 / 98 / 36 | -359.0..-180.0; -257.3..-140.1; 172.2..210.1 | -355.6..-159.5; -234.3..-109.1; 181.5..231.5 |
| flange_dream2/episodes/B0/flange/T/ep_00 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -320.3..-172.5; -0.0..165.9; -18.9..147.4 | -320.5..-154.0; -3.9..156.2; 6.1..171.7 |
| flange_dream2/episodes/B0/flange/T/ep_00 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -271.0..-254.6; -8.7..4.6; 124.5..280.7 | -245.7..-214.7; -9.4..11.9; 127.5..291.6 |
| flange_dream2/episodes/B0/flange/T/ep_01 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -344.1..-173.8; -0.1..191.4; -19.0..176.2 | -331.0..-160.1; -3.9..181.4; 11.2..200.4 |
| flange_dream2/episodes/B0/flange/T/ep_01 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -304.8..-254.0; -238.6..4.5; 20.0..287.9 | -291.9..-213.6; -226.2..12.9; 38.6..296.6 |
| flange_v2/dry/B0/flange/ep_00 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-256.2; 0.0..0.0; 124.5..124.5 | -293.6..-293.5; 176.8..176.8; 10.9..10.9 |
| flange_v2/dry/B0/flange/ep_00 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-256.2; 0.0..0.0; 124.5..124.5 | -293.6..-293.5; -433.2..-433.2; 10.9..10.9 |
| propeller/dry/B0/propeller/P1/ep_00 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-256.2; 0.0..0.0; 124.5..124.5 | -400.0..-400.0; 280.0..280.0; 173.0..173.0 |
| propeller/dry/B0/propeller/P1/ep_00 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-256.2; 0.0..0.0; 124.5..124.5 | -400.0..-400.0; -330.0..-330.0; 173.0..173.0 |
| propeller/dry/B0/propeller/P2/ep_00 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-256.2; 0.0..0.0; 124.5..124.5 | -400.0..-400.0; 280.0..280.0; 173.0..173.0 |
| propeller/dry/B0/propeller/P2/ep_00 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-256.2; 0.0..0.0; 124.5..124.5 | -400.0..-400.0; -330.0..-330.0; 173.0..173.0 |
| propeller/dry/B0/propeller/P3/ep_00 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-256.2; 0.0..0.0; 124.5..124.5 | -400.0..-400.0; 280.0..280.0; 173.0..173.0 |
| propeller/dry/B0/propeller/P3/ep_00 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-256.2; 0.0..0.0; 124.5..124.5 | -400.0..-400.0; -330.0..-330.0; 173.0..173.0 |
| propeller/dry/B0/propeller/P4/ep_00 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-256.2; 0.0..0.0; 124.5..124.5 | -400.0..-400.0; 280.0..280.0; 173.0..173.0 |
| propeller/dry/B0/propeller/P4/ep_00 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-256.2; 0.0..0.0; 124.5..124.5 | -400.0..-400.0; -330.0..-330.0; 173.0..173.0 |
| replay_ik_smoke_fixed / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-256.0; 0.0..0.0; 124.5..125.2 | -222.3..-222.3; 0.0..0.0; 130.0..130.0 |
| replay_ik_smoke_fixed / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-256.0; 0.0..0.0; 124.5..125.2 | -222.3..-222.3; 0.0..0.0; 130.0..130.0 |
| replay_yam_path_on_viscous / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -407.8..-102.7; -1.7..337.3; -14.0..292.8 | -391.9..-99.0; -8.2..329.4; 11.0..347.5 |
| replay_yam_path_on_viscous / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -283.2..-256.3; -43.5..-0.0; 125.1..200.8 | -256.3..-236.7; -43.0..6.3; 145.6..209.8 |
| replay_yam_path_on_viscous_contact_check / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -407.8..-102.7; -1.7..337.3; -14.0..292.8 | -391.9..-99.0; -8.2..329.4; 11.0..347.5 |
| replay_yam_path_on_viscous_contact_check / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -283.2..-256.3; -43.5..-0.0; 125.1..200.8 | -256.3..-236.7; -43.0..6.3; 145.6..209.8 |
| shackle_scene/dream/ep_00 / left | forearm_link / rim (-19.18) | 103 / 37 / 0 | -246.4..-138.3; 0.0..68.9; 65.8..150.2 | -247.3..-139.9; -2.2..61.1; 100.0..211.0 |
| shackle_scene/dream/ep_00 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -186.5..-165.2; -13.2..5.7; 90.4..112.7 |
| shackle_scene/dream/ep_01 / left | forearm_link / rim (-19.16) | 103 / 37 / 0 | -235.4..-174.9; 0.0..98.5; 60.7..115.3 | -237.9..-166.8; -5.6..107.9; 89.4..159.3 |
| shackle_scene/dream/ep_01 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -188.3..-167.0; -13.2..4.6; 90.4..114.7 |
| shackle_scene/dream/ep_02 / left | forearm_link / rim (-19.16) | 103 / 37 / 0 | -232.3..-174.7; -0.1..75.9; 65.8..123.6 | -233.5..-168.7; -7.3..76.5; 91.4..167.5 |
| shackle_scene/dream/ep_02 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -186.5..-168.7; -13.2..5.1; 92.5..114.7 |
| shackle_scene/dry/ep_00 / left | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | not recorded |
| shackle_scene/dry/ep_00 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | not recorded |
| sweep/B0/apple/ep_00 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -293.1..-91.0; 0.1..317.6; 12.6..148.1 | -295.7..-96.0; -3.9..302.2; 31.2..182.0 |
| sweep/B0/apple/ep_00 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -256.2..-237.0; -22.7..4.7; 113.6..158.6 | -231.0..-196.8; -24.0..9.8; 113.8..170.2 |
| sweep/B0/apple/ep_01 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -276.6..-122.0; 0.0..329.6; 12.2..125.5 | -272.6..-109.7; -3.9..313.5; 35.9..145.8 |
| sweep/B0/apple/ep_01 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -268.0..-253.3; -17.3..4.0; 115.8..178.5 | -240.0..-212.8; -18.4..8.7; 116.0..186.9 |
| sweep/B0/apple/ep_02 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -271.5..-127.3; 0.0..323.7; -7.3..124.4 | -271.7..-116.2; -3.9..305.7; 18.3..145.3 |
| sweep/B0/apple/ep_02 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -256.2..-244.7; -11.9..4.6; 116.8..184.0 | -227.5..-205.3; -12.7..9.7; 116.4..191.4 |
| sweep/B0/apple/ep_03 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -283.1..-129.4; 0.1..348.4; 3.4..151.0 | -282.8..-113.4; -3.9..325.1; 22.5..169.6 |
| sweep/B0/apple/ep_03 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -259.8..-253.8; -1.8..4.9; 113.1..150.4 | -235.4..-213.8; -4.9..12.0; 113.5..163.4 |
| sweep/B0/apple/ep_04 / left | forearm_link / near (-209.99) | 48 / 53 / 101 | -260.7..-111.0; 0.1..313.4; 10.8..124.4 | -264.4..-116.9; -7.3..305.4; 34.6..149.2 |
| sweep/B0/apple/ep_04 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -256.2..-239.4; -15.6..5.9; 113.3..178.6 | -228.5..-197.6; -18.3..11.4; 110.3..185.9 |
| sweep/B0/apple/ep_05 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -289.6..-114.8; 0.0..306.5; -12.0..167.0 | -295.8..-112.1; -5.6..296.2; 16.6..210.9 |
| sweep/B0/apple/ep_05 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -258.6..-247.8; -22.0..3.5; 116.0..183.7 | -234.0..-207.0; -24.3..9.0; 116.4..195.1 |
| sweep/B0/box/ep_00 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -257.8..-77.0; -3.4..222.2; -10.9..158.7 | -230.7..-86.0; -9.7..214.0; 10.4..173.9 |
| sweep/B0/box/ep_00 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -274.5..-243.0; -61.6..0.0; 120.7..334.2 | -250.0..-202.8; -58.6..4.4; 122.1..342.3 |
| sweep/B0/box/ep_01 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -304.4..-138.5; -4.4..325.1; -30.0..130.7 | -306.5..-130.2; -10.7..309.9; -2.5..139.7 |
| sweep/B0/box/ep_01 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -267.6..-249.1; -53.8..0.0; 120.6..295.7 | -243.1..-209.0; -50.4..6.5; 122.6..303.0 |
| sweep/B0/box/ep_02 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -324.3..-136.1; -4.3..324.8; -13.6..167.1 | -328.1..-124.9; -11.0..310.2; 9.3..180.2 |
| sweep/B0/box/ep_02 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -259.6..-237.5; -72.7..0.0; 120.6..359.5 | -232.7..-200.2; -68.5..6.3; 121.2..364.9 |
| sweep/B0/box/ep_03 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -351.9..-133.7; -6.2..298.6; -26.8..173.9 | -352.3..-115.9; -11.1..289.9; -1.5..190.2 |
| sweep/B0/box/ep_03 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-234.6; -58.5..0.0; 120.4..358.8 | -231.3..-194.7; -55.9..6.5; 121.7..364.2 |
| sweep/B0/box/ep_04 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -258.5..-75.0; -5.4..153.0; -16.2..172.5 | -230.4..-83.6; -11.2..157.8; 10.2..188.0 |
| sweep/B0/box/ep_04 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-215.6; -42.3..0.0; 120.9..374.2 | -229.4..-174.1; -42.3..8.0; 121.2..380.3 |
| sweep/B0/box/ep_05 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -260.9..-111.8; -7.4..128.0; -21.9..184.2 | -235.9..-122.4; -11.3..131.8; 4.7..207.3 |
| sweep/B0/box/ep_05 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-221.3; -61.7..0.0; 120.9..383.0 | -231.2..-183.1; -57.9..7.2; 122.1..389.2 |
| sweep/B0/propeller/P1/ep_00 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -494.5..-160.3; -12.8..117.6; -39.6..211.4 | -485.7..-151.1; -21.2..117.5; -17.8..279.3 |
| sweep/B0/propeller/P1/ep_00 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -322.0..-252.8; -75.3..0.3; 116.5..295.0 | -294.4..-211.9; -76.6..5.4; 117.9..305.8 |
| sweep/B0/propeller/P1/ep_01 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -416.7..-171.6; -5.3..257.9; -33.5..320.2 | -397.2..-168.1; -9.7..238.5; -2.7..337.5 |
| sweep/B0/propeller/P1/ep_01 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -276.6..-249.7; -10.6..11.4; 116.2..242.6 | -249.1..-211.4; -12.8..19.0; 115.2..254.2 |
| sweep/B0/propeller/P2/ep_00 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -474.1..-60.4; -7.3..173.8; -28.7..272.3 | -464.2..-68.1; -9.1..164.6; 2.6..290.6 |
| sweep/B0/propeller/P2/ep_00 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -275.4..-245.6; -14.7..14.2; 108.8..270.4 | -249.0..-207.6; -14.5..22.2; 112.4..281.2 |
| sweep/B0/propeller/P2/ep_01 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -376.7..-157.3; -3.8..251.3; -38.4..203.5 | -371.3..-162.9; -8.5..226.8; -11.3..277.6 |
| sweep/B0/propeller/P2/ep_01 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -283.4..-251.0; -3.5..16.3; 110.2..177.4 | -256.4..-212.3; -8.1..22.6; 111.7..194.8 |
| sweep/B0/propeller/P3/ep_00 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -395.5..-154.4; -2.2..143.7; -20.5..211.8 | -370.9..-145.7; -7.3..139.8; 8.3..239.7 |
| sweep/B0/propeller/P3/ep_00 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -279.4..-252.8; -10.4..18.1; 111.7..235.4 | -253.4..-214.8; -7.4..24.8; 113.3..246.1 |
| sweep/B0/propeller/P3/ep_01 / left | forearm_link / near (-209.99) | 48 / 53 / 101 | -367.2..-152.4; -4.5..144.4; -18.2..186.5 | -345.9..-144.3; -9.4..143.9; 9.7..221.0 |
| sweep/B0/propeller/P3/ep_01 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -293.9..-252.7; 0.0..26.2; 117.3..220.4 | -267.3..-212.2; -5.3..33.3; 118.1..230.8 |
| sweep/B0/propeller/P3/ep_02 / left | forearm_link / near (-209.99) | 48 / 53 / 101 | -307.2..-128.7; -4.9..196.0; -25.7..134.2 | -315.4..-132.9; -10.0..199.8; 6.8..207.0 |
| sweep/B0/propeller/P3/ep_02 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -264.2..-246.1; -17.5..11.4; 117.1..218.1 | -236.3..-205.7; -17.5..17.7; 118.4..229.7 |
| sweep/B0/propeller/P3/ep_03 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -383.7..-168.5; -5.2..124.4; -35.9..298.9 | -378.7..-160.1; -10.0..113.2; -7.4..322.9 |
| sweep/B0/propeller/P3/ep_03 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -273.3..-250.3; -4.1..21.8; 117.2..183.9 | -246.3..-210.6; -8.4..28.5; 116.7..196.0 |
| sweep/B0/propeller/P3/ep_04 / left | forearm_link / near (-209.99) | 48 / 53 / 101 | -463.1..-164.2; -4.4..155.9; -42.7..287.3 | -455.4..-155.3; -9.4..145.8; -11.8..307.0 |
| sweep/B0/propeller/P3/ep_04 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -286.6..-254.9; -31.0..12.3; 117.1..210.0 | -258.5..-216.0; -35.4..19.5; 116.7..222.6 |
| sweep/B0/propeller/P4/ep_00 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -395.5..-137.2; -2.2..139.8; -29.6..180.5 | -385.8..-126.9; -7.3..139.7; 1.8..231.5 |
| sweep/B0/propeller/P4/ep_00 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -313.2..-251.0; -32.5..15.3; 118.2..254.6 | -286.5..-210.8; -31.6..22.1; 119.7..267.1 |
| sweep/B0/propeller/P4/ep_01 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -415.4..-164.1; -3.9..287.9; -23.8..131.3 | -398.3..-160.0; -8.9..295.4; -12.8..155.0 |
| sweep/B0/propeller/P4/ep_01 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -295.0..-247.5; -76.0..2.8; 118.4..368.3 | -270.7..-207.3; -71.1..9.6; 119.9..373.4 |
| sweep/parked/apple/ep_00 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -276.8..-147.9; 0.1..331.1; 2.3..133.8 | -276.5..-136.3; -2.2..310.4; 18.2..161.6 |
| sweep/parked/apple/ep_00 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -186.5..-168.7; -16.2..2.9; 94.5..114.7 |
| sweep/parked/apple/ep_01 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -259.2..-127.3; 0.1..312.3; -9.0..154.4 | -251.0..-122.9; -3.9..295.6; 5.8..177.6 |
| sweep/parked/apple/ep_01 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -186.5..-168.7; -13.2..2.5; 94.5..114.7 |
| sweep/parked/apple/ep_02 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-109.5; 0.1..319.4; -11.8..124.4 | -238.4..-110.5; -3.9..306.1; 14.5..133.0 |
| sweep/parked/apple/ep_02 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -186.5..-168.7; -13.2..3.9; 96.5..117.8 |
| sweep/parked/apple/ep_03 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -267.3..-108.2; 0.1..312.4; -7.3..137.2 | -265.6..-114.5; -3.9..305.4; 19.4..165.7 |
| sweep/parked/apple/ep_03 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -186.5..-165.2; -13.2..3.3; 94.5..118.8 |
| sweep/parked/apple/ep_04 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-113.2; 0.1..316.2; 0.7..144.5 | -266.7..-108.1; -3.9..302.1; 21.0..184.0 |
| sweep/parked/apple/ep_04 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -186.5..-168.7; -13.2..2.5; 98.5..114.7 |
| sweep/parked/apple/ep_05 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-137.4; 0.1..330.8; 0.3..124.4 | -240.2..-128.5; -3.9..323.0; 18.2..131.1 |
| sweep/parked/apple/ep_05 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -186.5..-168.7; -16.2..2.1; 96.5..114.7 |
| sweep/parked_blind/apple/ep_00 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -259.1..-81.7; 0.0..368.1; -10.0..141.0 | -232.6..-85.1; -7.1..354.4; 12.0..172.0 |
| sweep/parked_blind/apple/ep_00 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -190.1..-122.4; -25.4..9.3; 81.3..112.7 |
| sweep/parked_blind/apple/ep_01 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -271.9..-169.9; 0.1..311.0; 10.2..124.4 | -270.0..-151.6; -1.7..298.4; 34.3..143.0 |
| sweep/parked_blind/apple/ep_01 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -193.7..-138.5; -16.2..8.2; 86.4..110.7 |
| sweep/parked_blind/apple/ep_02 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -264.6..-144.5; -0.4..202.7; 6.4..130.8 | -236.9..-135.8; -7.0..208.5; 31.5..159.6 |
| sweep/parked_blind/apple/ep_02 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -190.1..-136.7; -22.3..8.2; 82.3..109.7 |
| sweep/parked_blind/apple/ep_03 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -268.3..-125.4; -0.8..299.4; -37.7..130.1 | -269.3..-130.9; -7.3..291.5; -12.0..156.6 |
| sweep/parked_blind/apple/ep_03 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -190.1..-142.0; -14.7..9.5; 84.4..107.6 |
| sweep/parked_blind/apple/ep_04 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -267.8..-132.3; 0.1..300.2; -25.0..139.9 | -261.1..-133.4; -3.9..286.5; 0.7..155.9 |
| sweep/parked_blind/apple/ep_04 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -193.7..-138.5; -14.7..9.7; 82.3..114.7 |
| sweep/parked_blind/apple/ep_05 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -263.3..-119.5; -0.6..320.1; -25.4..126.3 | -260.9..-120.7; -7.2..313.2; 0.3..150.9 |
| sweep/parked_blind/apple/ep_05 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -190.1..-145.6; -14.7..8.2; 79.3..106.6 |
| sweep/r24/box/ep_00 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -263.3..-0.5; -5.2..245.9; -29.3..141.4 | -231.4..-6.1; -9.1..255.3; -4.2..133.0 |
| sweep/r24/box/ep_00 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.6..-245.2; -6.1..15.3; 120.7..180.4 | -231.4..-206.6; -9.4..24.5; 122.1..195.0 |
| sweep/r24/box/ep_01 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -347.8..-108.2; -7.0..195.2; -38.8..124.4 | -313.5..-125.0; -11.3..186.0; 7.7..171.6 |
| sweep/r24/box/ep_01 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-235.2; -21.7..5.5; 120.3..247.0 | -231.3..-199.4; -24.1..17.6; 122.0..257.6 |
| sweep/r24/box/ep_02 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -323.6..-93.4; -7.9..215.4; -32.1..133.0 | -319.7..-110.6; -13.3..191.5; -15.3..170.2 |
| sweep/r24/box/ep_02 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -260.2..-245.2; -11.9..0.0; 118.4..181.3 | -233.6..-208.3; -14.8..6.3; 120.1..190.6 |
| sweep/r24/box/ep_03 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -274.0..-114.4; -4.3..167.8; -36.1..126.4 | -239.2..-129.7; -10.0..175.5; 1.0..163.1 |
| sweep/r24/box/ep_03 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -280.4..-217.4; -16.9..17.4; 120.4..221.1 | -256.6..-187.0; -13.4..19.2; 121.7..229.5 |
| sweep/single/apple/ep_00 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -263.8..-244.3; -191.1..8.1; 55.0..124.3 | -242.1..-207.4; -188.7..16.3; 60.2..127.2 |
| sweep/single/apple/ep_00 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -188.3..119.7; -147.4..32.2; 13.4..165.3 |
| sweep/single/apple/ep_01 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -263.6..-231.7; -146.9..24.6; 64.5..124.4 | -243.3..-194.8; -149.5..29.6; 68.8..128.5 |
| sweep/single/apple/ep_01 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -188.3..103.7; -105.5..28.4; 25.6..133.9 |
| sweep/single/apple/ep_02 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -264.4..-177.3; 0.0..259.3; 12.8..135.2 | -250.0..-155.9; -7.3..240.1; 41.4..161.0 |
| sweep/single/apple/ep_02 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -193.7..-147.4; -13.2..6.8; 84.4..110.7 |
| sweep/single/apple/ep_03 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -283.7..-61.5; 0.0..287.0; 12.9..144.4 | -274.4..-40.5; -7.3..286.6; 39.1..179.1 |
| sweep/single/apple/ep_03 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -193.7..-148.3; -13.2..9.7; 84.4..114.7 |
| sweep/single/apple/ep_04 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -260.1..-86.8; 0.0..287.2; -6.5..139.0 | -237.6..-75.5; -10.1..268.9; 9.1..162.3 |
| sweep/single/apple/ep_04 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -193.7..-148.3; -13.2..8.2; 82.3..114.7 |
| sweep/single/apple/ep_05 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -264.5..-118.6; 0.0..305.6; 4.3..142.6 | -254.1..-103.8; -7.3..288.6; 21.7..165.4 |
| sweep/single/apple/ep_05 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -193.7..-137.6; -16.2..9.7; 78.3..114.7 |
| sweep/single/box/ep_00 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -263.7..-131.4; -6.6..155.6; 12.3..133.3 | -238.4..-113.2; -12.2..145.0; 35.5..146.6 |
| sweep/single/box/ep_00 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -193.7..-115.3; -28.4..8.2; 84.4..114.7 |
| sweep/single/box/ep_01 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -268.4..-133.0; -13.4..264.6; -22.0..141.4 | -246.6..-123.9; -19.1..249.6; 1.4..154.5 |
| sweep/single/box/ep_01 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -193.7..-110.9; -13.2..5.7; 80.3..114.7 |
| sweep/single/box/ep_02 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -270.5..-190.7; -189.4..18.2; -18.5..138.4 | -243.2..-149.7; -197.5..23.1; -14.9..154.2 |
| sweep/single/box/ep_02 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -188.3..71.6; -217.6..44.1; 25.6..217.9 |
| sweep/single/box/ep_03 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -265.5..-151.1; -19.6..292.7; -3.0..145.4 | -249.1..-128.0; -27.4..286.9; 15.4..160.1 |
| sweep/single/box/ep_03 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -193.7..-126.0; -16.2..7.8; 82.3..112.7 |
| sweep/single/box/ep_04 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -269.0..-106.8; -110.5..175.5; -19.0..140.6 | -242.8..-92.7; -112.1..159.9; 0.9..154.7 |
| sweep/single/box/ep_04 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -193.7..173.1; -71.1..23.5; 76.3..205.2 |
| sweep/single/box/ep_05 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -274.8..-85.9; -13.9..167.2; 19.7..220.9 | -247.4..-72.9; -18.8..161.0; 46.3..281.4 |
| sweep/single/box/ep_05 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -191.9..-102.9; -16.2..5.9; 83.4..110.7 |
| sweep/single_copy/apple/ep_00 / left | forearm_link / near (-211.58) | 33 / 69 / 98 | -257.8..-153.2; -4.0..186.7; 27.6..135.8 | -231.8..-132.6; -5.6..175.3; 40.5..153.3 |
| sweep/single_copy/apple/ep_00 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -215.0..-135.8; -19.3..234.0; 84.4..214.9 |
| sweep/single_copy/apple/ep_01 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -256.2..-115.7; -2.6..159.6; 50.6..129.0 | -229.2..-87.2; -7.3..151.3; 64.5..141.7 |
| sweep/single_copy/apple/ep_01 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -207.0..-142.0; -13.2..203.5; 80.3..158.3 |
| sweep/single_copy/apple/ep_02 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -256.2..-184.5; -2.2..193.9; 32.4..126.1 | -228.0..-158.3; -5.6..178.3; 50.9..134.8 |
| sweep/single_copy/apple/ep_02 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -222.1..-154.5; -16.2..218.8; 85.4..253.7 |
| sweep/single_copy/apple/ep_03 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -280.5..-97.8; -3.7..338.8; 2.9..141.3 | -273.3..-79.3; -7.3..322.3; 22.1..161.6 |
| sweep/single_copy/apple/ep_03 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -240.0..-138.5; -19.3..202.0; 38.8..240.3 |
| sweep/single_copy/apple/ep_04 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -294.7..-163.6; -2.5..301.8; 24.3..144.4 | -273.3..-138.1; -7.3..287.6; 41.8..165.2 |
| sweep/single_copy/apple/ep_04 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -229.2..-129.6; -22.3..177.6; 53.0..246.8 |
| sweep/single_copy/apple/ep_05 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -256.2..-131.9; -2.1..165.3; 41.0..128.1 | -228.0..-105.4; -5.6..153.1; 57.7..136.4 |
| sweep/single_copy/apple/ep_05 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -206.1..-138.4; -16.2..209.6; 88.4..170.4 |
| sweep/single_fixedcam/apple/ep_00 / left | forearm_link / near (-211.87) | 26 / 75 / 99 | -266.2..-218.3; -72.0..3.7; 128.1..152.6 | -236.1..-184.3; -68.6..10.6; 124.4..165.5 |
| sweep/single_fixedcam/apple/ep_00 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -215.0..-45.9; -187.9..5.4; 15.5..127.9 |
| sweep/single_fixedcam/apple/ep_01 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -271.1..-230.6; -56.0..5.6; 123.9..135.7 | -243.9..-192.8; -51.6..14.8; 124.7..147.0 |
| sweep/single_fixedcam/apple/ep_01 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -220.4..-81.5; -214.6..9.3; -22.9..128.9 |
| sweep/single_fixedcam/apple/ep_02 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -272.6..-143.4; -3.5..303.1; -8.1..133.7 | -266.2..-128.7; -9.5..295.8; 18.3..155.2 |
| sweep/single_fixedcam/apple/ep_02 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -186.5..-167.4; -16.2..5.1; 90.4..112.7 |
| sweep/single_fixedcam/apple/ep_03 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -266.6..-152.3; -1.6..309.0; 4.0..130.9 | -237.6..-141.9; -7.9..289.1; 23.0..156.5 |
| sweep/single_fixedcam/apple/ep_03 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -186.5..-167.8; -16.2..4.9; 90.4..110.7 |
| sweep/single_fixedcam/apple/ep_04 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -290.8..-148.4; -0.6..287.7; 14.3..174.8 | -271.6..-127.0; -7.3..269.2; 32.0..193.7 |
| sweep/single_fixedcam/apple/ep_04 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -186.5..-166.5; -16.2..3.7; 92.5..114.7 |
| sweep/single_fixedcam/apple/ep_05 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -272.4..-226.4; -67.4..2.4; 124.4..148.9 | -246.5..-191.1; -62.4..12.7; 123.9..160.6 |
| sweep/single_fixedcam/apple/ep_05 / right | forearm_link / rim (-19.07) | 103 / 37 / 0 | -174.7..-174.7; 0.0..0.0; 65.7..65.7 | -209.7..-70.8; -202.4..9.2; 5.4..116.8 |
| sweep/table/box/ep_00 / left | forearm_link / near (-209.18) | 48 / 54 / 95 | -558.3..-259.7; -75.4..213.2; 32.7..273.0 | -526.5..-203.8; -68.3..190.5; -16.3..175.5 |
| sweep/table/box/ep_00 / right | forearm_link / near (-210.45) | 44 / 57 / 99 | -335.4..-249.3; -20.8..2.2; 120.5..162.3 | -307.2..-213.2; -19.4..4.4; 120.0..173.9 |
| sweep/table/box/ep_01 / left | forearm_link / near (-209.20) | 51 / 53 / 93 | -556.3..-258.4; -78.1..216.3; 32.9..257.2 | -530.7..-208.1; -85.3..224.0; 25.9..178.9 |
| sweep/table/box/ep_01 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -305.4..-252.5; -27.9..0.0; 110.4..165.1 | -276.6..-213.3; -30.2..6.5; 110.0..177.6 |
| sweep/table/box/ep_02 / left | forearm_link / near (-209.20) | 51 / 53 / 93 | -555.9..-235.9; -50.5..213.7; 23.6..256.3 | -523.0..-209.1; -47.2..214.5; 0.8..154.9 |
| sweep/table/box/ep_02 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -318.8..-252.9; -54.3..0.0; 112.7..161.5 | -291.9..-214.5; -51.6..6.3; 109.8..175.0 |
| sweep/table/box/ep_03 / left | forearm_link / near (-209.20) | 51 / 53 / 93 | -560.6..-174.3; -68.2..204.2; 6.4..256.0 | -531.4..-170.8; -77.6..175.9; -43.5..157.8 |
| sweep/table/box/ep_03 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -303.5..-253.1; -47.8..0.0; 120.4..244.1 | -276.9..-215.0; -42.5..6.5; 121.7..250.9 |
| sweep/tip/apple/ep_00 / left | forearm_link / near (-210.39) | 46 / 56 / 98 | -267.7..-158.6; -0.9..313.7; -2.6..145.8 | -270.9..-152.1; -3.9..314.7; -2.3..148.6 |
| sweep/tip/apple/ep_00 / right | forearm_link / near (-210.63) | 44 / 58 / 98 | -258.1..-230.1; -34.3..12.0; 116.1..200.1 | -264.7..-223.7; -38.8..10.5; 108.6..205.2 |
| sweep/tip/apple/ep_01 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -291.5..-146.8; 0.0..318.1; -3.9..143.2 | -296.1..-142.8; -3.9..327.5; -12.4..152.0 |
| sweep/tip/apple/ep_01 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -258.1..-250.3; -11.9..4.1; 116.2..194.8 | -266.2..-244.3; -17.0..9.9; 110.5..200.5 |
| sweep/tip/apple/ep_02 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -258.0..-155.1; 0.0..313.3; -0.8..128.6 | -264.9..-151.6; -3.9..317.5; -4.0..132.6 |
| sweep/tip/apple/ep_02 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -257.9..-250.0; -34.4..5.4; 117.5..218.7 | -264.2..-243.7; -38.8..11.6; 110.9..223.0 |
| sweep/tip/apple/ep_03 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -282.3..-144.7; 0.1..343.3; 7.5..136.8 | -286.9..-141.5; -0.5..348.7; -0.6..137.3 |
| sweep/tip/apple/ep_03 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -262.9..-253.0; -31.2..4.2; 113.1..202.0 | -269.9..-245.8; -34.2..9.9; 105.3..205.6 |
| sweep/tip/apple/ep_04 / left | forearm_link / near (-209.99) | 48 / 53 / 101 | -280.1..-139.0; 0.0..321.4; 7.9..124.4 | -283.7..-133.4; -7.3..328.3; 1.4..128.2 |
| sweep/tip/apple/ep_04 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -256.2..-245.5; -2.4..5.5; 113.8..143.8 | -261.3..-241.9; -6.7..10.7; 104.8..148.4 |
| sweep/tip/apple/ep_05 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -256.2..-118.0; 0.0..309.0; 9.1..137.8 | -264.0..-118.1; -3.9..317.9; 6.1..147.7 |
| sweep/tip/apple/ep_05 / right | forearm_link / near (-209.99) | 47 / 54 / 101 | -258.5..-249.2; -17.3..4.5; 116.7..189.8 | -266.3..-241.9; -21.9..10.5; 110.7..192.7 |
| sweep/tip/box/ep_00 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -292.3..-135.0; -2.8..266.6; -13.9..211.5 | -296.6..-131.6; -8.4..268.5; -21.7..220.8 |
| sweep/tip/box/ep_00 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.7..-227.3; -72.4..0.0; 121.1..374.8 | -263.3..-223.5; -75.8..4.4; 116.6..382.0 |
| sweep/tip/box/ep_01 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -264.0..-31.8; -6.0..186.3; -49.8..160.6 | -273.6..-27.2; -11.6..204.0; -52.1..162.2 |
| sweep/tip/box/ep_01 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -301.9..-249.7; -19.1..0.0; 121.1..236.2 | -310.7..-244.3; -25.2..6.5; 117.1..243.3 |
| sweep/tip/box/ep_02 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -258.4..-29.9; -4.3..189.2; -24.6..133.3 | -265.2..-25.3; -10.2..194.8; -28.2..136.5 |
| sweep/tip/box/ep_02 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -285.1..-242.0; -53.0..0.0; 121.1..277.6 | -290.9..-234.8; -58.7..6.3; 115.7..282.8 |
| sweep/tip/box/ep_03 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -290.8..-154.7; -5.7..287.1; -10.5..155.3 | -294.3..-150.4; -12.0..292.1; -17.2..159.3 |
| sweep/tip/box/ep_03 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -353.0..-251.2; -74.1..0.1; -31.9..184.7 | -360.2..-245.3; -74.1..6.5; -36.3..190.6 |
| sweep/tip/box/ep_04 / left | forearm_link / near (-209.98) | 47 / 54 / 101 | -258.8..-70.4; -6.7..147.1; -22.7..205.6 | -263.8..-66.3; -11.3..153.8; -27.2..214.1 |
| sweep/tip/box/ep_04 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.2..-228.3; -58.8..0.0; 121.3..389.2 | -263.3..-222.1; -63.6..8.0; 115.7..393.3 |
| sweep/tip/box/ep_05 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -281.4..-131.4; -6.8..319.0; -15.4..171.9 | -286.5..-129.1; -13.1..325.1; -21.5..176.5 |
| sweep/tip/box/ep_05 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.3..-235.5; -48.7..0.0; 121.3..267.6 | -265.0..-232.7; -52.8..7.2; 116.6..273.7 |
| viscous_apple_makerwrist_live1 / left | forearm_link / near (-210.14) | 47 / 55 / 98 | -287.3..-64.4; -0.4..337.6; -12.6..128.0 | -282.7..-70.0; -3.9..319.8; 9.6..152.2 |
| viscous_apple_makerwrist_live1 / right | forearm_link / near (-210.59) | 44 / 58 / 98 | -268.1..-213.8; -55.4..4.5; 100.1..194.6 | -239.6..-182.5; -50.2..8.2; 113.1..204.2 |
| viscous_apple_vision_live2 / left | forearm_link / near (-209.99) | 47 / 54 / 101 | -369.3..-233.2; -304.4..46.9; 121.7..259.5 | -344.9..-179.7; -289.4..51.0; 120.2..255.1 |
| viscous_apple_vision_live2 / right | forearm_link / near (-209.98) | 47 / 54 / 101 | -256.9..-205.7; -49.2..0.1; 124.5..222.5 | -232.6..-160.8; -50.9..7.2; 124.9..238.1 |
| viscous_box_makerwrist_live1 / left | forearm_link / near (-210.01) | 49 / 52 / 99 | -271.1..-90.4; -6.8..260.3; -11.2..141.6 | -280.4..-99.5; -12.6..262.0; 15.3..151.7 |
| viscous_box_makerwrist_live1 / right | forearm_link / near (-211.38) | 35 / 67 / 98 | -259.7..-251.2; -29.8..3.8; 121.3..253.7 | -232.3..-209.8; -27.6..6.2; 121.0..262.7 |
