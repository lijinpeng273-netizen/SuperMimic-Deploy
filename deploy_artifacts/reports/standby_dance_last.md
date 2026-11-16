# Policy standby gate (last)

H1 torque MJCF, 1 kHz PD; gantry until the policy takes over, then NO support: STANDBY_PRE 60 s -> motion -> STANDBY_POST 30 s. Grid per clip and profile: latency [0, 10, 20] ms x mu [0.6, 1.0] x kp [0.85, 1.0, 1.15] x torso +[0.0, 1.8] kg = 36 runs.
Pass rule: 0 falls, 0 aborts, standby tilt err < 8.0 deg, hand-off jumps STANDBY->POLICY and POLICY->STANDBY_POST <= 0.5 x HOLD->STANDBY_PRE jump of the same run.
R1+X check applied (runner defaults): pressed at 60 s, motion starts at the first step at which STANDBY_PRE >= 3.0 s and over the last 3.0 s tilt err < 8.0 deg and max|dq| < 1.0 rad/s; never within 30.0 s -> no motion (counted as 'never'). 'moving at start': pelvis speed over the last 2 s > 2 cm/s or yaw rate > 2 deg/s when the motion started.

| clip | profile | pass | R1+X never / delayed (max wait) / moving at start | falls (phase) | abort runs | PRE tilt err max | PRE xy drift max | PRE yaw drift max | PRE max abs a | PRE dq/vlim | POST tilt err max | POST xy drift max | POST yaw drift max | POST max abs a | POST dq/vlim | R1+X ok after (max) | jump HOLD->SB | SB->POLICY | POLICY->POST | max ratio |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| dance | slack | 0/36 | 0 / 0 (0.00 s) / 0 | 0  | 36 | 31.98 deg | 66.3 cm | 9.4 deg | 7.49 | 0.947 | 7.77 deg | 30.6 cm | 11.6 deg | 5.69 | 0.682 | 10.04 s | 48.6-70.4 | 23.22 | 49.20 | 1.005 |
| dance | ground | 0/36 | 0 / 0 (0.00 s) / 0 | 0  | 36 | 31.98 deg | 41.8 cm | 8.7 deg | 7.49 | 0.842 | 6.64 deg | 19.9 cm | 11.3 deg | 4.98 | 0.366 | 9.90 s | 48.6-70.4 | 23.22 | 33.90 | 0.698 |

Jumps: max over joints of |tau(t) - tau(t - 1 ms)| [Nm/ms] in the window [switch - 1 ms, switch + latency + 40 ms]; max over the runs (HOLD->SB: min-max). Steady baselines (max over runs of the per-run p99 / max):

- STANDBY_PRE (after 1 s): p99 4.15, max 75.07; POLICY: p99 11.22, max 84.60; STANDBY_POST (after 1 s): p99 1.02, max 94.73 Nm/ms.
- Settled standby (after the first 5 s): PRE tilt err max 2.95 deg, max|dq| 2.55 rad/s, xy drift over the rest 34.9 cm; POST tilt err max 7.77 deg, max|dq| 9.55 rad/s, xy drift 4.9 cm.

## Failing runs

| clip | profile | lat | mu | kp | mass | R1+X never / wait / moving | upright (phase) | aborts | first abort | standby tilt err | jumps HOLD->SB / SB->POL / POL->POST |
|---|---|---|---|---|---|---|---|---|---|---|---|
| dance | ground | 0 | 0.6 | 0.85 | 0.0 | False / 0.00 / False | True (None) | 7 | standby_pre: (0, 'tilt error 30 deg') | 30.43 | 48.93 / 17.58 / 0.25 |
| dance | ground | 0 | 0.6 | 0.85 | 1.8 | False / 0.00 / False | True (None) | 8 | standby_pre: (0, 'tilt error 32 deg') | 31.98 | 48.59 / 18.50 / 12.66 |
| dance | ground | 0 | 0.6 | 1.0 | 0.0 | False / 0.00 / False | True (None) | 6 | standby_pre: (0, 'tilt error 27 deg') | 26.90 | 59.58 / 19.32 / 0.10 |
| dance | ground | 0 | 0.6 | 1.0 | 1.8 | False / 0.00 / False | True (None) | 6 | standby_pre: (0, 'tilt error 28 deg') | 28.27 | 59.49 / 19.46 / 0.14 |
| dance | ground | 0 | 0.6 | 1.15 | 0.0 | False / 0.00 / False | True (None) | 4 | standby_pre: (0, 'tilt error 24 deg') | 24.38 | 69.86 / 23.13 / 0.13 |
| dance | ground | 0 | 0.6 | 1.15 | 1.8 | False / 0.00 / False | True (None) | 5 | standby_pre: (0, 'tilt error 26 deg') | 25.52 | 69.77 / 22.82 / 0.10 |
| dance | ground | 0 | 1.0 | 0.85 | 0.0 | False / 0.00 / False | True (None) | 7 | standby_pre: (0, 'tilt error 30 deg') | 30.14 | 49.71 / 17.58 / 0.31 |
| dance | ground | 0 | 1.0 | 0.85 | 1.8 | False / 0.00 / False | True (None) | 7 | standby_pre: (0, 'tilt error 32 deg') | 31.55 | 49.46 / 18.09 / 10.31 |
| dance | ground | 0 | 1.0 | 1.0 | 0.0 | False / 0.00 / False | True (None) | 5 | standby_pre: (0, 'tilt error 27 deg') | 26.66 | 60.27 / 19.36 / 0.14 |
| dance | ground | 0 | 1.0 | 1.0 | 1.8 | False / 0.00 / False | True (None) | 6 | standby_pre: (0, 'tilt error 28 deg') | 27.99 | 60.23 / 19.43 / 0.19 |
| dance | ground | 0 | 1.0 | 1.15 | 0.0 | False / 0.00 / False | True (None) | 4 | standby_pre: (0, 'tilt error 24 deg') | 24.19 | 70.39 / 23.22 / 0.10 |
| dance | ground | 0 | 1.0 | 1.15 | 1.8 | False / 0.00 / False | True (None) | 4 | standby_pre: (0, 'tilt error 25 deg') | 25.29 | 70.39 / 22.92 / 0.13 |
| dance | ground | 10 | 0.6 | 0.85 | 0.0 | False / 0.00 / False | True (None) | 7 | standby_pre: (0, 'tilt error 30 deg') | 30.43 | 48.92 / 17.85 / 0.36 |
| dance | ground | 10 | 0.6 | 0.85 | 1.8 | False / 0.00 / False | True (None) | 8 | standby_pre: (0, 'tilt error 32 deg') | 31.98 | 48.59 / 18.16 / 31.01 |
| dance | ground | 10 | 0.6 | 1.0 | 0.0 | False / 0.00 / False | True (None) | 6 | standby_pre: (0, 'tilt error 27 deg') | 26.90 | 59.59 / 20.19 / 0.10 |
| dance | ground | 10 | 0.6 | 1.0 | 1.8 | False / 0.00 / False | True (None) | 7 | standby_pre: (0, 'tilt error 28 deg') | 28.27 | 59.49 / 20.83 / 0.13 |
| dance | ground | 10 | 0.6 | 1.15 | 0.0 | False / 0.00 / False | True (None) | 4 | standby_pre: (0, 'tilt error 24 deg') | 24.38 | 69.87 / 23.05 / 0.25 |
| dance | ground | 10 | 0.6 | 1.15 | 1.8 | False / 0.00 / False | True (None) | 5 | standby_pre: (0, 'tilt error 26 deg') | 25.52 | 69.77 / 22.70 / 0.17 |
| dance | ground | 10 | 1.0 | 0.85 | 0.0 | False / 0.00 / False | True (None) | 7 | standby_pre: (0, 'tilt error 30 deg') | 30.14 | 49.70 / 18.05 / 10.82 |
| dance | ground | 10 | 1.0 | 0.85 | 1.8 | False / 0.00 / False | True (None) | 8 | standby_pre: (0, 'tilt error 32 deg') | 31.55 | 49.45 / 17.62 / 4.42 |
| dance | ground | 10 | 1.0 | 1.0 | 0.0 | False / 0.00 / False | True (None) | 6 | standby_pre: (0, 'tilt error 27 deg') | 26.66 | 60.27 / 20.03 / 0.17 |
| dance | ground | 10 | 1.0 | 1.0 | 1.8 | False / 0.00 / False | True (None) | 6 | standby_pre: (0, 'tilt error 28 deg') | 27.99 | 60.23 / 20.36 / 0.19 |
| dance | ground | 10 | 1.0 | 1.15 | 0.0 | False / 0.00 / False | True (None) | 4 | standby_pre: (0, 'tilt error 24 deg') | 24.19 | 70.40 / 23.10 / 0.20 |
| dance | ground | 10 | 1.0 | 1.15 | 1.8 | False / 0.00 / False | True (None) | 5 | standby_pre: (0, 'tilt error 25 deg') | 25.29 | 70.40 / 22.75 / 0.11 |
| dance | ground | 20 | 0.6 | 0.85 | 0.0 | False / 0.00 / False | True (None) | 9 | standby_pre: (0, 'tilt error 30 deg') | 30.43 | 48.92 / 18.66 / 0.43 |
| dance | ground | 20 | 0.6 | 0.85 | 1.8 | False / 0.00 / False | True (None) | 8 | standby_pre: (0, 'tilt error 32 deg') | 31.98 | 48.59 / 18.81 / 33.90 |
| dance | ground | 20 | 0.6 | 1.0 | 0.0 | False / 0.00 / False | True (None) | 6 | standby_pre: (0, 'tilt error 27 deg') | 26.90 | 59.59 / 20.97 / 0.11 |
| dance | ground | 20 | 0.6 | 1.0 | 1.8 | False / 0.00 / False | True (None) | 8 | standby_pre: (0, 'tilt error 28 deg') | 28.27 | 59.49 / 21.34 / 0.19 |
| dance | ground | 20 | 0.6 | 1.15 | 0.0 | False / 0.00 / False | True (None) | 5 | standby_pre: (0, 'tilt error 24 deg') | 24.38 | 69.87 / 22.82 / 0.32 |
| dance | ground | 20 | 0.6 | 1.15 | 1.8 | False / 0.00 / False | True (None) | 5 | standby_pre: (0, 'tilt error 26 deg') | 25.52 | 69.78 / 23.04 / 0.18 |
| dance | ground | 20 | 1.0 | 0.85 | 0.0 | False / 0.00 / False | True (None) | 8 | standby_pre: (0, 'tilt error 30 deg') | 30.14 | 49.70 / 17.93 / 21.37 |
| dance | ground | 20 | 1.0 | 0.85 | 1.8 | False / 0.00 / False | True (None) | 8 | standby_pre: (0, 'tilt error 32 deg') | 31.55 | 49.45 / 18.38 / 6.33 |
| dance | ground | 20 | 1.0 | 1.0 | 0.0 | False / 0.00 / False | True (None) | 7 | standby_pre: (0, 'tilt error 27 deg') | 26.66 | 60.27 / 20.63 / 0.18 |
| dance | ground | 20 | 1.0 | 1.0 | 1.8 | False / 0.00 / False | True (None) | 8 | standby_pre: (0, 'tilt error 28 deg') | 27.99 | 60.23 / 21.19 / 0.22 |
| dance | ground | 20 | 1.0 | 1.15 | 0.0 | False / 0.00 / False | True (None) | 5 | standby_pre: (0, 'tilt error 24 deg') | 24.19 | 70.41 / 22.88 / 0.19 |
| dance | ground | 20 | 1.0 | 1.15 | 1.8 | False / 0.00 / False | True (None) | 5 | standby_pre: (0, 'tilt error 25 deg') | 25.29 | 70.40 / 23.00 / 0.13 |
| dance | slack | 0 | 0.6 | 0.85 | 0.0 | False / 0.00 / False | True (None) | 26 | standby_pre: (0, 'tilt error 30 deg') | 30.43 | 48.93 / 18.48 / 9.82 |
| dance | slack | 0 | 0.6 | 0.85 | 1.8 | False / 0.00 / False | True (None) | 26 | standby_pre: (0, 'tilt error 32 deg') | 31.98 | 48.59 / 18.27 / 48.85 |
| dance | slack | 0 | 0.6 | 1.0 | 0.0 | False / 0.00 / False | True (None) | 23 | standby_pre: (0, 'tilt error 27 deg') | 26.90 | 59.58 / 19.32 / 0.10 |
| dance | slack | 0 | 0.6 | 1.0 | 1.8 | False / 0.00 / False | True (None) | 24 | standby_pre: (0, 'tilt error 28 deg') | 28.27 | 59.49 / 19.45 / 0.14 |
| dance | slack | 0 | 0.6 | 1.15 | 0.0 | False / 0.00 / False | True (None) | 23 | standby_pre: (0, 'tilt error 24 deg') | 24.38 | 69.86 / 23.13 / 0.13 |
| dance | slack | 0 | 0.6 | 1.15 | 1.8 | False / 0.00 / False | True (None) | 24 | standby_pre: (0, 'tilt error 26 deg') | 25.52 | 69.77 / 22.82 / 0.10 |
| dance | slack | 0 | 1.0 | 0.85 | 0.0 | False / 0.00 / False | True (None) | 25 | standby_pre: (0, 'tilt error 30 deg') | 30.14 | 49.71 / 18.37 / 28.21 |
| dance | slack | 0 | 1.0 | 0.85 | 1.8 | False / 0.00 / False | True (None) | 32 | standby_pre: (0, 'tilt error 32 deg') | 31.55 | 49.46 / 18.08 / 4.57 |
| dance | slack | 0 | 1.0 | 1.0 | 0.0 | False / 0.00 / False | True (None) | 23 | standby_pre: (0, 'tilt error 27 deg') | 26.66 | 60.27 / 19.36 / 0.14 |
| dance | slack | 0 | 1.0 | 1.0 | 1.8 | False / 0.00 / False | True (None) | 26 | standby_pre: (0, 'tilt error 28 deg') | 27.99 | 60.23 / 19.42 / 0.19 |
| dance | slack | 0 | 1.0 | 1.15 | 0.0 | False / 0.00 / False | True (None) | 24 | standby_pre: (0, 'tilt error 24 deg') | 24.19 | 70.39 / 23.22 / 0.10 |
| dance | slack | 0 | 1.0 | 1.15 | 1.8 | False / 0.00 / False | True (None) | 24 | standby_pre: (0, 'tilt error 25 deg') | 25.29 | 70.39 / 22.92 / 0.12 |
| dance | slack | 10 | 0.6 | 0.85 | 0.0 | False / 0.00 / False | True (None) | 28 | standby_pre: (0, 'tilt error 30 deg') | 30.43 | 48.92 / 18.46 / 4.06 |
| dance | slack | 10 | 0.6 | 0.85 | 1.8 | False / 0.00 / False | True (None) | 31 | standby_pre: (0, 'tilt error 32 deg') | 31.98 | 48.59 / 18.54 / 43.28 |
| dance | slack | 10 | 0.6 | 1.0 | 0.0 | False / 0.00 / False | True (None) | 25 | standby_pre: (0, 'tilt error 27 deg') | 26.90 | 59.59 / 20.21 / 0.10 |
| dance | slack | 10 | 0.6 | 1.0 | 1.8 | False / 0.00 / False | True (None) | 28 | standby_pre: (0, 'tilt error 28 deg') | 28.27 | 59.49 / 20.86 / 0.13 |
| dance | slack | 10 | 0.6 | 1.15 | 0.0 | False / 0.00 / False | True (None) | 26 | standby_pre: (0, 'tilt error 24 deg') | 24.38 | 69.87 / 23.06 / 0.25 |
| dance | slack | 10 | 0.6 | 1.15 | 1.8 | False / 0.00 / False | True (None) | 28 | standby_pre: (0, 'tilt error 26 deg') | 25.52 | 69.77 / 22.70 / 0.17 |
| dance | slack | 10 | 1.0 | 0.85 | 0.0 | False / 0.00 / False | True (None) | 28 | standby_pre: (0, 'tilt error 30 deg') | 30.14 | 49.70 / 18.52 / 5.04 |
| dance | slack | 10 | 1.0 | 0.85 | 1.8 | False / 0.00 / False | True (None) | 31 | standby_pre: (0, 'tilt error 32 deg') | 31.55 | 49.45 / 18.52 / 49.20 |
| dance | slack | 10 | 1.0 | 1.0 | 0.0 | False / 0.00 / False | True (None) | 26 | standby_pre: (0, 'tilt error 27 deg') | 26.66 | 60.27 / 20.08 / 0.17 |
| dance | slack | 10 | 1.0 | 1.0 | 1.8 | False / 0.00 / False | True (None) | 26 | standby_pre: (0, 'tilt error 28 deg') | 27.99 | 60.23 / 20.35 / 0.19 |
| dance | slack | 10 | 1.0 | 1.15 | 0.0 | False / 0.00 / False | True (None) | 26 | standby_pre: (0, 'tilt error 24 deg') | 24.19 | 70.40 / 23.10 / 0.20 |
| dance | slack | 10 | 1.0 | 1.15 | 1.8 | False / 0.00 / False | True (None) | 29 | standby_pre: (0, 'tilt error 25 deg') | 25.29 | 70.40 / 22.76 / 0.12 |
| dance | slack | 20 | 0.6 | 0.85 | 0.0 | False / 0.00 / False | True (None) | 27 | standby_pre: (0, 'tilt error 30 deg') | 30.43 | 48.92 / 18.53 / 14.42 |
| dance | slack | 20 | 0.6 | 0.85 | 1.8 | False / 0.00 / False | True (None) | 29 | standby_pre: (0, 'tilt error 32 deg') | 31.98 | 48.59 / 18.39 / 3.84 |
| dance | slack | 20 | 0.6 | 1.0 | 0.0 | False / 0.00 / False | True (None) | 24 | standby_pre: (0, 'tilt error 27 deg') | 26.90 | 59.59 / 21.07 / 0.11 |
| dance | slack | 20 | 0.6 | 1.0 | 1.8 | False / 0.00 / False | True (None) | 27 | standby_pre: (0, 'tilt error 28 deg') | 28.27 | 59.49 / 21.41 / 0.17 |
| dance | slack | 20 | 0.6 | 1.15 | 0.0 | False / 0.00 / False | True (None) | 24 | standby_pre: (0, 'tilt error 24 deg') | 24.38 | 69.87 / 22.82 / 0.31 |
| dance | slack | 20 | 0.6 | 1.15 | 1.8 | False / 0.00 / False | True (None) | 24 | standby_pre: (0, 'tilt error 26 deg') | 25.52 | 69.78 / 23.11 / 0.19 |
| dance | slack | 20 | 1.0 | 0.85 | 0.0 | False / 0.00 / False | True (None) | 27 | standby_pre: (0, 'tilt error 30 deg') | 30.14 | 49.70 / 18.56 / 18.47 |
| dance | slack | 20 | 1.0 | 0.85 | 1.8 | False / 0.00 / False | True (None) | 34 | standby_pre: (0, 'tilt error 32 deg') | 31.55 | 49.45 / 18.62 / 6.65 |
| dance | slack | 20 | 1.0 | 1.0 | 0.0 | False / 0.00 / False | True (None) | 25 | standby_pre: (0, 'tilt error 27 deg') | 26.66 | 60.27 / 20.67 / 0.18 |
| dance | slack | 20 | 1.0 | 1.0 | 1.8 | False / 0.00 / False | True (None) | 26 | standby_pre: (0, 'tilt error 28 deg') | 27.99 | 60.23 / 21.33 / 0.23 |
| dance | slack | 20 | 1.0 | 1.15 | 0.0 | False / 0.00 / False | True (None) | 24 | standby_pre: (0, 'tilt error 24 deg') | 24.19 | 70.41 / 22.85 / 0.19 |
| dance | slack | 20 | 1.0 | 1.15 | 1.8 | False / 0.00 / False | True (None) | 24 | standby_pre: (0, 'tilt error 25 deg') | 25.29 | 70.40 / 22.99 / 0.13 |
