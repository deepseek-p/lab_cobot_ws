# Probe force-control single verification

target: high_voltage_probe_kit
trial: 1
task_success: False
failure_stage: ATTACH_TIMEOUT
failure_reason: FAILED_ATTACH high_voltage_probe_kit
planning_success: 1
cartesian_success: 1
contact_success: 1
attach_success: 0
holding_success: 0
lift_success: 0
force_source: VIRTUAL_ESTIMATE
force_control_samples: 80
force_control_state_sequence: POSITION_CLOSE -> CONTACT_TRANSITION -> FORCE_REGULATION -> POSITION_CLOSE -> FORCE_LIMIT
force_regulation_entered: True
force_hold_entered: False
force_unbalanced_triggered: False
force_limit_triggered: True
force_control_converged: false
ready_for_probe_n3: false

## Closed-loop evidence window

| elapsed_sec | state | cmd_l | cmd_r | F_l_filt | F_r_filt | F_mean | e_F | balance | delta_q |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 14.436483 | POSITION_CLOSE | 0.019800 | 0.019800 | 0.000000 | 0.000000 | 0.000000 | 8.000000 | 0.000000 | 0.000250 |
| 14.538660 | CONTACT_TRANSITION | 0.020000 | 0.020000 | 0.000000 | 0.000000 | 0.000000 | 8.000000 | 0.000000 | 0.000250 |
| 14.640654 | CONTACT_TRANSITION | 0.020300 | 0.020000 | 0.000000 | 6.666667 | 3.333333 | 4.666667 | 6.666667 | 0.000250 |
| 14.742526 | CONTACT_TRANSITION | 0.020500 | 0.020000 | 0.000000 | 13.333333 | 6.666667 | 1.333333 | 13.333333 | 0.000250 |
| 14.844399 | CONTACT_TRANSITION | 0.020800 | 0.020000 | 0.000000 | 20.000000 | 10.000000 | -2.000000 | 20.000000 | 0.000250 |
| 14.946275 | CONTACT_TRANSITION | 0.021000 | 0.020000 | 0.000000 | 20.000000 | 10.000000 | -2.000000 | 20.000000 | 0.000250 |
| 15.048044 | CONTACT_TRANSITION | 0.021300 | 0.020000 | 0.000000 | 20.000000 | 10.000000 | -2.000000 | 20.000000 | 0.000250 |
| 15.150328 | CONTACT_TRANSITION | 0.021500 | 0.020000 | 0.000000 | 20.000000 | 10.000000 | -2.000000 | 20.000000 | 0.000250 |
| 15.252152 | CONTACT_TRANSITION | 0.021800 | 0.020000 | 0.000000 | 20.000000 | 10.000000 | -2.000000 | 20.000000 | 0.000250 |
| 15.354652 | CONTACT_TRANSITION | 0.022000 | 0.020000 | 0.000000 | 20.000000 | 10.000000 | -2.000000 | 20.000000 | 0.000250 |
