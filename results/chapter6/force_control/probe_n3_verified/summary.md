# Probe N=3 Force-Control Verification Summary

Force source: VIRTUAL_ESTIMATE
Controller: force-feedback incremental gripper control
Arm control: MoveIt2 / Cartesian position trajectory control
Force-controlled DOF: gripper closure
Experiment type: simulation prototype validation

## Trial 1
planning_success=1 cartesian_success=1 contact=1/1 contact_success=1
force_control_final_state=FORCE_HOLD force_control_success=1 attach_success=1 holding_success=1 lift_success=1 failure_stage=SUCCESS failure_reason=none
state_counts=CONTACT_TRANSITION:12, FORCE_HOLD:1, FORCE_REGULATION:1, POSITION_CLOSE:56
state_chain=POSITION_CLOSE -> CONTACT_TRANSITION -> FORCE_REGULATION -> FORCE_HOLD
nonzero_delta_q_count=67 gripper_command_left_range=0.006000..0.022800 gripper_command_right_range=0.006000..0.020000
steady_force_left=20.000 steady_force_right=20.000 steady_force_mean=20.000 balance_error=0.000
contact_to_force_regulation_time=1.225 contact_to_force_hold_time=1.226
evidence: elapsed,state,force_error,delta_q,cmd_l,cmd_r,force_l_f,force_r_f
9.706,POSITION_CLOSE,8.000,0.000250,0.006000,0.006000,0.000,0.000
9.813,POSITION_CLOSE,8.000,0.000250,0.006300,0.006300,0.000,0.000
9.915,POSITION_CLOSE,8.000,0.000250,0.006500,0.006500,0.000,0.000
10.017,POSITION_CLOSE,8.000,0.000250,0.006800,0.006800,0.000,0.000
10.118,POSITION_CLOSE,8.000,0.000250,0.007000,0.007000,0.000,0.000
10.220,POSITION_CLOSE,8.000,0.000250,0.007300,0.007300,0.000,0.000

## Trial 2
planning_success=1 cartesian_success=1 contact=1/1 contact_success=1
force_control_final_state=FORCE_HOLD force_control_success=1 attach_success=1 holding_success=1 lift_success=1 failure_stage=SUCCESS failure_reason=none
state_counts=CONTACT_TRANSITION:13, FORCE_HOLD:1, FORCE_REGULATION:1, POSITION_CLOSE:56
state_chain=POSITION_CLOSE -> CONTACT_TRANSITION -> FORCE_REGULATION -> FORCE_HOLD
nonzero_delta_q_count=67 gripper_command_left_range=0.006000..0.022800 gripper_command_right_range=0.006000..0.020000
steady_force_left=20.000 steady_force_right=20.000 steady_force_mean=20.000 balance_error=0.000
contact_to_force_regulation_time=1.332 contact_to_force_hold_time=1.333
evidence: elapsed,state,force_error,delta_q,cmd_l,cmd_r,force_l_f,force_r_f
14.383,POSITION_CLOSE,8.000,0.000250,0.006000,0.006000,0.000,0.000
14.486,POSITION_CLOSE,8.000,0.000250,0.006300,0.006300,0.000,0.000
14.588,POSITION_CLOSE,8.000,0.000250,0.006500,0.006500,0.000,0.000
14.700,POSITION_CLOSE,8.000,0.000250,0.006800,0.006800,0.000,0.000
14.802,POSITION_CLOSE,8.000,0.000250,0.007000,0.007000,0.000,0.000
14.905,POSITION_CLOSE,8.000,0.000250,0.007300,0.007300,0.000,0.000

## Trial 3
planning_success=1 cartesian_success=1 contact=1/1 contact_success=1
force_control_final_state=FORCE_HOLD force_control_success=1 attach_success=1 holding_success=1 lift_success=1 failure_stage=SUCCESS failure_reason=none
state_counts=CONTACT_TRANSITION:4, FORCE_HOLD:1, FORCE_REGULATION:1, POSITION_CLOSE:60
state_chain=POSITION_CLOSE -> CONTACT_TRANSITION -> FORCE_REGULATION -> FORCE_HOLD
nonzero_delta_q_count=63 gripper_command_left_range=0.006000..0.021800 gripper_command_right_range=0.006000..0.021000
steady_force_left=20.000 steady_force_right=20.000 steady_force_mean=20.000 balance_error=0.000
contact_to_force_regulation_time=0.410 contact_to_force_hold_time=0.411
evidence: elapsed,state,force_error,delta_q,cmd_l,cmd_r,force_l_f,force_r_f
15.682,POSITION_CLOSE,8.000,0.000250,0.006000,0.006000,0.000,0.000
15.785,POSITION_CLOSE,8.000,0.000250,0.006300,0.006300,0.000,0.000
15.888,POSITION_CLOSE,8.000,0.000250,0.006500,0.006500,0.000,0.000
15.989,POSITION_CLOSE,8.000,0.000250,0.006800,0.006800,0.000,0.000
16.091,POSITION_CLOSE,8.000,0.000250,0.007000,0.007000,0.000,0.000
16.193,POSITION_CLOSE,8.000,0.000250,0.007300,0.007300,0.000,0.000

## Totals
FORCE_FEEDBACK_ACTIVE=3/3
FORCE_HOLD_REACHED=3/3
TASK_SUCCESS=3/3
abnormal_counts=FORCE_LIMIT:0, FORCE_UNBALANCED:0, FORCE_SENSOR_INVALID:0, ATTACH_TIMEOUT:0, HOLD_LOST:0, LIFT:0
