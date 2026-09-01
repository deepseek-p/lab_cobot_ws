# IGBT N=3 Force-Control Verification Summary

Force source: VIRTUAL_ESTIMATE
Control: force-feedback incremental gripper control
Arm: MoveIt2 / Cartesian position / trajectory control
Force-controlled DOF: gripper closure
Experiment: simulation prototype validation

## Trial 1
planning_success=1 cartesian_success=1 contact=1/1 contact_success=1
force_control_final_state=FORCE_HOLD force_control_success=1 attach_success=1 holding_success=1 lift_success=1 failure_stage=SUCCESS failure_reason=none
lift_primary_success=1 lift_state_verified=0 lift_error_code=1 lift_error_text=
contact_order=left@24.349s then right@24.491s left_right_contact_delta=0.142s
first_contact_to_bilateral=0.142s bilateral_to_force_regulation=0.090s bilateral_to_force_hold=0.091s first_contact_to_force_hold=0.233s
state_chain=POSITION_CLOSE -> CONTACT_TRANSITION -> FORCE_REGULATION -> FORCE_HOLD
state_counts=CONTACT_TRANSITION:3, FORCE_HOLD:1, FORCE_REGULATION:1, POSITION_CLOSE:16
distinct_left_raw_force=0.000, 20.000
distinct_right_raw_force=0.000, 20.000
nonzero_delta_q=17 positive_delta_q=17 negative_delta_q=0 cmd_left_range=0.030100..0.034100 cmd_right_range=0.030100..0.034300
steady_force_left=20.000 steady_force_right=20.000 steady_force_mean=20.000 balance_error=0.000
evidence: elapsed,state,force_left,force_right,force_mean,force_error,delta_q,cmd_left,cmd_right
22.638,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.030100,0.030100
22.740,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.030300,0.030300
22.842,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.030600,0.030600
22.944,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.030800,0.030800
23.047,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.031100,0.031100
23.149,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.031300,0.031300
23.250,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.031600,0.031600
23.352,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.031800,0.031800
24.174,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.033800,0.033800
24.276,CONTACT_TRANSITION,0.000,0.000,0.000,8.000,0.000250,0.034100,0.034100
24.378,CONTACT_TRANSITION,20.000,0.000,10.000,-2.000,0.000000,0.034100,0.034300
24.480,CONTACT_TRANSITION,20.000,0.000,10.000,-2.000,0.000000,0.034100,0.034300
24.581,FORCE_REGULATION,20.000,20.000,20.000,-12.000,0.000000,0.034100,0.034300
24.582,FORCE_HOLD,20.000,20.000,20.000,-12.000,0.000000,0.034100,0.034300

## Trial 2
planning_success=1 cartesian_success=1 contact=1/1 contact_success=1
force_control_final_state=FORCE_HOLD force_control_success=1 attach_success=1 holding_success=1 lift_success=1 failure_stage=SUCCESS failure_reason=none
lift_primary_success=1 lift_state_verified=0 lift_error_code=1 lift_error_text=
contact_order=N/A left_right_contact_delta=N/A
first_contact_to_bilateral=N/A bilateral_to_force_regulation=N/A bilateral_to_force_hold=N/A first_contact_to_force_hold=N/A
state_chain=POSITION_CLOSE -> CONTACT_TRANSITION -> FORCE_REGULATION -> FORCE_HOLD
state_counts=CONTACT_TRANSITION:1, FORCE_HOLD:1, FORCE_REGULATION:1, POSITION_CLOSE:17
distinct_left_raw_force=0.000, 20.000
distinct_right_raw_force=0.000, 20.000
nonzero_delta_q=17 positive_delta_q=17 negative_delta_q=0 cmd_left_range=0.030100..0.034300 cmd_right_range=0.030100..0.034300
steady_force_left=20.000 steady_force_right=20.000 steady_force_mean=20.000 balance_error=0.000
evidence: elapsed,state,force_left,force_right,force_mean,force_error,delta_q,cmd_left,cmd_right
22.091,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.030100,0.030100
22.193,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.030300,0.030300
22.296,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.030600,0.030600
22.400,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.030800,0.030800
22.502,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.031100,0.031100
22.604,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.031300,0.031300
22.705,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.031600,0.031600
22.807,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.031800,0.031800
23.520,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.033600,0.033600
23.621,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.033800,0.033800
23.723,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.034100,0.034100
23.825,CONTACT_TRANSITION,20.000,0.000,10.000,-2.000,0.000000,0.034300,0.034300
23.926,FORCE_REGULATION,20.000,20.000,20.000,-12.000,0.000000,0.034300,0.034300
23.926,FORCE_HOLD,20.000,20.000,20.000,-12.000,0.000000,0.034300,0.034300

## Trial 3
planning_success=1 cartesian_success=1 contact=1/1 contact_success=1
force_control_final_state=FORCE_HOLD force_control_success=1 attach_success=1 holding_success=1 lift_success=1 failure_stage=SUCCESS failure_reason=none
lift_primary_success=1 lift_state_verified=0 lift_error_code=1 lift_error_text=
contact_order=N/A left_right_contact_delta=N/A
first_contact_to_bilateral=N/A bilateral_to_force_regulation=N/A bilateral_to_force_hold=N/A first_contact_to_force_hold=N/A
state_chain=POSITION_CLOSE -> CONTACT_TRANSITION -> FORCE_REGULATION -> FORCE_HOLD
state_counts=CONTACT_TRANSITION:1, FORCE_HOLD:1, FORCE_REGULATION:1, POSITION_CLOSE:16
distinct_left_raw_force=0.000, 20.000
distinct_right_raw_force=0.000, 20.000
nonzero_delta_q=17 positive_delta_q=17 negative_delta_q=0 cmd_left_range=0.030100..0.034100 cmd_right_range=0.030100..0.034300
steady_force_left=20.000 steady_force_right=20.000 steady_force_mean=20.000 balance_error=0.000
evidence: elapsed,state,force_left,force_right,force_mean,force_error,delta_q,cmd_left,cmd_right
34.627,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.030100,0.030100
34.732,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.030300,0.030300
34.834,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.030600,0.030600
34.937,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.030800,0.030800
35.039,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.031100,0.031100
35.141,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.031300,0.031300
35.244,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.031600,0.031600
35.347,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.031800,0.031800
35.959,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.033300,0.033300
36.060,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.033600,0.033600
36.162,POSITION_CLOSE,0.000,0.000,0.000,8.000,0.000250,0.033800,0.033800
36.265,CONTACT_TRANSITION,0.000,0.000,0.000,8.000,0.000250,0.034100,0.034100
36.367,FORCE_REGULATION,20.000,20.000,20.000,-12.000,0.000000,0.034100,0.034300
36.367,FORCE_HOLD,20.000,20.000,20.000,-12.000,0.000000,0.034100,0.034300

## Totals
FORCE_AWARE_DECISION=3/3
ACTIVE_FORCE_ADJUSTMENT=3/3
FORCE_HOLD_REACHED=3/3
ATTACH_SUCCESS=3/3
HOLDING_SUCCESS=3/3
PRIMARY_LIFT_SUCCESS=3/3
STATE_VERIFIED_COMPLETION=0/3
TASK_SUCCESS=3/3
abnormal_counts=CONTACT_LEFT_ONLY:0, CONTACT_RIGHT_ONLY:0, FORCE_LIMIT:0, FORCE_UNBALANCED:0, FORCE_SENSOR_INVALID:0, ATTACH_TIMEOUT:0, HOLD_LOST:0, LIFT_CONTROLLER_ABORT:0, PRE_GRASP_PLAN:0, DESCEND:0
