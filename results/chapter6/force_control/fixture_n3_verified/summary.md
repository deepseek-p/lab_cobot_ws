# Fixture N=3 Force-Control Verification Summary

Force source: VIRTUAL_ESTIMATE
Controller: force-feedback incremental gripper control
Arm control: MoveIt2 / Cartesian position trajectory control
Force-controlled DOF: gripper closure
Experiment type: simulation prototype validation

## Trial 1
planning_success=1 cartesian_success=1 contact=1/1 contact_success=1
force_control_final_state=FORCE_HOLD force_control_success=1 attach_success=1 holding_success=1 lift_success=1 failure_stage=SUCCESS failure_reason=none
lift_primary_success=1 lift_state_verified=0 lift_error_code=1 lift_error_text=
state_chain=FORCE_REGULATION -> FORCE_HOLD
state_counts=FORCE_HOLD:1, FORCE_REGULATION:1
distinct_raw_virtual_force_levels=13.800
steady_force_left=13.800 steady_force_right=13.800 steady_force_mean=13.800 balance_error=0.000
contact_to_force_regulation_time= contact_to_force_hold_time=
evidence: elapsed,state,force_left,force_right,force_mean,force_error,delta_q,cmd_left,cmd_right
19.646,FORCE_REGULATION,13.800,13.800,13.800,-5.800,0.000000,0.006000,0.006000
19.649,FORCE_HOLD,13.800,13.800,13.800,-5.800,0.000000,0.006000,0.006000

## Trial 2
planning_success=1 cartesian_success=1 contact=1/1 contact_success=1
force_control_final_state=FORCE_HOLD force_control_success=1 attach_success=1 holding_success=1 lift_success=1 failure_stage=SUCCESS failure_reason=none
lift_primary_success=1 lift_state_verified=0 lift_error_code=1 lift_error_text=
state_chain=CONTACT_TRANSITION -> FORCE_REGULATION -> FORCE_HOLD
state_counts=CONTACT_TRANSITION:1, FORCE_HOLD:1, FORCE_REGULATION:1
distinct_raw_virtual_force_levels=0.000, 13.800
steady_force_left=13.800 steady_force_right=13.800 steady_force_mean=13.800 balance_error=0.000
contact_to_force_regulation_time=0.101 contact_to_force_hold_time=0.102
evidence: elapsed,state,force_left,force_right,force_mean,force_error,delta_q,cmd_left,cmd_right
18.462,CONTACT_TRANSITION,0.000,13.800,6.900,1.100,0.000000,0.006000,0.006000
18.563,FORCE_REGULATION,13.800,13.800,13.800,-5.800,0.000000,0.006000,0.006000
18.564,FORCE_HOLD,13.800,13.800,13.800,-5.800,0.000000,0.006000,0.006000

## Trial 3
planning_success=1 cartesian_success=1 contact=1/1 contact_success=1
force_control_final_state=FORCE_HOLD force_control_success=1 attach_success=1 holding_success=1 lift_success=1 failure_stage=SUCCESS failure_reason=none
lift_primary_success=1 lift_state_verified=0 lift_error_code=1 lift_error_text=
state_chain=CONTACT_TRANSITION -> FORCE_REGULATION -> FORCE_HOLD
state_counts=CONTACT_TRANSITION:1, FORCE_HOLD:1, FORCE_REGULATION:1
distinct_raw_virtual_force_levels=0.000, 13.800
steady_force_left=13.800 steady_force_right=13.800 steady_force_mean=13.800 balance_error=0.000
contact_to_force_regulation_time=0.101 contact_to_force_hold_time=0.101
evidence: elapsed,state,force_left,force_right,force_mean,force_error,delta_q,cmd_left,cmd_right
32.706,CONTACT_TRANSITION,0.000,13.800,6.900,1.100,0.000000,0.006000,0.006000
32.807,FORCE_REGULATION,13.800,13.800,13.800,-5.800,0.000000,0.006000,0.006000
32.808,FORCE_HOLD,13.800,13.800,13.800,-5.800,0.000000,0.006000,0.006000

## Totals
FORCE_FEEDBACK_ACTIVE=3/3
FORCE_HOLD_REACHED=3/3
ATTACH_SUCCESS=3/3
HOLDING_SUCCESS=3/3
PRIMARY_LIFT_SUCCESS=3/3
STATE_VERIFIED_TASK_COMPLETION=0/3
TASK_SUCCESS=3/3
abnormal_counts=FORCE_LIMIT:0, FORCE_UNBALANCED:0, FORCE_SENSOR_INVALID:0, ATTACH_TIMEOUT:0, HOLD_LOST:0, LIFT_CONTROLLER_ABORT:0
