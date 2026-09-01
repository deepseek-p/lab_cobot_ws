# Chapter 6 Final Baseline Summary

Source: results/chapter6/final_baseline/*/grasp_trials.csv

## Target Results

| target | trials | reset clean | planning | descend | contact | attach | holding | lift final | success | system rate | valid grasp rate | primary lift | state verified | recovery attempted |
| -- | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: |
| high_voltage_probe_kit | 10 | 10 | 10 | 10 | 10 | 10 | 10 | 10 | 10 | 100.00% | 100.00% | 6 | 4 | 0 |
| tooling_fixture_box | 10 | 10 | 10 | 10 | 10 | 10 | 10 | 10 | 10 | 100.00% | 100.00% | 8 | 2 | 0 |
| material_spare_igbt | 10 | 10 | 10 | 10 | 8 | 8 | 8 | 8 | 8 | 80.00% | 80.00% | 6 | 2 | 0 |

## Overall

- Overall system success: 28/30 = 93.33%
- Overall reset clean: 30/30
- Primary lift success: 20/30
- State-verified lift completion after controller abort: 8/30
- Scene-detached recovery attempted: 0/30

## Failure Stages

- SUCCESS: 28
- CONTACT_LEFT_ONLY: 2

## Notes

- force gate was disabled for this contact-feedback baseline.
- /gripper/contact/force was recorded only; it was not used as a success or failure gate.
- lift_state_verified indicates primary controller execution returned an abort, but measured TCP final pose and holding state satisfied validation bounds; these cases are reported separately and are not counted as primary lift success.
