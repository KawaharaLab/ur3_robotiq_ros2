# Synchronization QA

Input: `/home/tsumura/data/10times_test/20260505_123511/bag/session_20260505_123511`

No timestamp correction was applied. Offsets are `bag receive - header`.

## Topic timing

| topic | msgs | median Hz | header | median offset ms | IQR ms | p95 abs ms | max abs ms | drift ms/min | backward | duplicate | large gaps |
|---|---:|---:|:---:|---:|---:|---:|---:|---:|---:|---:|---:|
| /camera_fixed/realsense2_camera/color/image_raw/compressed | 929 | 14.882 | yes | 69.984 | 3.529 | 70.529 | 1419.286 | -3.790 | 0 | 0 | 5 |
| /camera_wrist/realsense2_camera/color/image_raw/compressed | 926 | 14.882 | yes | 69.670 | 3.377 | 70.285 | 1395.354 | 1.311 | 0 | 0 | 7 |
| /force_torque/left | 6263 | 100.006 | yes | 0.075 | 0.049 | 0.322 | 15.431 | -0.005 | 0 | 0 | 0 |
| /force_torque/right | 6175 | 100.006 | yes | 0.075 | 0.058 | 0.322 | 4.095 | -0.003 | 0 | 0 | 0 |
| /gelsight/left/image_raw/compressed | 425 | 14.993 | yes | 5.003 | 0.122 | 5.163 | 14.120 | 0.092 | 0 | 0 | 0 |
| /gelsight/right/image_raw/compressed | 940 | 15.004 | yes | 5.301 | 0.147 | 5.875 | 6.750 | 0.028 | 0 | 0 | 0 |
| /joint_states | 7828 | 125.012 | yes | 0.186 | 0.107 | 0.543 | 14.225 | -0.010 | 0 | 0 | 0 |
| /robotiq_2f_gripper/finger_distance_mm | 1223 | 20.806 | no | n/a | n/a | n/a | n/a | n/a | 0 | 0 | 30 |
| /scaled_joint_trajectory_controller/controller_state | 1 | n/a | yes | 80909.175 | 0.000 | 80909.175 | 80909.175 | n/a | 0 | 0 | 0 |
| /tf | 8944 | 125.012 | yes | 0.267 | 0.142 | 0.637 | 14.232 | -0.013 | 0 | 1118 | 0 |
| /tf_static | 3 | 0.000 | yes | 151534.169 | 2611346.849 | 4851839.868 | 5374096.056 | -60000.000 | 1 | 0 | 0 |
| /trial_marker | 20 | 0.195 | no | n/a | n/a | n/a | n/a | n/a | 0 | 0 | 0 |

## Event coverage

- trial_run_received: 0
- phase1_goal_sent: 0
- phase1_action_succeeded: 0
- phase2_close_goal_sent: 0
- phase2_close_action_succeeded: 0
- hold_start: 0
- hold_end: 0
- phase3_open_goal_sent: 0
- phase3_open_action_succeeded: 0
- sequence_finished: 0
- hold duration(s): n/a

## Loading coverage

- unavailable: stamped close/action boundary events are absent

## Synchronization

- ft_left_minus_nearest_right: `{"pairs": 6174, "median_ms": -1.815303, "iqr_ms": 0.023352250000000074, "p95_abs_ms": 1.8649688, "max_abs_ms": 4.032156, "definition": "A header timestamp minus nearest B header timestamp"}`
- gelsight_left_minus_nearest_right: `{"pairs": 425, "median_ms": 13.271972, "iqr_ms": 8.967536, "p95_abs_ms": 22.8229876, "max_abs_ms": 31.683565, "definition": "A header timestamp minus nearest B header timestamp"}`
- realsense_wrist_minus_nearest_fixed: `{"pairs": 925, "median_ms": 23.867188, "iqr_ms": 0.07641600000000182, "p95_abs_ms": 24.0510744, "max_abs_ms": 43.396484, "definition": "A header timestamp minus nearest B header timestamp"}`
- ft_right_observed_signal_lag: `{"target_lag_ms": 200.0, "normalized_correlation": 0.11252712138732034, "reliable": false, "positive_lag_means": "target signal occurs later than reference", "correction_applied": false}`
- ft_left_to_gelsight_left_observed_lag: `{"target_lag_ms": -840.0, "normalized_correlation": 0.1931045917564034, "reliable": false, "positive_lag_means": "target signal occurs later than reference", "correction_applied": false, "interpretation": "Observed signal lag includes physical deformation/force response and must not be used as a timestamp correction."}`
- ft_right_to_gelsight_right_observed_lag: `{"target_lag_ms": -400.0, "normalized_correlation": 0.10318421858095093, "reliable": false, "positive_lag_means": "target signal occurs later than reference", "correction_applied": false, "interpretation": "Observed signal lag includes physical deformation/force response and must not be used as a timestamp correction."}`

## Warnings

- /force_torque/right: starts 0.869 s after bag start
- /gelsight/left/image_raw/compressed: ends 34.330 s before bag end
- /robotiq_2f_gripper/finger_distance_mm: no header; only bag receive time is available
- /scaled_joint_trajectory_controller/controller_state: p95 |bag-header| is 80909.175 ms
- /tf_static: 1 backward header/timestamp jumps
- /tf_static: p95 |bag-header| is 4851839.868 ms
- /trial_marker: ends 0.827 s before bag end
- /trial_marker: no header; only bag receive time is available
- /trial_marker: starts 3.112 s after bag start
- ft_left_to_gelsight_left_observed_lag: cross-correlation is weak (0.193); lag is not identifiable
- ft_right_observed_signal_lag: cross-correlation is weak (0.113); lag is not identifiable
- ft_right_to_gelsight_right_observed_lag: cross-correlation is weak (0.103); lag is not identifiable
