# Automotive Perception Application - Configurable Thresholds and Parameters

## Overview
This document provides a comprehensive list of all configurable thresholds and parameters for each function in the automotive perception application. These parameters can be fine-tuned to optimize the system's performance for different scenarios and requirements.

## Global Configuration Parameters

### Parameters Class (Python Emulator)
Located in `automotive_perception_emulator.py` lines 125-140:

```python
class Parameters:
    is_micro_doppler_check_enabled: bool = True
    min_vru_micro_doppler_cycles: int = 1
    is_micro_doppler_check_on_crossing_vru_applied: bool = True
    is_micro_doppler_check_on_stationary_vru_applied: bool = True
    innovation_check_dx_threshold: float = 50.0
    innovation_check_dy_threshold: float = 50.0
    implausible_vy_thresh_la_hypo: float = 8.0
    split_detection_cnt_max_val: int = 3
    implausible_rcs_thresh: float = -9.5
    max_longitudinal_distance_for_rcs_countermeasure: float = 20.0
    elevation_check_dx_limits: List[float] = [0.0, 100.0]
    elevation_check_dz_thresholds: List[float] = [2.0, 3.0]
```

## Function-Specific Thresholds

### 1. applySuppressionUntilNextVideoUpdateCheck
**Purpose**: Checks if object is suppressed until next video update
**Configurable Parameters**: None (boolean flag check)
**Threshold**: `obj.is_suppressed_until_next_video_update == True`

### 2. applyPostProcessVideoOtcCheck
**Purpose**: Checks if object is suppressed due to video OTC post processing
**Configurable Parameters**: None (boolean flag check)
**Threshold**: `obj.is_suppressed_due_to_video_otc_post_processing == True`

### 3. isMovingTowardsEgoLane
**Purpose**: Determines if object is moving towards ego lane
**Configurable Parameters**: None (mathematical check)
**Logic**: `is_negative(dy_obj * vy_obj_rel) OR is_negative(dy_obj * vy_obj_over_ground)`

### 4. isDepObjProbablyVideoGhost
**Purpose**: Identifies potential video ghost objects
**Configurable Thresholds**:
- `total_num_front_center_location_radar_updates_min`: 0 (> 0)
- `total_num_front_center_location_radar_updates_max`: 3 (< 3)
- `updates_since_last_radar_update_threshold`: 0 (== 0)
- `total_num_video_updates_threshold`: 3 (> 3)
- `micro_doppler_cycles_threshold`: 0 (== 0)
- `expected_vr_counter_threshold`: 0 (> 0)
- `very_low_rcs_threshold`: -15.0 (< -15.0)

### 5. applyUnreliableAngularVelocityCheck
**Purpose**: Detects unreliable angular velocity in objects
**Configurable Thresholds**:
- `p_moving_threshold`: 0.15
- `turning_ego_yaw_rate_threshold`: 10.0 degrees (converted to radians)
- `num_cycles_existing_threshold`: 7
- `crossing_fast_threshold_vru_ego_turning`: 0.88
- `crossing_fast_threshold_vru`: 0.68
- `crossing_fast_threshold_standing_car`: 0.35
- `crossing_fast_threshold_default`: 0.5
- `cycles_since_last_video_update_with_angular_velocity_threshold`: 1 (VRU), 0 (non-VRU)
- `dr_innovation_threshold`: 3.0
- `alpha_innovation_threshold`: 4.0 degrees (converted to radians)

### 6. applyUpdatedWithStatLocWithHighMDopplerWithOutgoingVrCheck
**Purpose**: Checks VRU updated with stationary location and high micro-doppler with outgoing VR
**Configurable Thresholds**:
- `abs_vel_x_threshold`: 0.2 (< 0.2)
- `abs_vel_y_threshold`: 1.0 (> 1.0)
- `object_y_position_threshold`: 0.5 (abs(y) < 0.5)

### 7. applyIsMeasuredRatioCheckForFastWnj
**Purpose**: Checks measurement ratio for fast WNJ objects
**Configurable Thresholds**:
- `fast_crossing_velocity_threshold`: 4.6 (abs(vy) > 4.6)
- `measurement_ratio_threshold`: 0.7 (< 0.7)
- `max_video_updates_threshold`: 5 (<= 5)
- `min_cycles_existing`: 1 (> 1)
- `max_cycles_existing`: 255 (< 255)

### 8. applyWaterSprinklersCheckAcc
**Purpose**: Detects water sprinkler artifacts for ACC
**Configurable Thresholds**:
- `avg_dx_innovation_threshold`: (dynamic based on conditions)
- `rcs_threshold`: (dynamic based on conditions)
- `p_non_obstacle_threshold`: (dynamic based on conditions)

### 9. applyNonCrossingObjectCheck
**Purpose**: Identifies objects that appear crossing but are probably not moving
**Configurable Thresholds**:
- `appears_crossing_velocity_threshold`: 0.5 (vy > 0.5)
- `prob_currently_moving_threshold`: 0.1 (< 0.1)
- `prob_has_been_observed_moving_threshold`: 0.1 (< 0.1)
- `micro_doppler_cycles_threshold`: 0 (== 0)
- `total_num_cycles_with_oncoming_locations_threshold`: 0 (== 0)

### 10. applyWaterSprinklesCheck
**Purpose**: Detects water sprinkler/ground reflection artifacts
**Configurable Thresholds**:
- `avg_dx_innovation_threshold`: (dynamic)
- `rcs_threshold`: (dynamic)
- `elevation_threshold`: (dynamic)
- `p_non_obstacle_threshold`: (dynamic)

### 11. applyIsMeasuredRatioCheckForRadarOnlyLongitudinallyMoving
**Purpose**: Checks measurement ratio for radar-only longitudinally moving objects
**Configurable Thresholds**:
- `min_cycles_existing`: 3
- `lateral_position_threshold`: 1.25 (abs(y) <= 1.25)
- `measurement_ratio_threshold`: (dynamic based on age)

### 12. applyMicroDopplerCheck
**Purpose**: Validates micro-doppler signatures for VRU objects
**Configurable Thresholds**:
- `micro_doppler_check_enabled`: True/False
- `min_vru_micro_doppler_cycles`: 1
- `micro_doppler_check_on_crossing_vru_applied`: True/False
- `micro_doppler_check_on_stationary_vru_applied`: True/False
- `expected_vr_counter_threshold`: 2 (>= 2)
- `front_center_radar_updates_min`: 0 (> 0)
- `corner_radar_updates_max`: 1 (< 1)
- `object_age_threshold`: 12
- `front_center_radar_updates_threshold`: 8 (> 8)
- `upper_abs_vy_threshold_old`: 3.2
- `upper_abs_vy_threshold_young`: 99.0
- `crossing_vy_min_threshold`: 0.5 (> 0.5)
- `crossing_vx_max_threshold`: 4.0 (< 4.0)
- `stationary_vx_threshold`: 0.5 (< 0.5)
- `stationary_vy_threshold`: 0.5 (< 0.5)

### 13. applyRadarOnlyRcsAndDrInnovationLimit
**Purpose**: Limits radar-only objects with high innovation and low RCS
**Configurable Thresholds**:
- `dr_innovation_threshold`: 1.2 (abs(innovation) > 1.2)
- `rcs_threshold`: -15.0 (< -15.0)

### 14. applyElevationCheck
**Purpose**: Checks object elevation appropriateness
**Configurable Thresholds**:
- `stationary_velocity_threshold`: 1.0 (< 1.0)
- `recent_video_update_threshold`: 10 (< 10)
- `elevation_dx_limits`: [0.0, 100.0]
- `elevation_dz_thresholds`: [2.0, 3.0]
- `elevation_threshold_min`: 2.0
- `elevation_threshold_max`: 3.0

### 15. applyNonPlausibleLocationChecks
**Purpose**: Checks for non-plausible locations in truck objects
**Configurable Thresholds**:
- `non_plausible_location_count_threshold`: (dynamic)
- `video_existence_threshold`: (dynamic)
- `updates_since_last_video_threshold`: (dynamic)

### 16. applyFourpluswheelerChecks
**Purpose**: Validates 4+ wheeler vehicle characteristics
**Configurable Thresholds**:
- `sensor_updates_threshold`: (dynamic)
- `velocity_threshold`: (dynamic)
- `distance_threshold`: (dynamic)
- `innovation_threshold`: (dynamic)
- `rcs_threshold`: (dynamic)
- `elevation_threshold`: (dynamic)
- `video_existence_threshold`: (dynamic)
- `prob_moving_threshold`: (dynamic)
- `micro_doppler_threshold`: (dynamic)
- `stationary_locations_threshold`: (dynamic)
- `facing_angle_threshold`: (dynamic)

### 17. applySplitCheckDisqualifyObjectForFunctions
**Purpose**: Disqualifies objects based on split detection
**Configurable Thresholds**:
- `split_detection_cnt_max_val`: 3 (from Parameters class)
- `stopping_split_counter_threshold`: (dynamic)

### 18. applyOrientationConsistencyCheck
**Purpose**: Checks consistency between object orientation and velocity
**Configurable Thresholds**:
- `dimension_threshold`: (dynamic)
- `cycles_existing_threshold`: (dynamic)
- `video_updates_threshold`: (dynamic)
- `radar_updates_threshold`: (dynamic)
- `orientation_unreliable_count_threshold`: (dynamic)

### 19. modifyUnreliableOrientationCount
**Purpose**: Modifies orientation unreliable count
**Configurable Thresholds**:
- `cycles_no_orientation_update_threshold`: (dynamic)
- `orientation_unreliable_count_increment`: (dynamic)

### 20. applyStationaryVruVideoGhostCheck
**Purpose**: Detects stationary VRU video ghosts
**Configurable Thresholds**:
- `stationary_vx_threshold`: 0.5 (< 0.5)
- `stationary_vy_threshold`: 0.5 (< 0.5)
- `high_rcs_threshold`: -5.0 (> -5.0)
- `low_video_existence_threshold`: 0.1 (< 0.1)
- `recent_video_update_threshold`: 5 (< 5)

### 21. applyInnovationCheck
**Purpose**: Checks for high dx innovation in relevant range
**Configurable Thresholds**:
- `innovation_check_dx_threshold`: 50.0 (from Parameters class)
- `innovation_check_dy_threshold`: 50.0 (from Parameters class)
- `dx_innovation_threshold_default`: 1.6
- `dx_innovation_threshold_close_vru`: 1.5 (abs(x) < 20.0 && VRU)
- `dx_innovation_threshold_low_rcs`: 1.1 (rcs < -5.0 && vy_unreliable > 1.9)
- `dx_innovation_threshold_very_low_rcs`: 1.5 (rcs < -15.0)

### 22. calcDxInnovationThreshold
**Purpose**: Calculates dynamic dx innovation threshold
**Configurable Thresholds**:
- `position_threshold`: 20.0 (abs(x) < 20.0)
- `rcs_threshold_1`: -5.0 (< -5.0)
- `rcs_threshold_2`: -15.0 (< -15.0)
- `vy_unreliable_threshold`: 1.9 (> 1.9)
- `base_threshold`: 1.6
- `close_vru_threshold`: 1.5
- `low_rcs_threshold`: 1.1
- `very_low_rcs_threshold`: 1.5

### 23. applyImplausibleVyVruCheck
**Purpose**: Checks for implausible VY velocity in VRU objects
**Configurable Thresholds**:
- `implausible_vy_thresh_la_hypo`: 8.0 (from Parameters class)
- `turning_ego_yaw_rate_threshold`: 10.0 degrees (converted to radians)

### 24. applySensorBasedInnoCheck
**Purpose**: Checks for high bad sensor innovation count
**Configurable Thresholds**:
- `bad_sensor_innovation_count_threshold`: (dynamic)
- `position_relevance_threshold`: (dynamic)

### 25. applyImplausibleVideoTtcForVru
**Purpose**: Checks for implausible video TTC in VRU objects
**Configurable Thresholds**:
- `recent_video_update_threshold`: 1 (< 1)
- `max_video_ttc_threshold`: float('inf') (== max_float)

### 26. applyVyInconsistentCheck
**Purpose**: Checks for inconsistent VY measurements
**Configurable Thresholds**:
- `vy_inconsistent_threshold`: (dynamic)
- `vy_unreliable_accumulated_threshold`: (dynamic)
- `position_threshold`: (dynamic)

### 27. applyVideoHandleSharedCheck
**Purpose**: Checks for shared video handles between objects
**Configurable Thresholds**:
- `prob_moving_threshold`: (dynamic)
- `video_update_threshold`: (dynamic)
- `handle_comparison_threshold`: (dynamic)

### 28. modifyBadSensorBasedInnoCount
**Purpose**: Updates bad sensor innovation counter
**Configurable Thresholds**:
- `radar_innovation_threshold`: (dynamic)
- `video_innovation_threshold`: (dynamic)
- `position_threshold`: (dynamic)

### 29. applyRadarOnlyNLDCheck
**Purpose**: Checks radar-only objects for NLD candidates
**Configurable Thresholds**:
- `ego_driving_straight_radius`: 2500.0
- `acceleration_threshold`: 0.15
- `angle_dt_threshold`: 0.012
- `relevant_area_y_threshold_1`: 1.25 (abs(y) <= 1.25)
- `relevant_area_x_threshold_1`: 120.0 (x < 120.0)
- `relevant_area_y_threshold_2`: 6.0 (abs(y) <= 6.0)
- `relevant_area_x_threshold_2`: 10.0 (x < 10.0)
- `object_age_threshold`: 3 (>= 3)
- `max_cycles_threshold`: 30 (>= 30)
- `high_lateral_velocity_threshold`: 3.0 (abs(vy) > 3.0)
- `close_range_x_threshold`: 8.0 (x < 8.0)
- `close_range_y_threshold`: 4.0 (abs(y) < 4.0)

### 30. applyRadarOnlyStationaryCheck
**Purpose**: Checks for radar-only stationary objects
**Configurable Thresholds**:
- `stationary_vx_threshold`: 0.3 (< 0.3)
- `stationary_vy_threshold`: 0.3 (< 0.3)

### 31. applyIsMeasuredRatioCheckForStandingLongiVru
**Purpose**: Checks measurement ratio for standing longitudinal VRU
**Configurable Thresholds**:
- `prob_moving_threshold`: (dynamic)
- `cycles_existing_threshold`: (dynamic)
- `measurement_history_threshold`: (dynamic)

### 32. applyImplausiblyAcceleratingVruCheck
**Purpose**: Checks for implausibly accelerating VRU
**Configurable Thresholds**:
- `close_distance_threshold`: (dynamic)
- `young_object_threshold`: (dynamic)
- `acceleration_threshold`: (dynamic)

### 33. applyUndefinedCrossingVruFromCorner
**Purpose**: Checks for undefined crossing VRU from corner radar
**Configurable Thresholds**:
- `corner_detection_threshold`: (dynamic)
- `crossing_behavior_threshold`: (dynamic)
- `prob_moving_threshold`: (dynamic)

### 34. applyImplausiblePedCheck
**Purpose**: Checks for implausible pedestrian characteristics
**Configurable Thresholds**:
- `lateral_position_threshold`: (dynamic)
- `micro_doppler_threshold`: (dynamic)
- `rcs_threshold`: (dynamic)
- `sensor_updates_threshold`: (dynamic)
- `stationary_locations_threshold`: (dynamic)
- `cycles_existing_threshold`: (dynamic)
- `innovation_threshold`: (dynamic)
- `elevation_threshold`: (dynamic)

### 35. applyImplausiblePedCheckLRR
**Purpose**: LRR-specific implausible pedestrian checks
**Configurable Thresholds**:
- `lateral_position_threshold`: (dynamic)
- `video_updates_threshold`: (dynamic)
- `micro_doppler_threshold`: (dynamic)
- `rcs_threshold`: (dynamic)
- `sensor_updates_threshold`: (dynamic)
- `expected_vr_threshold`: (dynamic)
- `stationary_locations_threshold`: (dynamic)
- `innovation_threshold`: (dynamic)
- `cycles_existing_threshold`: (dynamic)

### 36. applyImplausibleCarAtCloseRangeCheck
**Purpose**: Checks for implausible cars at close range
**Configurable Thresholds**:
- `close_range_x_threshold`: (dynamic)
- `car_dimension_threshold`: (dynamic)
- `video_updates_threshold`: (dynamic)

### 37. applyElevatedObjectCheck
**Purpose**: Checks for elevated objects with unreliable characteristics
**Configurable Thresholds**:
- `rcs_threshold`: (dynamic)
- `elevation_threshold`: (dynamic)
- `video_updates_threshold`: (dynamic)
- `cycles_existing_threshold`: (dynamic)
- `measurement_history_threshold`: (dynamic)
- `stationary_locations_threshold`: (dynamic)
- `innovation_threshold`: (dynamic)

### 38. applyBridgeCheck
**Purpose**: Checks for bridge-like object characteristics
**Configurable Thresholds**:
- `video_innovation_threshold`: (dynamic)
- `elevation_threshold`: (dynamic)
- `stationary_locations_threshold`: (dynamic)
- `video_updates_threshold`: (dynamic)

### 39. applyCornerRadarAssoWithStationaryLocationsForTheFirstTime
**Purpose**: Checks young VRU first detected by corner radar
**Configurable Thresholds**:
- `young_object_threshold`: (dynamic)
- `corner_radar_threshold`: (dynamic)
- `stationary_locations_threshold`: (dynamic)

### 40. applyInconsistentAlphaCheck
**Purpose**: Checks for inconsistent alpha innovations
**Configurable Thresholds**:
- `radar_alpha_innovation_threshold`: (dynamic)
- `video_alpha_innovation_threshold`: (dynamic)
- `position_threshold`: (dynamic)
- `video_updates_threshold`: (dynamic)

## Key Tuning Guidelines

### High-Level Parameter Categories

1. **Distance/Position Thresholds**: Control at what distances functions are active
2. **Velocity Thresholds**: Define moving vs stationary behavior
3. **Age/Cycle Thresholds**: Control how long objects must exist before checks apply
4. **Sensor Quality Thresholds**: Define minimum sensor update requirements
5. **Innovation Thresholds**: Control sensitivity to measurement inconsistencies
6. **Probability Thresholds**: Define confidence levels for object state decisions
7. **RCS Thresholds**: Control radar cross-section based filtering
8. **Elevation Thresholds**: Control height-based object validation

### Tuning Recommendations

1. **Conservative Tuning**: Increase thresholds to reduce false positives
2. **Aggressive Tuning**: Decrease thresholds to catch more edge cases
3. **Scenario-Specific**: Different thresholds for highway vs urban environments
4. **Object-Type-Specific**: Different thresholds for VRU vs vehicles
5. **Sensor-Specific**: Different thresholds based on sensor capabilities

### Critical Parameters for Safety

1. **Micro-doppler thresholds**: Critical for VRU detection
2. **Innovation thresholds**: Critical for ghost object elimination
3. **Elevation thresholds**: Critical for bridge/overpass scenarios
4. **RCS thresholds**: Critical for small object detection
5. **Velocity thresholds**: Critical for crossing object detection

## Configuration Management

### Parameter File Structure
```json
{
  "global_parameters": {
    "micro_doppler_check_enabled": true,
    "min_vru_micro_doppler_cycles": 1,
    "innovation_check_dx_threshold": 50.0,
    "innovation_check_dy_threshold": 50.0,
    "implausible_vy_thresh_la_hypo": 8.0,
    "split_detection_cnt_max_val": 3,
    "implausible_rcs_thresh": -9.5,
    "max_longitudinal_distance_for_rcs_countermeasure": 20.0,
    "elevation_check_dx_limits": [0.0, 100.0],
    "elevation_check_dz_thresholds": [2.0, 3.0]
  },
  "function_specific_parameters": {
    "micro_doppler_check": {
      "expected_vr_counter_threshold": 2,
      "object_age_threshold": 12,
      "crossing_vy_min_threshold": 0.5,
      "crossing_vx_max_threshold": 4.0,
      "stationary_velocity_threshold": 0.5
    },
    "innovation_check": {
      "dx_innovation_threshold_default": 1.6,
      "dx_innovation_threshold_close_vru": 1.5,
      "dx_innovation_threshold_low_rcs": 1.1,
      "dx_innovation_threshold_very_low_rcs": 1.5
    },
    "radar_only_nld_check": {
      "ego_driving_straight_radius": 2500.0,
      "acceleration_threshold": 0.15,
      "angle_dt_threshold": 0.012,
      "relevant_area_y_threshold": 1.25,
      "relevant_area_x_threshold": 120.0,
      "object_age_threshold": 3,
      "high_lateral_velocity_threshold": 3.0
    }
  }
}
```

This comprehensive list provides all the configurable parameters that can be fine-tuned to optimize the automotive perception system's performance for specific use cases, environments, and safety requirements.