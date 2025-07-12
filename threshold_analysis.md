# Automotive Perception System - Threshold Analysis

This document provides a comprehensive list of all threshold values used throughout the automotive perception system. These thresholds are critical for fine-tuning the system's behavior and decision-making logic.

## 1. Configurable Parameters (Python)

These are the main configurable parameters that can be adjusted:

```python
# From Parameters class in automotive_perception_emulator.py
innovation_check_dx_threshold: float = 50.0          # Innovation check X distance threshold
innovation_check_dy_threshold: float = 50.0          # Innovation check Y distance threshold  
implausible_vy_thresh_la_hypo: float = 8.0          # Implausible VY threshold for LA hypothesis
implausible_rcs_thresh: float = -9.5                # Implausible RCS threshold
max_longitudinal_distance_for_rcs_countermeasure: float = 20.0  # Max distance for RCS countermeasure
elevation_check_dx_limits: List[float] = [0.0, 100.0]          # Elevation check X limits
elevation_check_dz_thresholds: List[float] = [2.0, 3.0]        # Elevation check Z thresholds
min_vru_micro_doppler_cycles: int = 1               # Minimum VRU micro doppler cycles
split_detection_cnt_max_val: int = 3                # Split detection count max value
```

## 2. Movement and Velocity Thresholds

### 2.1 Probability Movement Thresholds
```cpp
// From inputFile.cpp
constexpr vfc::float32_t pMovingThreshold{0.15F};                    // Moving probability threshold
constexpr vfc::float32_t hasBeenObservedMovingLow{0.5F};            // Has been observed moving low threshold
constexpr vfc::float32_t isCurrentlyMoving{0.01F};                 // Is currently moving threshold
constexpr vfc::float32_t minProbHasBeenObsMoving{0.95F};           // Min prob has been observed moving
constexpr vfc::float32_t maxProbMoving{0.15F};                     // Max prob moving threshold
```

### 2.2 Velocity Thresholds  
```cpp
// From inputFile.cpp
constexpr vfc::float32_t movingVelocityThreshold{2.0F};             // Moving velocity threshold
constexpr vfc::float32_t stationaryVelocityThreshold{1.0F};         // Stationary velocity threshold
constexpr vfc::float32_t lowerAbsSpeedThreshold{0.5F};              // Lower absolute speed threshold
constexpr vfc::float32_t upperAbsVxThreshold{4.0F};                 // Upper absolute VX threshold
constexpr vfc::float32_t xVelAbsThreshold{4.0F};                    // X velocity absolute threshold
constexpr vfc::float32_t yVelAbsThreshold{0.7F};                    // Y velocity absolute threshold
constexpr vfc::float32_t absVelThreshold{1.7F};                     // Absolute velocity threshold
constexpr vfc::float32_t absVelocityThreshold{0.3F};                // Absolute velocity threshold (pedestrian)
constexpr vfc::float32_t velThreshold{0.5F};                        // General velocity threshold
constexpr vfc::float32_t vxThreshold{2.2F};                         // VX threshold
constexpr vfc::float32_t vyThreshold{0.4F};                         // VY threshold
constexpr vfc::float32_t minVxThreshold{2.0F};                      // Minimum VX threshold
constexpr vfc::float32_t maxVyThreshold{0.4F};                      // Maximum VY threshold
constexpr vfc::float32_t minAbsVelOverGround{4.5F};                 // Min absolute velocity over ground
constexpr vfc::float32_t maxVelocityOverGroundThreshold{0.98F};     // Max velocity over ground threshold
```

### 2.3 Crossing Speed Thresholds
```cpp
// From inputFile.cpp
constexpr vfc::float32_t minSpeedForCrossingWNJ{4.6F};              // Min speed for crossing WNJ
constexpr vfc::float32_t isCrossingFastThreshold = isObjectVru && isEgoTurning ? 0.88F : 
                                                   vfc::numeric_limits<vfc::float32_t>::max();
```

## 3. Angular and Turning Thresholds

### 3.1 Yaw Rate Thresholds
```cpp
// From inputFile.cpp
constexpr vfc::float32_t turningEgoYawRateThreshold{10.0F * vfc::typedDegree2Radian<vfc::float32_t>()};  // ~10 degrees
constexpr vfc::float32_t angleDtThreshold{0.087266F};               // ~5 deg/s
constexpr vfc::float32_t angleDtThreshold{0.0262F};                 // ~1.5 deg/s
constexpr vfc::float32_t angleDtThreshold{0.523599F};               // ~30 deg/s
constexpr vfc::float32_t accelerationThreshold{0.15F};              // Acceleration threshold
constexpr vfc::float32_t angle_dt_threshold{0.012F};                // Angle dt threshold
```

### 3.2 Alpha Innovation Thresholds
```cpp
// From inputFile.cpp
constexpr vfc::float32_t alphaInnoThreshold{4.0F * vfc::typedDegree2Radian<vfc::float32_t>()};
constexpr vfc::float32_t highAlphaThreshold{2.0F};                  // High alpha threshold
constexpr vfc::float32_t highAlphaThreshold{0.02F};                 // ~1.15 degree
constexpr vfc::float32_t radarAngleInnoThreshold{5.0F};             // Radar angle innovation threshold
constexpr vfc::float32_t videoAngleInnoThreshold{2.0F};             // Video angle innovation threshold
```

## 4. RCS (Radar Cross Section) Thresholds

### 4.1 General RCS Thresholds
```cpp
// From inputFile.cpp
constexpr vfc::float32_t rcsThreshold{-8.1F};                       // RCS threshold (general)
constexpr vfc::float32_t rcsThreshold{-7.5F};                       // RCS threshold (SMO)
constexpr vfc::float32_t rcsThreshold{-15.0F};                      // RCS threshold (radar only)
constexpr vfc::float32_t rcsThreshold{-6.0F};                       // RCS threshold (elevation)
constexpr vfc::float32_t rcsHighThreshold{-3.0F};                   // RCS high threshold
constexpr vfc::float32_t rcsLowThreshold{-12.0F};                   // RCS low threshold
constexpr vfc::float32_t rcsThresholdPed{3.5F};                     // RCS threshold for pedestrian
constexpr vfc::float32_t rcsThresholdPed{0.0F};                     // RCS threshold for pedestrian (variant)
constexpr vfc::float32_t rcsThresholdTwhoWheeler{6.5F};             // RCS threshold for two-wheeler
```

## 5. Innovation Thresholds

### 5.1 DR (Distance/Range) Innovation Thresholds
```cpp
// From inputFile.cpp
constexpr vfc::float32_t drInnovationThreshold{1.2F};               // DR innovation threshold
constexpr vfc::float32_t drInnovationRelaxThreshold{0.78F};         // DR innovation relax threshold
constexpr vfc::float32_t drInnoThreshold{3.0F};                     // DR innovation threshold
constexpr vfc::float32_t drInnovThresh{0.7F};                       // DR innovation threshold (short)
constexpr vfc::float32_t drVideoInnoThreshold{6.0F};                // DR video innovation threshold
constexpr vfc::float32_t drVideoInnoThreshold{7.7F};                // DR video innovation threshold (variant)
constexpr vfc::float32_t videoDrInnoThreshold{2.2F};                // Video DR innovation threshold
constexpr vfc::float32_t normInnoRadarDr{2.0F};                     // Normalized radar DR innovation
```

### 5.2 DX Innovation Thresholds
```cpp
// From inputFile.cpp
constexpr vfc::float32_t dxInnovationThresholdDefault{1.6F};        // DX innovation threshold default
constexpr vfc::float32_t dxInnovationThresholdVru{1.5F};            // DX innovation threshold VRU
constexpr vfc::float32_t dxInnovationThresholdIfVyUnreliable{1.1F}; // DX innovation threshold if VY unreliable
constexpr vfc::float32_t dxInnovationThresholdIfVeryLowRCSOnly{1.5F}; // DX innovation threshold if very low RCS
constexpr vfc::float32_t dxInnovationThresholdIfVruLowRCSInCloseRange{1.4F}; // DX innovation threshold VRU low RCS close range
constexpr vfc::float32_t vyUnreliableThreshold{1.9F};               // VY unreliable threshold
constexpr vfc::float32_t vyUnreliableAccumulatedThreshold{2.6F};    // VY unreliable accumulated threshold
```

## 6. Distance and Position Thresholds

### 6.1 Distance Thresholds
```cpp
// From inputFile.cpp
constexpr vfc::float32_t dxThreshold{35.0F};                        // DX threshold (close range)
constexpr vfc::float32_t dyThreshold{3.0F};                         // DY threshold
constexpr vfc::float32_t dyThreshold{1.2F};                         // DY threshold (variant)
constexpr vfc::float32_t lowDyThreshold{2.5F};                      // Low DY threshold
constexpr vfc::float32_t longDistThresh{10.0F};                     // Long distance threshold
constexpr vfc::float32_t lateralPositionThresholdFar{1.25F};        // Lateral position threshold far
constexpr vfc::float32_t lateralPositionThresholdClose{6.0F};       // Lateral position threshold close
constexpr vfc::float32_t maxAbsDy{0.5F};                            // Maximum absolute DY
constexpr vfc::float32_t laneWidthThresh{2.5F};                     // Lane width threshold
```

### 6.2 Elevation Thresholds
```cpp
// From inputFile.cpp
constexpr vfc::float32_t dzThreshold{0.1F};                         // DZ threshold
constexpr vfc::float32_t dzThreshold{2.1F};                         // DZ threshold (variant)
constexpr vfc::float32_t dzThresholdforSMO{0.3F};                   // DZ threshold for SMO
constexpr vfc::float32_t dzThreshForCar{1.9F};                      // DZ threshold for car
constexpr vfc::float32_t dzThreshold_1{1.7F};                       // DZ threshold variant 1
constexpr vfc::float32_t dzThreshold_2{2.2F};                       // DZ threshold variant 2
// Python elevation thresholds
allowed_dz_threshold = 2.0 + (obj_dx / 100.0) * (3.0 - 2.0)        // Linear interpolation between 2.0-3.0
```

## 7. Time-based Thresholds

### 7.1 Cycle Count Thresholds
```cpp
// From inputFile.cpp
constexpr vfc::uint16_t numCyclesExistingThreshold{7U};             // Number of cycles existing threshold
constexpr vfc::uint16_t objectAgeThreshold{12U};                    // Object age threshold
constexpr vfc::uint16_t minCyclesAlive{1U};                         // Minimum cycles alive
constexpr vfc::uint16_t minCyclesAlive{4U};                         // Minimum cycles alive (variant)
constexpr vfc::uint16_t statLocThreshold{4U};                       // Stationary location threshold
constexpr vfc::uint16_t minObjectAlive{25U};                        // Minimum object alive
constexpr vfc::uint16_t minAgeOfGoodFusedObject{5U};                // Minimum age of good fused object
constexpr vfc::uint16_t minCyclesWithVideoConfirmation{5U};         // Min cycles with video confirmation
constexpr vfc::uint16_t recentRadarOnlyObjectThreshold{40U};        // Recent radar only object threshold
```

### 7.2 Update Count Thresholds
```cpp
// From inputFile.cpp
constexpr vfc::uint8_t lowerRadarConfirmationThresholdDefault{1U};  // Lower radar confirmation threshold default
constexpr vfc::uint8_t lowerRadarConfirmationThresholdForVideoGhost{0U}; // Lower radar confirmation threshold for video ghost
constexpr vfc::uint8_t oncomingLocationThreshold = unreliableMovingVru ? 1U : 0U; // Oncoming location threshold
constexpr vfc::uint8_t cyclesSinceLastVideoUpdateWithAngularVelocityThreshold{
    isEgoTurning ? 3U : 12U};                                       // Cycles since last video update with angular velocity
constexpr vfc::uint8_t minCornerRadarUpdatesThresholdForPlausibleRcs{11U}; // Min corner radar updates threshold
constexpr vfc::uint8_t maxCyclesThresholdSinceLastCornerSensorUpdate{19U}; // Max cycles threshold since last corner sensor update
constexpr vfc::uint8_t maxCyclesSinceLastVideoUpdate4TrustworthyCheck{1U}; // Max cycles since last video update for trustworthy check
constexpr vfc::uint8_t thresholdTrustworthyRadar{15U};              // Threshold trustworthy radar
constexpr vfc::uint8_t minCyclesSinceLastSensorUpdate{10U};         // Min cycles since last sensor update
constexpr vfc::uint8_t minCyclesSinceLastVideoUpdate{5U};           // Min cycles since last video update
```

## 8. Ratio and Percentage Thresholds

### 8.1 Measurement Ratio Thresholds
```cpp
// From inputFile.cpp
constexpr vfc::float32_t minRatioOfUpdatesToPrediction{0.7F};       // Min ratio of updates to prediction
constexpr vfc::float32_t minRatioOfUpdatesToAliveCount{0.7F};       // Min ratio of updates to alive count
constexpr vfc::float32_t ratioBadDetectionThreshold{0.375F};        // Ratio bad detection threshold
```

### 8.2 Probability Thresholds
```cpp
// From inputFile.cpp
constexpr vfc::float32_t pPedestrianThresholdHigh{0.7F};            // Pedestrian probability threshold high
constexpr vfc::float32_t standingObjectProb{0.1F};                  // Standing object probability
constexpr vfc::float32_t wExistThreshold{0.4F};                     // W exist threshold
constexpr vfc::float32_t groundReflexTreshold{0.8F};                // Ground reflex threshold
constexpr vfc::float32_t statThresh{0.5F};                          // Stationary threshold
constexpr vfc::float32_t hbomThresh{0.9F};                          // HBOM threshold
```

## 9. Specific Function Thresholds

### 9.1 Video Ghost Detection
```cpp
// From inputFile.cpp and automotive_perception_emulator.py
total_num_front_center_location_radar_updates > 0 && < 3           // Initial radar update phase
total_num_video_updates > 3                                        // Tracked by video
total_num_front_left_corner_updates == 0 && total_num_front_right_corner_updates == 0  // No corner radar
number_micro_doppler_cycles == 0                                   // No micro doppler
expected_vr_high_enough_for_mu_doppler_counter > 0                 // Expected VR high enough
rcs < -15.0                                                        // Very low RCS
```

### 9.2 Micro Doppler Check
```python
# From automotive_perception_emulator.py
object_age_threshold = 12                                          # Object age threshold
upper_abs_vy_threshold = 3.2 if is_object_old else 99.0          # Upper absolute VY threshold
crossing_vru_conditions: abs_vel_over_ground[1] > 0.5 and < upper_abs_vy_threshold and abs_vel_over_ground[0] < 4.0
stationary_vru_conditions: abs_vel_over_ground[0] < 0.5 and abs_vel_over_ground[1] < 0.5
```

### 9.3 Innovation Check
```python
# From automotive_perception_emulator.py
dx_innovation_threshold = 1.6  # Default
dx_innovation_threshold = 1.5  # If abs(state.x) < 20.0 and is_object_vru
dx_innovation_threshold = 1.1  # If rcs < -5.0 and vy_unreliable_accumulated > 1.9
dx_innovation_threshold = 1.5  # If rcs < -15.0
```

### 9.4 Ego Vehicle Motion
```python
# From automotive_perception_emulator.py
turning_ego_yaw_rate_threshold = 10.0 * (pi / 180.0)             # 10 degrees in radians
acceleration_threshold = 0.15                                     # Acceleration threshold
angle_dt_threshold = 0.012                                        # Angle dt threshold
ego_driving_straight_radius = 2500.0                              # Ego driving straight radius
```

### 9.5 Relevant Area Check
```python
# From automotive_perception_emulator.py
is_object_in_relevant_area = (
    (abs(obj_data.state.y) <= 1.25 and obj_data.state.x < 120.0) or
    (abs(obj_data.state.y) <= 6.0 and obj_data.state.x < 10.0)
)
```

## 10. Threshold Reduction/Adaptation Logic

### 10.1 Stopping Split Threshold Reduction
```cpp
// From inputFile.cpp
constexpr vfc::int32_t thresholdReductionStoppingSplit{2};          // Threshold reduction stopping split
// Applied as: static_cast<vfc::uint8_t>(vfc::max(0, static_cast<vfc::int32_t>(threshold) - thresholdReductionStoppingSplit))
```

### 10.2 Dynamic Threshold Calculation
```javascript
// From script.js
const threshold = 1.5 + (range / 100) + (rcs / 20) + (vyUnreliableAccumulated / 10);  // Dynamic threshold calculation
```

## 11. Hardcoded Limits and Boundaries

### 11.1 Numeric Limits
```cpp
// From inputFile.cpp
vfc::numeric_limits<vfc::float32_t>::max()                         // Float max value
vfc::numeric_limits<vfc::uint8_t>::max()                          // Uint8 max value (255)
vfc::numeric_limits<vfc::float32_t>::epsilon()                     // Float epsilon
```

### 11.2 Object Dimension Limits
```cpp
// From inputFile.cpp
const bool isTooWideForVru{objWidth > 1.5F};                       // Too wide for VRU threshold
```

## 12. Normalization Factors

### 12.1 Innovation Normalization
```cpp
// From inputFile.cpp
const vfc::float32_t normFactorInnoRadarAlpha = isStationary ? 0.5F : 1.0F;           // Radar alpha normalization
const vfc::float32_t normFactorInnoVideoDr = isStationary && (distanceToCenter < 20.0F) ? 0.6F : 1000.0F; // Video DR normalization
```

## Summary

This comprehensive analysis reveals **over 150 different threshold values** used throughout the automotive perception system. The thresholds can be categorized into:

1. **Configurable Parameters** (9 values) - Can be easily adjusted via the Parameters class
2. **Movement/Velocity Thresholds** (25+ values) - Control object motion detection
3. **RCS Thresholds** (10+ values) - Control radar cross-section based decisions
4. **Innovation Thresholds** (15+ values) - Control sensor fusion innovation limits
5. **Distance/Position Thresholds** (15+ values) - Control spatial decision boundaries
6. **Time-based Thresholds** (20+ values) - Control temporal decision making
7. **Ratio/Probability Thresholds** (10+ values) - Control probabilistic decisions
8. **Function-specific Thresholds** (40+ values) - Control specific algorithm behaviors

### Recommendations for Fine-tuning:

1. **Start with configurable parameters** in the Parameters class for initial adjustments
2. **Use the Python emulator** to test threshold changes before modifying C++ code
3. **Consider threshold interdependencies** - some thresholds work together
4. **Document threshold changes** and their impact on system behavior
5. **Use data-driven approaches** to optimize threshold values based on real-world performance

The thresholds are designed to handle various automotive scenarios including:
- VRU (Vulnerable Road User) detection and tracking
- Radar-only vs. fused object processing
- Video ghost detection and suppression
- Innovation-based quality assessment
- Micro-doppler signature validation
- Elevation and positioning checks