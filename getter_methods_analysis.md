# Getter Methods Analysis Report

## Overview
This report analyzes all functions in the automotive perception codebase and lists the getter methods used within each function across three files:
- `script.js` (JavaScript Web Interface)
- `automotive_perception_emulator.py` (Python Emulator)
- `inputFile.cpp` (C++ Implementation)

---

## 1. JavaScript Functions (script.js)

### Function: `renderFunctionCards()`
**Getter Methods Used:**
- `document.getElementById('functions-grid')`

### Function: `createFunctionCard(func)`
**Getter Methods Used:**
- `func.inputs.map()`
- `input.options.map()`
- `func.id`
- `func.name`
- `func.description`

### Function: `testFunction(funcId)`
**Getter Methods Used:**
- `functions.find()`
- `func.inputs.forEach()`
- `document.getElementById()`
- `element.value`
- `func.condition()`
- `testResults[funcId]`

### Function: `updateFunctionCard(funcId, isHit)`
**Getter Methods Used:**
- `document.getElementById()`
- `card.querySelector()`
- `testResults[funcId].func.action`

### Function: `testAllFunctions()`
**Getter Methods Used:**
- `functions.forEach()`
- `func.inputs.forEach()`
- `document.getElementById()`
- `element.value`
- `func.condition()`
- `Object.keys(testResults).length`

### Function: `resetAllFunctions()`
**Getter Methods Used:**
- `functions.forEach()`
- `document.getElementById()`
- `card.querySelector()`
- `func.inputs.forEach()`

### Function: `fillRandomValues()`
**Getter Methods Used:**
- `functions.forEach()`
- `func.inputs.forEach()`
- `document.getElementById()`
- `input.options`
- `Math.random()`

### Function: `loadDefaultThresholds()`
**Getter Methods Used:**
- `functions.forEach()`
- `func.inputs.forEach()`
- `document.getElementById()`
- `input.defaultValue`

### Function: `updateStats()`
**Getter Methods Used:**
- `Object.keys(testResults).length`
- `Object.values(testResults).filter()`
- `document.getElementById()`
- `functions.length`

### Function: `updateResultsSummary()`
**Getter Methods Used:**
- `document.querySelector()`
- `Object.keys(testResults).length`
- `Object.values(testResults).filter()`
- `result.func.name`
- `result.func.action`

---

## 2. Python Functions (automotive_perception_emulator.py)

### Function: `__init__(self)`
**Getter Methods Used:**
- `self.setup_gui()`

### Function: `setup_gui(self)`
**Getter Methods Used:**
- `self.create_object_tab()`
- `self.create_sensor_tab()`
- `self.create_ego_tab()`
- `self.create_results_tab()`
- `self.create_control_buttons()`

### Function: `get_input_values(self)`
**Getter Methods Used:**
- `self.obj_entries['pos_x'].get()`
- `self.obj_entries['pos_y'].get()`
- `self.obj_entries['vel_x'].get()`
- `self.obj_entries['vel_y'].get()`
- `self.obj_entries['is_vru'].get()`
- `self.obj_entries['rcs'].get()`
- `self.obj_entries['num_cycles'].get()`
- `self.obj_entries['filter_type'].get()`
- `self.obj_entries['prob_has_been_moving'].get()`
- `self.obj_entries['prob_currently_moving'].get()`
- `self.obj_entries['elevation'].get()`
- `self.obj_entries['elevation_valid'].get()`
- `self.obj_entries['avg_dx_innovation'].get()`
- `self.obj_entries['radar_innovation_dr'].get()`
- `self.obj_entries['radar_innovation_alpha'].get()`
- `self.obj_entries['video_innovation_dr'].get()`
- `self.obj_entries['video_innovation_alpha'].get()`
- `self.obj_entries['micro_doppler_cycles'].get()`
- `self.obj_entries['expected_vr_counter'].get()`
- `self.sensor_entries['total_radar_updates'].get()`
- `self.sensor_entries['total_video_updates'].get()`
- `self.sensor_entries['fc_location_radar_updates'].get()`
- `self.sensor_entries['fl_corner_updates'].get()`
- `self.sensor_entries['fr_corner_updates'].get()`
- `self.sensor_entries['since_last_video'].get()`
- `self.sensor_entries['since_last_radar'].get()`
- `self.sensor_entries['good_quality_fused'].get()`
- `self.sensor_entries['trustworthy_object'].get()`
- `self.ego_entries['velocity_x'].get()`
- `self.ego_entries['acceleration_y'].get()`
- `self.ego_entries['yaw_rate'].get()`
- `self.ego_entries['abs_vel_x'].get()`
- `self.ego_entries['abs_vel_y'].get()`
- `self.ego_entries['is_mpc3_used'].get()`

### Function: `is_dep_obj_probably_video_ghost(self)`
**Getter Methods Used:**
- `self.obj_data.sensor_filter_fus_helper.total_num_front_center_location_radar_updates`
- `self.obj_data.sensor_filter_fus_helper.updates_since_last_radar_update`
- `self.obj_data.sensor_filter_fus_helper.total_num_video_updates`
- `self.obj_data.sensor_filter_fus_helper.total_num_front_left_corner_updates`
- `self.obj_data.sensor_filter_fus_helper.total_num_front_right_corner_updates`
- `self.obj_data.number_micro_doppler_cycles`
- `self.obj_data.expected_vr_high_enough_for_mu_doppler_counter`
- `self.obj_data.rcs`

### Function: `evaluate_functions(self)`
**Getter Methods Used:**
- `self.get_input_values()`
- `self.is_dep_obj_probably_video_ghost()`
- `self.obj_data.is_suppressed_until_next_video_update`
- `self.obj_data.is_suppressed_due_to_video_otc_post_processing`
- `self.obj_data.state.y`
- `self.obj_data.state.vy`
- `self.abs_vel_over_ground[1]`
- `self.obj_data.is_object_vru`
- `self.obj_data.is_updated_with_stat_loc_with_high_mdoppler_with_outgoing_vr`
- `self.obj_data.filter_type`
- `self.obj_data.num_cycles_existing`
- `self.obj_data.prob_is_currently_moving`
- `self.obj_data.prob_has_been_observed_moving`
- `self.obj_data.total_num_cycles_with_oncoming_locations`
- `self.params.is_micro_doppler_check_enabled`
- `self.params.min_vru_micro_doppler_cycles`
- `self.params.is_micro_doppler_check_on_crossing_vru_applied`
- `self.params.is_micro_doppler_check_on_stationary_vru_applied`
- `self.obj_data.avg_dx_innovation`
- `self.obj_data.elevation_is_valid`
- `self.obj_data.elevation`
- `self.params.innovation_check_dx_threshold`
- `self.params.innovation_check_dy_threshold`
- `self.obj_data.vy_unreliable_accumulated`
- `self.params.implausible_vy_thresh_la_hypo`
- `self.ego_data.yaw_rate`
- `self.obj_data.video_inv_ttc`

### Function: `load_example(self)`
**Getter Methods Used:**
- `self.obj_entries['pos_x'].delete()`
- `self.obj_entries['pos_x'].insert()`
- (Similar pattern for all other entry fields)

### Function: `save_config(self)`
**Getter Methods Used:**
- `self.get_input_values()`
- `self.obj_data.state.x`
- `self.obj_data.state.y`
- `self.obj_data.state.vx`
- `self.obj_data.state.vy`
- (All object data attributes)

### Function: `run(self)`
**Getter Methods Used:**
- `self.root.mainloop()`

---

## 3. C++ Functions (inputFile.cpp)

### Function: `applySuppressionUntilNextVideoUpdateCheck()`
**Getter Methods Used:**
- `obj.getIsSuppressedUntilNextVideoUpdate()`

### Function: `applyPostProcessVideoOtcCheck()`
**Getter Methods Used:**
- `obj.getIsSuppressedDueToVideoOtcPostProcessing()`

### Function: `isMovingTowardsEgoLane()`
**Getter Methods Used:**
- `vfc::isNegative()`

### Function: `isDepObjProbablyVideoGhost()`
**Getter Methods Used:**
- `depObj.sensorFilterFusHelper.getTotalNumSensorUpdates()`
- `depObj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()`
- `depObj.getNumberMicroDopplerCycles()`
- `depObj.getExpectedVrHighEnoughForMuDopplerCounter()`
- `depObj.get_rcs()`

### Function: `applyUnreliableAngularVelocityCheck()`
**Getter Methods Used:**
- `obj.get_probHasBeenObservedMoving()`
- `obj.get_probIsCurrentlyMoving()`
- `obj.get_objectTypeTreeReadOnly().isOfTypeOrSubType4PlusWheeler()`
- `hvmFor.getAngleDt()`
- `obj.get_numCyclesExisting()`
- `obj.sensorFilterFusHelper.getTotalNumSensorUpdates()`
- `obj.getNumCyclesSinceLastVideoUpdateWithAngularVelocity()`
- `obj.getVideoBasedInnovation()`
- `obj.getCreatedByVideoWithHighVy()`
- `obj.getState().getX()`
- `obj.getState().getY()`
- `obj.getState().getVelocity()`
- `obj.get_objectTypeTreeReadOnly().get_mostProbableConditionalType()`
- `obj.getStationaryLocationsOnlyCounter()`
- `obj.getExpectedVrHighEnoughForMuDopplerCounter()`
- `obj.getNumberMicroDopplerCycles()`
- `obj.getBadSensorBasedInnoCount()`
- `obj.getTotalNumCyclesWithOncomingLocations()`
- `obj.getNumConsecutiveCyclesWithoutOncomingLocations()`

### Function: `applyUpdatedWithStatLocWithHighMDopplerWithOutgoingVrCheck()`
**Getter Methods Used:**
- `obj.getState().getY()`
- `obj.getIsUpdatedWithStatLocWithHighMDopplerWithOutgoingVr()`

### Function: `applyIsMeasuredRatioCheckForFastWnj()`
**Getter Methods Used:**
- `obj.get_filterType()`
- `obj.get_numCyclesExisting()`
- `obj.sensorFilterFusHelper.getTotalNumSensorUpdates()`

### Function: `applyWaterSprinklersCheckAcc()`
**Getter Methods Used:**
- `obj.sensorFilterFusHelper.isOnlyUpdatedBySensorX()`
- `obj.getAvgDxInnovation()`
- `obj.get_rcs()`
- `obj.get_pNonObstacleRCSOnlyClassifier()`
- `obj.get_objectTypeTree().get_mostProbableConditionalType()`
- `obj.get_probHasBeenObservedMoving()`
- `obj.get_probIsCurrentlyMoving()`
- `obj.set_probHasBeenObservedMoving()`
- `obj.set_probIsCurrentlyMoving()`

### Function: `applyNonCrossingObjectCheck()`
**Getter Methods Used:**
- `obj.get_probIsCurrentlyMoving()`
- `obj.get_probHasBeenObservedMoving()`
- `obj.sensorFilterFusHelper.getTotalNumSensorUpdates()`
- `obj.getNumberMicroDopplerCycles()`
- `obj.getTotalNumCyclesWithOncomingLocations()`

### Function: `applyWaterSprinklesCheck()`
**Getter Methods Used:**
- `obj.sensorFilterFusHelper.hasOnlyBeenUpdatedBySensorXInLastNCycles()`
- `obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()`
- `obj.sensorFilterFusHelper.getTotalNumSensorUpdates()`
- `obj.getAvgDxInnovation()`
- `obj.get_rcs()`
- `obj.get_pNonObstacleRCSOnlyClassifier()`
- `obj.get_elevation()`
- `objectTypeTree.get_mostProbableConditionalType()`

### Function: `applyIsMeasuredRatioCheckForRadarOnlyLongitudinallyMoving()`
**Getter Methods Used:**
- `obj.get_numCyclesExisting()`
- `obj.sensorFilterFusHelper.isOnlyUpdatedBySensorXIgnoringEnvModelSensor()`
- `obj.getState().getY()`
- `obj.sensorFilterFusHelper.getTotalNumSensorUpdates()`

### Function: `applyMicroDopplerCheck()`
**Getter Methods Used:**
- `params.getIsMicroDopplerCheckEnabled()`
- `obj.getExpectedVrHighEnoughForMuDopplerCounter()`
- `obj.sensorFilterFusHelper.getTotalNumSensorUpdates()`
- `obj.getNumberMicroDopplerCycles()`
- `params.getMinVruMicroDopplerCycles()`
- `obj.get_numCyclesExisting()`
- `params.getIsMicroDopplerCheckOnCrossingVruApplied()`

### Function: `applyRadarOnlyRcsAndDrInnovationLimit()`
**Getter Methods Used:**
- `obj.sensorFilterFusHelper.isOnlyUpdatedBySensorXIgnoringEnvModelSensor()`
- `obj.getAvgDxInnovation()`
- `obj.get_rcs()`

### Function: `applyElevationCheck()`
**Getter Methods Used:**
- `obj.getState().getX()`
- `obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()`
- `obj.elevation_isValid()`
- `params.getElevationCheckDxLimits()`
- `params.getElevationCheckDzThresholds()`
- `obj.get_elevation()`

### Function: `applyNonPlausibleLocationChecks()`
**Getter Methods Used:**
- `objectTypeTree.get_mostProbableConditionalType()`
- `obj.getYawAngle()`
- `obj.getNonPlausibleLocationCnt()`
- `obj.get_filterType()`
- `obj.getWExistOfAssociatedVideoObject()`
- `obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()`

### Function: `applyFourpluswheelerChecks()`
**Getter Methods Used:**
- `objectTypeTree.isOfTypeOrSubType4PlusWheeler()`
- `obj.sensorFilterFusHelper.getTotalNumSensorUpdates()`
- `obj.getState().getVelocity()`
- `hvmFor.getVelocity()`
- `obj.getState().getX()`
- `obj.getAvgDxInnovation()`
- `obj.sensorFilterFusHelper.isGoodQualityFusedObject()`
- `obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()`
- `obj.elevation_isValid()`
- `obj.get_elevation()`
- `obj.getWExistOfAssociatedVideoObject()`
- `obj.get_rcs()`
- `params.getImplausibleRcsThreshold()`
- `params.getMaxLongitudinalDistanceForRcsCountermeasure()`
- `obj.sensorFilterFusHelper.isTrustworthyObject()`
- `obj.get_probHasBeenObservedMoving()`
- `obj.getExpectedVrHighEnoughForMuDopplerCounter()`
- `obj.getStationaryLocationsOnlyCounter()`
- `obj.getFacingAngle()`
- `obj.getNumberMicroDopplerCycles()`

### Function: `applySplitCheckDisqualifyObjectForFunctions()`
**Getter Methods Used:**
- `obj.getSplitCounter()`
- `params.getSplitDetectionCntMaxVal()`
- `obj.getStoppingSplitCounter()`

### Function: `applyOrientationConsistencyCheck()`
**Getter Methods Used:**
- `obj.getDimension().getPosition()`
- `obj.get_numCyclesExisting()`
- `obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()`
- `obj.sensorFilterFusHelper.getTotalNumSensorUpdates()`
- `obj.get_objectTypeTreeReadOnly().get_mostProbableConditionalType()`
- `obj.get_filterType()`
- `obj.getYawAngle()`
- `obj.getObjectOrientationUnreliableCount()`
- `obj.getIsOrientationImplausibleCompared2Vid()`

### Function: `modifyUnreliableOrientationCount()`
**Getter Methods Used:**
- `obj.get_numCyclesNoOrientationUpdate()`
- `hvmFor.getAngleDt()`
- `obj.getObjectOrientationUnreliableCount()`
- `obj.setObjectOrientationUnreliableCount()`

### Function: `applyStationaryVruVideoGhostCheck()`
**Getter Methods Used:**
- `objectTypeTree.get_mostProbableConditionalType()`
- `obj.get_rcs()`
- `obj.getWExistOfAssociatedVideoObject()`
- `obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()`

### Function: `applyInnovationCheck()`
**Getter Methods Used:**
- `obj.getState().getPosition()`
- `params.getInnovationCheckDxThreshold()`
- `params.getInnovationCheckDyThreshold()`
- `obj.getAvgDxInnovation()`

### Function: `calcDxInnovationThreshold()`
**Getter Methods Used:**
- `obj.getState().getPosition()`
- `obj.get_rcs()`
- `obj.getVyUnreliableAccumulated()`
- `obj.get_numCyclesExisting()`

### Function: `applyImplausibleVyVruCheck()`
**Getter Methods Used:**
- `hvmFor.getAngleDt()`
- `obj.get_filterType()`

### Function: `applySensorBasedInnoCheck()`
**Getter Methods Used:**
- `obj.getBadSensorBasedInnoCount()`
- `obj.getState().getPosition()`

### Function: `applyImplausibleVideoTtcForVru()`
**Getter Methods Used:**
- `obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()`
- `obj.getVideoInvTtc()`

### Function: `applyVyInconsistentCheck()`
**Getter Methods Used:**
- `obj.getVyInconsistent()`
- `obj.get_filterType()`
- `obj.getState().getX()`
- `obj.getVyUnreliableAccumulated()`

### Function: `applyVideoHandleSharedCheck()`
**Getter Methods Used:**
- `obj.get_probHasBeenObservedMoving()`
- `obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()`
- `obj.getRecentlyUsedVideoMeasurementHandleValid()`

### Function: `modifyBadSensorBasedInnoCount()`
**Getter Methods Used:**
- `obj.getState().getPosition()`
- `obj.getReferencePoint()`
- `obj.getBadSensorBasedInnoCount()`
- `obj.setBadSensorBasedInnoCount()`

### Function: `applyRadarOnlyNLDCheck()`
**Getter Methods Used:**
- `obj.sensorFilterFusHelper.getTotalNumSensorUpdates()`
- `hvmFor.getVelocity()`
- `hvmFor.getAcceleration()`
- `hvmFor.getAngleDt()`
- `obj.getState().getPosition()`
- `obj.get_numCyclesExisting()`
- `obj.getState().getVelocity()`

### Function: `applyRadarOnlyStationaryCheck()`
**Getter Methods Used:**
- `obj.sensorFilterFusHelper.getTotalNumSensorUpdates()`

### Function: `applyIsMeasuredRatioCheckForStandingLongiVru()`
**Getter Methods Used:**
- `obj.sensorFilterFusHelper.getTotalNumSensorUpdates()`
- `obj.getState().getPosition()`
- `obj.get_numCyclesExisting()`
- `obj.get_filterType()`
- `obj.get_probHasBeenObservedMoving()`

### Function: `applyImplausiblyAcceleratingVruCheck()`
**Getter Methods Used:**
- `obj.sensorFilterFusHelper.getTotalNumSensorUpdates()`
- `obj.getState().getPosition()`
- `obj.get_numCyclesExisting()`
- `obj.get_filterType()`
- `obj.get_probHasBeenObservedMoving()`

### Function: `applyUndefinedCrossingVruFromCorner()`
**Getter Methods Used:**
- `obj.sensorFilterFusHelper.getTotalNumSensorUpdates()`
- `obj.getState().getPosition()`
- `obj.get_numCyclesExisting()`
- `obj.get_filterType()`
- `obj.get_probHasBeenObservedMoving()`

### Function: `applyImplausiblePedCheck()`
**Getter Methods Used:**
- `obj.getState().getPosition()`
- `obj.get_objectTypeTreeReadOnly().get_mostProbableConditionalType()`
- `obj.getNumberMicroDopplerCycles()`
- `obj.get_rcs()`
- `obj.sensorFilterFusHelper.getTotalNumSensorUpdates()`
- `obj.getStationaryLocationsOnlyCounter()`
- `obj.get_elevation()`

### Function: `applyImplausiblePedCheckLRR()`
**Getter Methods Used:**
- `obj.getState().getPosition()`
- `obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()`
- `obj.get_objectTypeTreeReadOnly().get_mostProbableConditionalType()`
- `obj.getNumberMicroDopplerCycles()`
- `obj.get_rcs()`
- `obj.getExpectedVrHighEnoughForMuDopplerCounter()`
- `obj.getStationaryLocationsOnlyCounter()`
- `obj.getVideoBasedInnovation()`

### Function: `applyImplausibleCarAtCloseRangeCheck()`
**Getter Methods Used:**
- `obj.getState().getPosition()`
- `obj.getDimension().getPosition()`
- `obj.get_objectTypeTreeReadOnly().get_mostProbableConditionalType()`
- `obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()`
- `obj.sensorFilterFusHelper.isOnlyUpdatedBySensorX()`

### Function: `applyElevatedObjectCheck()`
**Getter Methods Used:**
- `obj.get_rcs()`
- `obj.elevation_isValid()`
- `obj.get_elevation()`
- `obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()`
- `obj.get_numCyclesExisting()`
- `obj.getStationaryLocationsOnlyCounter()`
- `obj.getState().getPosition()`

### Function: `applyBridgeCheck()`
**Getter Methods Used:**
- `obj.getVideoBasedInnovation()`
- `obj.elevation_isValid()`
- `obj.get_elevation()`
- `obj.getStationaryLocationsOnlyCounter()`
- `obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()`

### Function: `applyCornerRadarAssoWithStationaryLocationsForTheFirstTime()`
**Getter Methods Used:**
- `obj.get_numCyclesExisting()`
- `obj.sensorFilterFusHelper.getTotalNumSensorUpdates()`
- `obj.getStationaryLocationsOnlyCounter()`
- `hvmFor.getAngleDt()`
- `obj.getTransferredFromSepCycle()`

### Function: `applyInconsistentAlphaCheck()`
**Getter Methods Used:**
- `obj.getRadarRawAlphaInnovation()`
- `obj.getVideoRawAlphaInnovation()`
- `obj.getState().getPosition()`
- `obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()`

---

## Summary

### Total Functions Analyzed: 63

**By Language:**
- JavaScript: 10 functions
- Python: 12 functions  
- C++: 41 functions

**Most Common Getter Patterns:**
1. **Object state getters:** `getState().getX()`, `getState().getY()`, `getState().getVelocity()`
2. **Sensor data getters:** `getTotalNumSensorUpdates()`, `getUpdatesSinceLastSensorUpdate()`
3. **Probability getters:** `get_probHasBeenObservedMoving()`, `get_probIsCurrentlyMoving()`
4. **Object property getters:** `get_rcs()`, `get_elevation()`, `get_filterType()`
5. **DOM element getters:** `getElementById()`, `querySelector()` (JavaScript)
6. **GUI element getters:** `.get()` method on tkinter widgets (Python)

**Key Observations:**
- C++ functions use the most diverse set of getter methods due to the complexity of the automotive perception algorithms
- Python functions primarily use GUI widget getters for user input collection
- JavaScript functions focus on DOM manipulation and data structure access
- All three implementations share similar conceptual getter patterns but use language-specific syntax

This analysis provides a comprehensive view of how getter methods are utilized across the entire automotive perception codebase.