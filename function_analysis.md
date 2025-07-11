# C++ Automotive Perception Functions Analysis

## Overview
This document analyzes the automotive perception post-processing functions from `inputFile.cpp`. Each function implements specific checks for object validation and safety system (AEB/ACC) qualification.

## Function Analysis

### 1. applySuppressionUntilNextVideoUpdateCheck
- **Input Parameters:** obj, functionRelevanceBitField
- **Object Attributes Used:** obj.getIsSuppressedUntilNextVideoUpdate()
- **Condition:** obj.getIsSuppressedUntilNextVideoUpdate() == true
- **Action:** Disqualifies for AEB

### 2. applyPostProcessVideoOtcCheck
- **Input Parameters:** obj, functionRelevanceBitField
- **Object Attributes Used:** obj.getIsSuppressedDueToVideoOtcPostProcessing()
- **Condition:** obj.getIsSuppressedDueToVideoOtcPostProcessing() == true
- **Action:** Disqualifies for AEB

### 3. isMovingTowardsEgoLane
- **Input Parameters:** dyObj, vyObjRel, vyObjOverGround
- **Local Variables:** dyObj, vyObjRel, vyObjOverGround
- **Condition:** vfc::isNegative(dyObj * vyObjRel) || vfc::isNegative(dyObj * vyObjOverGround)
- **Returns:** Boolean indicating movement towards ego lane

### 4. isDepObjProbablyVideoGhost
- **Input Parameters:** depObj
- **Object Attributes Used:** 
  - depObj.sensorFilterFusHelper.getTotalNumSensorUpdates()
  - depObj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()
  - depObj.getNumberMicroDopplerCycles()
  - depObj.getExpectedVrHighEnoughForMuDopplerCounter()
  - depObj.get_rcs()
- **Condition:** Complex logic combining radar updates, video tracking, micro doppler, and RCS
- **Returns:** Boolean indicating if object is probably a video ghost

### 5. applyUnreliableAngularVelocityCheck
- **Input Parameters:** obj, hvmFor, absVelOverGround, isObjectVru, isDepObjProbablyVideoGhost, functionRelevanceBitField
- **Object Attributes Used:** 
  - obj.get_probHasBeenObservedMoving()
  - obj.get_probIsCurrentlyMoving()
  - obj.get_objectTypeTreeReadOnly()
  - obj.get_numCyclesExisting()
  - obj.getVideoBasedInnovation()
  - obj.getState()
  - obj.getSplitCounter()
  - etc.
- **Conditions:** Multiple complex conditions for unreliable angular velocity detection
- **Action:** Disqualifies for AEB or VY-dependent functions

### 6. applyUpdatedWithStatLocWithHighMDopplerWithOutgoingVrCheck
- **Input Parameters:** obj, absVelOverGround, isObjectVru, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.getState().getY()
  - obj.getIsUpdatedWithStatLocWithHighMDopplerWithOutgoingVr()
- **Condition:** isObjectVru && absVelOverGround[0] < 0.2F && absVelOverGround[1] > 1.F && vfc::abs(obj.getState().getY()) < 0.5F && obj.getIsUpdatedWithStatLocWithHighMDopplerWithOutgoingVr()
- **Action:** Disqualifies for AEB

### 7. applyIsMeasuredRatioCheckForFastWnj
- **Input Parameters:** absVelOverGround, obj, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.get_filterType()
  - obj.get_numCyclesExisting()
  - obj.sensorFilterFusHelper.getTotalNumSensorUpdates()
- **Condition:** Fast crossing WNJ objects with insufficient measurement ratio
- **Action:** Disqualifies for AEB

### 8. applyWaterSprinklersCheckAcc
- **Input Parameters:** absVelOverGround, obj
- **Object Attributes Used:**
  - obj.sensorFilterFusHelper.isOnlyUpdatedBySensorX()
  - obj.getAvgDxInnovation()
  - obj.get_rcs()
  - obj.get_pNonObstacleRCSOnlyClassifier()
  - obj.get_objectTypeTree()
- **Condition:** Radar-only objects with water sprinkler characteristics
- **Action:** Modifies object probabilities

### 9. applyNonCrossingObjectCheck
- **Input Parameters:** obj, absVelOverGround, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.get_probIsCurrentlyMoving()
  - obj.get_probHasBeenObservedMoving()
  - obj.sensorFilterFusHelper.getTotalNumSensorUpdates()
  - obj.getNumberMicroDopplerCycles()
  - obj.getTotalNumCyclesWithOncomingLocations()
- **Condition:** Objects appearing to cross but probably not moving
- **Action:** Disqualifies for VY-dependent functions

### 10. applyWaterSprinklesCheck
- **Input Parameters:** obj, absVelOverGround, objectTypeTree, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.sensorFilterFusHelper.hasOnlyBeenUpdatedBySensorXInLastNCycles()
  - obj.getAvgDxInnovation()
  - obj.get_rcs()
  - obj.get_elevation()
  - obj.get_pNonObstacleRCSOnlyClassifier()
- **Condition:** Objects with water sprinkler/ground reflection characteristics
- **Action:** Disqualifies for AEB

### 11. applyIsMeasuredRatioCheckForRadarOnlyLongitudinallyMoving
- **Input Parameters:** absVelOverGround, obj, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.get_numCyclesExisting()
  - obj.sensorFilterFusHelper.isOnlyUpdatedBySensorXIgnoringEnvModelSensor()
  - obj.getState().getY()
  - obj.sensorFilterFusHelper.getTotalNumSensorUpdates()
- **Condition:** Radar-only longitudinally moving objects with low measurement ratio
- **Action:** Disqualifies for AEB

### 12. applyMicroDopplerCheck
- **Input Parameters:** isObjectVru, absVelOverGround, obj, params, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.getExpectedVrHighEnoughForMuDopplerCounter()
  - obj.sensorFilterFusHelper.getTotalNumSensorUpdates()
  - obj.getNumberMicroDopplerCycles()
  - obj.get_numCyclesExisting()
- **Condition:** VRU objects missing expected micro-doppler signatures
- **Action:** Disqualifies for AEB

### 13. applyRadarOnlyRcsAndDrInnovationLimit
- **Input Parameters:** obj, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.sensorFilterFusHelper.isOnlyUpdatedBySensorXIgnoringEnvModelSensor()
  - obj.getAvgDxInnovation()
  - obj.get_rcs()
- **Condition:** Radar-only objects with high innovation and low RCS
- **Action:** Disqualifies for AEB

### 14. applyElevationCheck
- **Input Parameters:** obj, absVelOverGround, isObjectVru, params, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.getState().getX()
  - obj.elevation_isValid()
  - obj.get_elevation()
  - obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()
- **Condition:** Objects with inappropriate elevation (too high)
- **Action:** Disqualifies for AEB

### 15. applyNonPlausibleLocationChecks
- **Input Parameters:** obj, objectTypeTree, isMPC3Used, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.getYawAngle()
  - obj.getNonPlausibleLocationCnt()
  - obj.get_filterType()
  - obj.getWExistOfAssociatedVideoObject()
  - obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()
- **Condition:** Trucks with non-plausible locations
- **Action:** Disqualifies for AEB

### 16. applyFourpluswheelerChecks
- **Input Parameters:** obj, hvmFor, params, objectTypeTree, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.sensorFilterFusHelper.getTotalNumSensorUpdates()
  - obj.getState().getVelocity()
  - obj.getState().getX()
  - obj.getAvgDxInnovation()
  - obj.get_rcs()
  - obj.elevation_isValid()
  - obj.get_elevation()
  - obj.getWExistOfAssociatedVideoObject()
  - obj.get_probHasBeenObservedMoving()
  - obj.sensorFilterFusHelper.isTrustworthyObject()
  - obj.getExpectedVrHighEnoughForMuDopplerCounter()
  - obj.getStationaryLocationsOnlyCounter()
  - obj.getFacingAngle()
  - obj.getNumberMicroDopplerCycles()
- **Condition:** Various implausible characteristics for 4+ wheeler vehicles
- **Action:** Disqualifies for AEB and/or ACC

### 17. applySplitCheckDisqualifyObjectForFunctions
- **Input Parameters:** isObjectVru, obj, params, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.getSplitCounter()
  - obj.getStoppingSplitCounter()
- **Condition:** Non-VRU objects with split detection
- **Action:** Disqualifies for AEB and ACC

### 18. applyOrientationConsistencyCheck
- **Input Parameters:** absVelOverGround, hvmFor, obj, objectTypeTree, isObjectVru, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.getDimension()
  - obj.get_numCyclesExisting()
  - obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()
  - obj.sensorFilterFusHelper.getTotalNumSensorUpdates()
  - obj.get_objectTypeTreeReadOnly()
  - obj.get_filterType()
  - obj.getYawAngle()
  - obj.getState()
  - obj.getObjectOrientationUnreliableCount()
  - obj.getIsOrientationImplausibleCompared2Vid()
- **Condition:** Objects with inconsistent orientation vs velocity direction
- **Action:** Disqualifies for AEB

### 19. modifyUnreliableOrientationCount
- **Input Parameters:** hvmFor, isObjectVru, obj
- **Object Attributes Used:**
  - obj.get_numCyclesNoOrientationUpdate()
  - obj.getObjectOrientationUnreliableCount()
- **Condition:** Non-VRU objects during high ego yaw rate without orientation updates
- **Action:** Modifies orientation unreliable count

### 20. applyStationaryVruVideoGhostCheck
- **Input Parameters:** obj, absVelOverGround, objectTypeTree, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.get_rcs()
  - obj.getWExistOfAssociatedVideoObject()
  - obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()
- **Condition:** Stationary VRU with high RCS and low video existence probability
- **Action:** Disqualifies for AEB

### 21. applyInnovationCheck
- **Input Parameters:** hvmFor, obj, isVruObject, params, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.getState().getPosition()
  - obj.getAvgDxInnovation()
- **Condition:** Objects with high dx innovation in relevant range
- **Action:** Disqualifies for AEB

### 22. calcDxInnovationThreshold
- **Input Parameters:** hvmFor, obj, isVruObject
- **Object Attributes Used:**
  - obj.getState().getPosition()
  - obj.get_rcs()
  - obj.getVyUnreliableAccumulated()
  - obj.get_numCyclesExisting()
  - obj.sensorFilterFusHelper.isGoodQualityFusedObject()
  - obj.sensorFilterFusHelper.getTotalNumSensorUpdates()
- **Returns:** Float32 threshold value for dx innovation

### 23. applyImplausibleVyVruCheck
- **Input Parameters:** isObjectVru, absVelOverGround, hvmFor, obj, params, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.get_filterType()
- **Condition:** VRU with LA filter type and high VY velocity when ego not turning
- **Action:** Disqualifies for AEB and ACC

### 24. applySensorBasedInnoCheck
- **Input Parameters:** obj, isObjectVru, absVelOverGround, params, objectTypeTree, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.getBadSensorBasedInnoCount()
  - obj.getState().getPosition()
- **Condition:** Objects with high bad sensor innovation count
- **Action:** Disqualifies for AEB

### 25. applyImplausibleVideoTtcForVru
- **Input Parameters:** isVru, obj, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()
  - obj.getVideoInvTtc()
- **Condition:** VRU with implausible video TTC
- **Action:** Disqualifies for AEB

### 26. applyVyInconsistentCheck
- **Input Parameters:** obj, hvmFor, isVru, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.getVyInconsistent()
  - obj.get_filterType()
  - obj.getState().getPosition()
  - obj.getVyUnreliableAccumulated()
- **Condition:** VRU with inconsistent VY measurements
- **Action:** Disqualifies for AEB

### 27. applyVideoHandleSharedCheck
- **Input Parameters:** hvmFor, isObjectVru, absVelOverGround, obj, depCollection, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.get_probHasBeenObservedMoving()
  - obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()
  - obj.isRecentlyUsedVideoMeasurementHandleValid()
  - obj.getRecentlyUsedVideoMeasurementHandle()
  - obj.get10bitObjectId()
- **Condition:** Stationary VRU sharing video handle with moving object
- **Action:** Disqualifies for AEB

### 28. modifyBadSensorBasedInnoCount
- **Input Parameters:** obj, absVelOverGround, isMPC3Used
- **Object Attributes Used:**
  - obj.getReferencePointPosition()
  - obj.getRadarBasedInnovation()
  - obj.getVideoBasedInnovation()
  - obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()
- **Action:** Updates bad sensor innovation counter

### 29. applyRadarOnlyNLDCheck
- **Input Parameters:** hvmFor, obj, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.sensorFilterFusHelper.getTotalNumSensorUpdates()
  - obj.getState().getPosition()
  - obj.getState().getVelocity()
  - obj.get_numCyclesExisting()
- **Condition:** Radar-only objects that are NLD candidates
- **Action:** Disqualifies for AEB and ACC

### 30. applyRadarOnlyStationaryCheck
- **Input Parameters:** absVelOverGround, obj, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.sensorFilterFusHelper.getTotalNumSensorUpdates()
- **Condition:** Radar-only stationary objects
- **Action:** Disqualifies for AEB and ACC

### 31. applyIsMeasuredRatioCheckForStandingLongiVru
- **Input Parameters:** obj, isObjectVru, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.get_filterType()
  - obj.get_probHasBeenObservedMoving()
  - obj.get_probIsCurrentlyMoving()
  - obj.get_numCyclesExisting()
  - obj.m_sensorMeasuredHistory.numberOfSingleUpdateBySensorXOrNonUpdate()
- **Condition:** Standing longitudinal VRU with poor measurement history
- **Action:** Disqualifies for AEB

### 32. applyImplausiblyAcceleratingVruCheck
- **Input Parameters:** obj, isObjectVru, hvmFor, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.getState().getX()
  - obj.get_numCyclesExisting()
  - obj.getCurrentStateInAbsoluteCoordinates()
- **Condition:** Close, young VRU with implausibly high forward acceleration
- **Action:** Disqualifies for AEB

### 33. applyUndefinedCrossingVruFromCorner
- **Input Parameters:** obj, absVelOverGround, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.sensorFilterFusHelper.getTotalNumSensorUpdates()
  - obj.getState().getPosition()
  - obj.get_numCyclesExisting()
  - obj.get_filterType()
  - obj.get_probHasBeenObservedMoving()
  - obj.get_objectTypeTreeReadOnly()
- **Condition:** Corner-detected VRU with undefined crossing behavior
- **Action:** Disqualifies for AEB

### 34. applyImplausiblePedCheck
- **Input Parameters:** obj, absVelOverGround, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.getState().getY()
  - obj.get_objectTypeTreeReadOnly()
  - obj.getNumberMicroDopplerCycles()
  - obj.get_rcs()
  - obj.sensorFilterFusHelper.getTotalNumSensorUpdates()
  - obj.getStationaryLocationsOnlyCounter()
  - obj.get_numCyclesExisting()
  - obj.getRadarBasedInnovation()
  - obj.getVideoBasedInnovation()
  - obj.elevation_isValid()
  - obj.get_elevation()
- **Condition:** Pedestrians with implausible characteristics
- **Action:** Disqualifies for AEB

### 35. applyImplausiblePedCheckLRR
- **Input Parameters:** obj, absVelOverGround, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.getState().getY()
  - obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()
  - obj.get_objectTypeTreeReadOnly()
  - obj.getNumberMicroDopplerCycles()
  - obj.get_rcs()
  - obj.sensorFilterFusHelper.getTotalNumSensorUpdates()
  - obj.getExpectedVrHighEnoughForMuDopplerCounter()
  - obj.getStationaryLocationsOnlyCounter()
  - obj.getVideoBasedInnovation()
  - obj.get_numCyclesExisting()
- **Condition:** LRR-specific implausible pedestrian checks
- **Action:** Disqualifies for AEB

### 36. applyImplausibleCarAtCloseRangeCheck
- **Input Parameters:** obj, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.getState().getX()
  - obj.getDimension().getPosition()
  - obj.get_objectTypeTreeReadOnly()
  - obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()
- **Condition:** Long cars at close range updated only by front center radar
- **Action:** Disqualifies for AEB

### 37. applyElevatedObjectCheck
- **Input Parameters:** obj, absVelOverGround, isObjectVru, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.get_rcs()
  - obj.elevation_isValid()
  - obj.get_elevation()
  - obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()
  - obj.get_numCyclesExisting()
  - obj.getTransferredFromSepCycle()
  - obj.m_sensorMeasuredHistory.numberOfSingleUpdateBySensorXOrNonUpdate()
  - obj.get_objectTypeTree()
  - obj.getStationaryLocationsOnlyCounter()
  - obj.getVideoBasedInnovation()
  - obj.get_objectTypeTreeReadOnly()
  - obj.getState().getX()
  - obj.getRadarBasedInnovation()
  - obj.sensorFilterFusHelper.getTotalNumSensorUpdates()
- **Condition:** Elevated objects with unreliable characteristics
- **Action:** Disqualifies for AEB

### 38. applyBridgeCheck
- **Input Parameters:** obj, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.getVideoBasedInnovation()
  - obj.elevation_isValid()
  - obj.get_elevation()
  - obj.getStationaryLocationsOnlyCounter()
  - obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()
- **Condition:** Objects with bridge-like characteristics
- **Action:** Disqualifies for AEB

### 39. applyCornerRadarAssoWithStationaryLocationsForTheFirstTime
- **Input Parameters:** isObjectVru, hvmFor, obj, absVelOverGround, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.get_numCyclesExisting()
  - obj.sensorFilterFusHelper.getTotalNumSensorUpdates()
  - obj.getStationaryLocationsOnlyCounter()
- **Condition:** Young VRU first detected by corner radar with stationary locations
- **Action:** Disqualifies for AEB

### 40. applyInconsistentAlphaCheck
- **Input Parameters:** obj, functionRelevanceBitField
- **Object Attributes Used:**
  - obj.getRadarRawAlphaInnovation()
  - obj.getVideoRawAlphaInnovation()
  - obj.getState().getX()
  - obj.getState().getY()
  - obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()
- **Condition:** Objects with inconsistent radar and video alpha innovations
- **Action:** Disqualifies for AEB

## Complete Variable Dependencies

### Input Parameters
- obj (DepObject reference)
- functionRelevanceBitField
- absVelOverGround
- isObjectVru
- hvmFor
- params
- objectTypeTree
- depCollection
- isDepObjProbablyVideoGhost
- isMPC3Used
- dyObj
- vyObjRel
- vyObjOverGround

### Object Attributes (Superset)
- getIsSuppressedUntilNextVideoUpdate()
- getIsSuppressedDueToVideoOtcPostProcessing()
- sensorFilterFusHelper.getTotalNumSensorUpdates()
- sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate()
- getNumberMicroDopplerCycles()
- getExpectedVrHighEnoughForMuDopplerCounter()
- get_rcs()
- get_probHasBeenObservedMoving()
- get_probIsCurrentlyMoving()
- get_objectTypeTreeReadOnly()
- get_numCyclesExisting()
- getVideoBasedInnovation()
- getState()
- getSplitCounter()
- getStoppingSplitCounter()
- getIsUpdatedWithStatLocWithHighMDopplerWithOutgoingVr()
- get_filterType()
- getAvgDxInnovation()
- get_pNonObstacleRCSOnlyClassifier()
- get_objectTypeTree()
- getTotalNumCyclesWithOncomingLocations()
- get_elevation()
- elevation_isValid()
- getYawAngle()
- getNonPlausibleLocationCnt()
- getWExistOfAssociatedVideoObject()
- getDimension()
- get_probHasBeenObservedMoving()
- sensorFilterFusHelper.isTrustworthyObject()
- getFacingAngle()
- getObjectOrientationUnreliableCount()
- get_numCyclesNoOrientationUpdate()
- getIsOrientationImplausibleCompared2Vid()
- getBadSensorBasedInnoCount()
- getVideoInvTtc()
- getVyInconsistent()
- getVyUnreliableAccumulated()
- isRecentlyUsedVideoMeasurementHandleValid()
- getRecentlyUsedVideoMeasurementHandle()
- get10bitObjectId()
- getReferencePointPosition()
- getRadarBasedInnovation()
- m_sensorMeasuredHistory.numberOfSingleUpdateBySensorXOrNonUpdate()
- getCurrentStateInAbsoluteCoordinates()
- getStationaryLocationsOnlyCounter()
- getTransferredFromSepCycle()
- getRadarRawAlphaInnovation()
- getVideoRawAlphaInnovation()