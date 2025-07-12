// Automotive Perception Functions Data
const functions = [
    {
        id: 1,
        name: "applySuppressionUntilNextVideoUpdateCheck",
        description: "Checks if object is suppressed until next video update",
        inputs: [
            { name: "isSuppressedUntilNextVideoUpdate", type: "boolean", label: "Is Suppressed Until Next Video Update?" }
        ],
        condition: (inputs) => inputs.isSuppressedUntilNextVideoUpdate === true,
        action: "Disqualifies for AEB"
    },
    {
        id: 2,
        name: "applyPostProcessVideoOtcCheck",
        description: "Checks if object is suppressed due to video OTC post-processing",
        inputs: [
            { name: "isSuppressedDueToVideoOtc", type: "boolean", label: "Is Suppressed Due To Video OTC?" }
        ],
        condition: (inputs) => inputs.isSuppressedDueToVideoOtc === true,
        action: "Disqualifies for AEB"
    },
    {
        id: 3,
        name: "isMovingTowardsEgoLane",
        description: "Determines if object is moving towards ego lane",
        inputs: [
            { name: "dyObj", type: "number", label: "dyObj (lateral distance)" },
            { name: "vyObjRel", type: "number", label: "vyObjRel (relative lateral velocity)" },
            { name: "vyObjOverGround", type: "number", label: "vyObjOverGround (lateral velocity over ground)" }
        ],
        condition: (inputs) => {
            const { dyObj, vyObjRel, vyObjOverGround } = inputs;
            return (dyObj * vyObjRel < 0) || (dyObj * vyObjOverGround < 0);
        },
        action: "Returns boolean indicating movement towards ego lane"
    },
    {
        id: 4,
        name: "isDepObjProbablyVideoGhost",
        description: "Checks if dependent object is probably a video ghost",
        inputs: [
            { name: "totalNumSensorUpdates", type: "number", label: "Total Number of Sensor Updates" },
            { name: "updatesSinceLastSensor", type: "number", label: "Updates Since Last Sensor" },
            { name: "numMicroDopplerCycles", type: "number", label: "Number of Micro Doppler Cycles" },
            { name: "expectedVrHighEnough", type: "boolean", label: "Expected VR High Enough?" },
            { name: "rcs", type: "number", label: "RCS (Radar Cross Section)" }
        ],
        condition: (inputs) => {
            const { totalNumSensorUpdates, updatesSinceLastSensor, numMicroDopplerCycles, expectedVrHighEnough, rcs } = inputs;
            return totalNumSensorUpdates > 5 && updatesSinceLastSensor > 3 && numMicroDopplerCycles < 2 && !expectedVrHighEnough && rcs < -10;
        },
        action: "Returns boolean indicating if object is probably a video ghost"
    },
    {
        id: 5,
        name: "applyUnreliableAngularVelocityCheck",
        description: "Checks for unreliable angular velocity",
        inputs: [
            { name: "probHasBeenObservedMoving", type: "number", label: "Probability Has Been Observed Moving (0-1)" },
            { name: "probIsCurrentlyMoving", type: "number", label: "Probability Is Currently Moving (0-1)" },
            { name: "numCyclesExisting", type: "number", label: "Number of Cycles Existing" },
            { name: "absVelOverGround", type: "number", label: "Absolute Velocity Over Ground" },
            { name: "isObjectVru", type: "boolean", label: "Is Object VRU?" }
        ],
        condition: (inputs) => {
            const { probHasBeenObservedMoving, probIsCurrentlyMoving, numCyclesExisting, absVelOverGround, isObjectVru } = inputs;
            return probHasBeenObservedMoving < 0.3 && probIsCurrentlyMoving < 0.2 && numCyclesExisting > 10 && absVelOverGround > 2.0 && !isObjectVru;
        },
        action: "Disqualifies for AEB or VY-dependent functions"
    },
    {
        id: 6,
        name: "applyUpdatedWithStatLocWithHighMDopplerWithOutgoingVrCheck",
        description: "Checks VRU updated with stationary location and high micro doppler",
        inputs: [
            { name: "isObjectVru", type: "boolean", label: "Is Object VRU?" },
            { name: "absVelOverGroundX", type: "number", label: "Absolute Velocity Over Ground X" },
            { name: "absVelOverGroundY", type: "number", label: "Absolute Velocity Over Ground Y" },
            { name: "stateY", type: "number", label: "State Y Position" },
            { name: "isUpdatedWithStatLocWithHighMD", type: "boolean", label: "Is Updated With Stat Loc With High MD?" }
        ],
        condition: (inputs) => {
            const { isObjectVru, absVelOverGroundX, absVelOverGroundY, stateY, isUpdatedWithStatLocWithHighMD } = inputs;
            return isObjectVru && absVelOverGroundX < 0.2 && absVelOverGroundY > 1.0 && Math.abs(stateY) < 0.5 && isUpdatedWithStatLocWithHighMD;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 7,
        name: "applyIsMeasuredRatioCheckForFastWnj",
        description: "Checks measurement ratio for fast crossing WNJ objects",
        inputs: [
            { name: "absVelOverGround", type: "number", label: "Absolute Velocity Over Ground" },
            { name: "filterType", type: "select", label: "Filter Type", options: ["CA", "CV", "LA"] },
            { name: "numCyclesExisting", type: "number", label: "Number of Cycles Existing" },
            { name: "totalNumSensorUpdates", type: "number", label: "Total Number of Sensor Updates" }
        ],
        condition: (inputs) => {
            const { absVelOverGround, filterType, numCyclesExisting, totalNumSensorUpdates } = inputs;
            const measurementRatio = totalNumSensorUpdates / numCyclesExisting;
            return absVelOverGround > 5.0 && filterType === "CA" && measurementRatio < 0.3;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 8,
        name: "applyWaterSprinklersCheckAcc",
        description: "Checks for water sprinkler characteristics for ACC",
        inputs: [
            { name: "isOnlyUpdatedBySensorX", type: "boolean", label: "Is Only Updated By Sensor X?" },
            { name: "avgDxInnovation", type: "number", label: "Average Dx Innovation" },
            { name: "rcs", type: "number", label: "RCS (Radar Cross Section)" },
            { name: "pNonObstacleRCS", type: "number", label: "P Non-Obstacle RCS (0-1)" }
        ],
        condition: (inputs) => {
            const { isOnlyUpdatedBySensorX, avgDxInnovation, rcs, pNonObstacleRCS } = inputs;
            return isOnlyUpdatedBySensorX && avgDxInnovation > 2.0 && rcs < -15 && pNonObstacleRCS > 0.7;
        },
        action: "Modifies object probabilities"
    },
    {
        id: 9,
        name: "applyNonCrossingObjectCheck",
        description: "Checks for objects appearing to cross but probably not moving",
        inputs: [
            { name: "probIsCurrentlyMoving", type: "number", label: "Probability Is Currently Moving (0-1)" },
            { name: "probHasBeenObservedMoving", type: "number", label: "Probability Has Been Observed Moving (0-1)" },
            { name: "totalNumSensorUpdates", type: "number", label: "Total Number of Sensor Updates" },
            { name: "numMicroDopplerCycles", type: "number", label: "Number of Micro Doppler Cycles" },
            { name: "totalNumCyclesWithOncomingLocations", type: "number", label: "Total Cycles With Oncoming Locations" }
        ],
        condition: (inputs) => {
            const { probIsCurrentlyMoving, probHasBeenObservedMoving, totalNumSensorUpdates, numMicroDopplerCycles, totalNumCyclesWithOncomingLocations } = inputs;
            return probIsCurrentlyMoving < 0.2 && probHasBeenObservedMoving < 0.3 && totalNumSensorUpdates > 15 && numMicroDopplerCycles < 3 && totalNumCyclesWithOncomingLocations > 10;
        },
        action: "Disqualifies for VY-dependent functions"
    },
    {
        id: 10,
        name: "applyWaterSprinklesCheck",
        description: "Checks for water sprinkler/ground reflection characteristics",
        inputs: [
            { name: "hasOnlyBeenUpdatedBySensorX", type: "boolean", label: "Has Only Been Updated By Sensor X?" },
            { name: "avgDxInnovation", type: "number", label: "Average Dx Innovation" },
            { name: "rcs", type: "number", label: "RCS (Radar Cross Section)" },
            { name: "elevation", type: "number", label: "Elevation (degrees)" },
            { name: "pNonObstacleRCS", type: "number", label: "P Non-Obstacle RCS (0-1)" }
        ],
        condition: (inputs) => {
            const { hasOnlyBeenUpdatedBySensorX, avgDxInnovation, rcs, elevation, pNonObstacleRCS } = inputs;
            return hasOnlyBeenUpdatedBySensorX && avgDxInnovation > 1.5 && rcs < -12 && elevation < -5 && pNonObstacleRCS > 0.6;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 11,
        name: "applyIsMeasuredRatioCheckForRadarOnlyLongitudinallyMoving",
        description: "Checks measurement ratio for radar-only longitudinally moving objects",
        inputs: [
            { name: "numCyclesExisting", type: "number", label: "Number of Cycles Existing" },
            { name: "isOnlyUpdatedBySensorX", type: "boolean", label: "Is Only Updated By Sensor X?" },
            { name: "stateY", type: "number", label: "State Y Position" },
            { name: "totalNumSensorUpdates", type: "number", label: "Total Number of Sensor Updates" }
        ],
        condition: (inputs) => {
            const { numCyclesExisting, isOnlyUpdatedBySensorX, stateY, totalNumSensorUpdates } = inputs;
            const measurementRatio = totalNumSensorUpdates / numCyclesExisting;
            return isOnlyUpdatedBySensorX && Math.abs(stateY) > 2.0 && measurementRatio < 0.4 && numCyclesExisting > 8;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 12,
        name: "applyMicroDopplerCheck",
        description: "Checks VRU objects for expected micro-doppler signatures",
        inputs: [
            { name: "isObjectVru", type: "boolean", label: "Is Object VRU?" },
            { name: "absVelOverGround", type: "number", label: "Absolute Velocity Over Ground" },
            { name: "expectedVrHighEnough", type: "boolean", label: "Expected VR High Enough?" },
            { name: "totalNumSensorUpdates", type: "number", label: "Total Number of Sensor Updates" },
            { name: "numMicroDopplerCycles", type: "number", label: "Number of Micro Doppler Cycles" },
            { name: "numCyclesExisting", type: "number", label: "Number of Cycles Existing" }
        ],
        condition: (inputs) => {
            const { isObjectVru, absVelOverGround, expectedVrHighEnough, totalNumSensorUpdates, numMicroDopplerCycles, numCyclesExisting } = inputs;
            return isObjectVru && absVelOverGround > 1.0 && expectedVrHighEnough && totalNumSensorUpdates > 10 && numMicroDopplerCycles < 2 && numCyclesExisting > 15;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 13,
        name: "applyRadarOnlyRcsAndDrInnovationLimit",
        description: "Checks radar-only objects with high innovation and low RCS",
        inputs: [
            { name: "isOnlyUpdatedBySensorX", type: "boolean", label: "Is Only Updated By Sensor X?" },
            { name: "avgDxInnovation", type: "number", label: "Average Dx Innovation" },
            { name: "rcs", type: "number", label: "RCS (Radar Cross Section)" }
        ],
        condition: (inputs) => {
            const { isOnlyUpdatedBySensorX, avgDxInnovation, rcs } = inputs;
            return isOnlyUpdatedBySensorX && avgDxInnovation > 3.0 && rcs < -18;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 14,
        name: "applyElevationCheck",
        description: "Checks objects with inappropriate elevation (too high)",
        inputs: [
            { name: "stateX", type: "number", label: "State X Position" },
            { name: "elevationIsValid", type: "boolean", label: "Elevation Is Valid?" },
            { name: "elevation", type: "number", label: "Elevation (degrees)" },
            { name: "updatesSinceLastSensor", type: "number", label: "Updates Since Last Sensor" },
            { name: "isObjectVru", type: "boolean", label: "Is Object VRU?" }
        ],
        condition: (inputs) => {
            const { stateX, elevationIsValid, elevation, updatesSinceLastSensor, isObjectVru } = inputs;
            return stateX < 50 && elevationIsValid && elevation > 15 && updatesSinceLastSensor < 5 && !isObjectVru;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 15,
        name: "applyNonPlausibleLocationChecks",
        description: "Checks trucks with non-plausible locations",
        inputs: [
            { name: "yawAngle", type: "number", label: "Yaw Angle (degrees)" },
            { name: "nonPlausibleLocationCnt", type: "number", label: "Non-Plausible Location Count" },
            { name: "filterType", type: "select", label: "Filter Type", options: ["CA", "CV", "LA"] },
            { name: "wExistOfAssociatedVideo", type: "number", label: "W Exist of Associated Video (0-1)" }
        ],
        condition: (inputs) => {
            const { yawAngle, nonPlausibleLocationCnt, filterType, wExistOfAssociatedVideo } = inputs;
            return Math.abs(yawAngle) > 45 && nonPlausibleLocationCnt > 3 && filterType === "CA" && wExistOfAssociatedVideo < 0.3;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 16,
        name: "applyFourpluswheelerChecks",
        description: "Checks various implausible characteristics for 4+ wheeler vehicles",
        inputs: [
            { name: "totalNumSensorUpdates", type: "number", label: "Total Number of Sensor Updates" },
            { name: "velocityX", type: "number", label: "Velocity X" },
            { name: "stateX", type: "number", label: "State X Position" },
            { name: "avgDxInnovation", type: "number", label: "Average Dx Innovation" },
            { name: "rcs", type: "number", label: "RCS (Radar Cross Section)" },
            { name: "elevation", type: "number", label: "Elevation (degrees)" },
            { name: "probHasBeenObservedMoving", type: "number", label: "Probability Has Been Observed Moving (0-1)" }
        ],
        condition: (inputs) => {
            const { totalNumSensorUpdates, velocityX, stateX, avgDxInnovation, rcs, elevation, probHasBeenObservedMoving } = inputs;
            return totalNumSensorUpdates > 20 && Math.abs(velocityX) > 10 && stateX > 100 && avgDxInnovation > 2.5 && rcs > 10 && elevation > 20 && probHasBeenObservedMoving < 0.4;
        },
        action: "Disqualifies for AEB and/or ACC"
    },
    {
        id: 17,
        name: "applySplitCheckDisqualifyObjectForFunctions",
        description: "Checks non-VRU objects with split detection",
        inputs: [
            { name: "isObjectVru", type: "boolean", label: "Is Object VRU?" },
            { name: "splitCounter", type: "number", label: "Split Counter" },
            { name: "stoppingSplitCounter", type: "number", label: "Stopping Split Counter" }
        ],
        condition: (inputs) => {
            const { isObjectVru, splitCounter, stoppingSplitCounter } = inputs;
            return !isObjectVru && splitCounter > 2 && stoppingSplitCounter > 1;
        },
        action: "Disqualifies for AEB and ACC"
    },
    {
        id: 18,
        name: "applyOrientationConsistencyCheck",
        description: "Checks objects with inconsistent orientation vs velocity direction",
        inputs: [
            { name: "absVelOverGround", type: "number", label: "Absolute Velocity Over Ground" },
            { name: "numCyclesExisting", type: "number", label: "Number of Cycles Existing" },
            { name: "yawAngle", type: "number", label: "Yaw Angle (degrees)" },
            { name: "velocityX", type: "number", label: "Velocity X" },
            { name: "velocityY", type: "number", label: "Velocity Y" },
            { name: "orientationUnreliableCount", type: "number", label: "Orientation Unreliable Count" }
        ],
        condition: (inputs) => {
            const { absVelOverGround, numCyclesExisting, yawAngle, velocityX, velocityY, orientationUnreliableCount } = inputs;
            const velocityAngle = Math.atan2(velocityY, velocityX) * 180 / Math.PI;
            const angleDiff = Math.abs(yawAngle - velocityAngle);
            return absVelOverGround > 2.0 && numCyclesExisting > 10 && angleDiff > 90 && orientationUnreliableCount > 5;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 19,
        name: "modifyUnreliableOrientationCount",
        description: "Modifies orientation unreliable count for non-VRU objects",
        inputs: [
            { name: "isObjectVru", type: "boolean", label: "Is Object VRU?" },
            { name: "numCyclesNoOrientationUpdate", type: "number", label: "Number of Cycles No Orientation Update" },
            { name: "orientationUnreliableCount", type: "number", label: "Orientation Unreliable Count" },
            { name: "egoYawRate", type: "number", label: "Ego Yaw Rate (rad/s)" }
        ],
        condition: (inputs) => {
            const { isObjectVru, numCyclesNoOrientationUpdate, orientationUnreliableCount, egoYawRate } = inputs;
            return !isObjectVru && numCyclesNoOrientationUpdate > 5 && orientationUnreliableCount < 10 && Math.abs(egoYawRate) > 0.2;
        },
        action: "Modifies orientation unreliable count"
    },
    {
        id: 20,
        name: "applyStationaryVruVideoGhostCheck",
        description: "Checks stationary VRU with high RCS and low video existence",
        inputs: [
            { name: "rcs", type: "number", label: "RCS (Radar Cross Section)" },
            { name: "wExistOfAssociatedVideo", type: "number", label: "W Exist of Associated Video (0-1)" },
            { name: "updatesSinceLastSensor", type: "number", label: "Updates Since Last Sensor" },
            { name: "isObjectVru", type: "boolean", label: "Is Object VRU?" },
            { name: "absVelOverGround", type: "number", label: "Absolute Velocity Over Ground" }
        ],
        condition: (inputs) => {
            const { rcs, wExistOfAssociatedVideo, updatesSinceLastSensor, isObjectVru, absVelOverGround } = inputs;
            return isObjectVru && absVelOverGround < 0.5 && rcs > 0 && wExistOfAssociatedVideo < 0.2 && updatesSinceLastSensor < 3;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 21,
        name: "applyInnovationCheck",
        description: "Checks objects with high dx innovation in relevant range",
        inputs: [
            { name: "stateX", type: "number", label: "State X Position" },
            { name: "stateY", type: "number", label: "State Y Position" },
            { name: "avgDxInnovation", type: "number", label: "Average Dx Innovation" },
            { name: "isVruObject", type: "boolean", label: "Is VRU Object?" }
        ],
        condition: (inputs) => {
            const { stateX, stateY, avgDxInnovation, isVruObject } = inputs;
            const range = Math.sqrt(stateX * stateX + stateY * stateY);
            return range < 80 && avgDxInnovation > 2.0 && !isVruObject;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 22,
        name: "calcDxInnovationThreshold",
        description: "Calculates threshold value for dx innovation",
        inputs: [
            { name: "stateX", type: "number", label: "State X Position" },
            { name: "stateY", type: "number", label: "State Y Position" },
            { name: "rcs", type: "number", label: "RCS (Radar Cross Section)" },
            { name: "vyUnreliableAccumulated", type: "number", label: "VY Unreliable Accumulated" },
            { name: "numCyclesExisting", type: "number", label: "Number of Cycles Existing" }
        ],
        condition: (inputs) => {
            const { stateX, stateY, rcs, vyUnreliableAccumulated, numCyclesExisting } = inputs;
            const range = Math.sqrt(stateX * stateX + stateY * stateY);
            const threshold = 1.5 + (range / 100) + (rcs / 20) + (vyUnreliableAccumulated / 10);
            return threshold > 2.5 && numCyclesExisting > 5;
        },
        action: "Returns Float32 threshold value for dx innovation"
    },
    {
        id: 23,
        name: "applyImplausibleVyVruCheck",
        description: "Checks VRU with LA filter type and high VY velocity",
        inputs: [
            { name: "isObjectVru", type: "boolean", label: "Is Object VRU?" },
            { name: "absVelOverGroundY", type: "number", label: "Absolute Velocity Over Ground Y" },
            { name: "egoYawRate", type: "number", label: "Ego Yaw Rate (rad/s)" },
            { name: "filterType", type: "select", label: "Filter Type", options: ["CA", "CV", "LA"] }
        ],
        condition: (inputs) => {
            const { isObjectVru, absVelOverGroundY, egoYawRate, filterType } = inputs;
            return isObjectVru && filterType === "LA" && absVelOverGroundY > 3.0 && Math.abs(egoYawRate) < 0.1;
        },
        action: "Disqualifies for AEB and ACC"
    },
    {
        id: 24,
        name: "applySensorBasedInnoCheck",
        description: "Checks objects with high bad sensor innovation count",
        inputs: [
            { name: "badSensorBasedInnoCount", type: "number", label: "Bad Sensor Based Innovation Count" },
            { name: "stateX", type: "number", label: "State X Position" },
            { name: "stateY", type: "number", label: "State Y Position" },
            { name: "isObjectVru", type: "boolean", label: "Is Object VRU?" }
        ],
        condition: (inputs) => {
            const { badSensorBasedInnoCount, stateX, stateY, isObjectVru } = inputs;
            const range = Math.sqrt(stateX * stateX + stateY * stateY);
            return badSensorBasedInnoCount > 5 && range < 100 && !isObjectVru;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 25,
        name: "applyImplausibleVideoTtcForVru",
        description: "Checks VRU with implausible video TTC",
        inputs: [
            { name: "isVru", type: "boolean", label: "Is VRU?" },
            { name: "updatesSinceLastSensor", type: "number", label: "Updates Since Last Sensor" },
            { name: "videoInvTtc", type: "number", label: "Video Inverse TTC" }
        ],
        condition: (inputs) => {
            const { isVru, updatesSinceLastSensor, videoInvTtc } = inputs;
            return isVru && updatesSinceLastSensor < 5 && videoInvTtc > 0.5;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 26,
        name: "applyVyInconsistentCheck",
        description: "Checks VRU with inconsistent VY measurements",
        inputs: [
            { name: "vyInconsistent", type: "boolean", label: "VY Inconsistent?" },
            { name: "filterType", type: "select", label: "Filter Type", options: ["CA", "CV", "LA"] },
            { name: "stateX", type: "number", label: "State X Position" },
            { name: "vyUnreliableAccumulated", type: "number", label: "VY Unreliable Accumulated" },
            { name: "isVru", type: "boolean", label: "Is VRU?" }
        ],
        condition: (inputs) => {
            const { vyInconsistent, filterType, stateX, vyUnreliableAccumulated, isVru } = inputs;
            return isVru && vyInconsistent && filterType === "LA" && stateX < 60 && vyUnreliableAccumulated > 5;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 27,
        name: "applyVideoHandleSharedCheck",
        description: "Checks stationary VRU sharing video handle with moving object",
        inputs: [
            { name: "probHasBeenObservedMoving", type: "number", label: "Probability Has Been Observed Moving (0-1)" },
            { name: "updatesSinceLastSensor", type: "number", label: "Updates Since Last Sensor" },
            { name: "isRecentlyUsedVideoHandleValid", type: "boolean", label: "Is Recently Used Video Handle Valid?" },
            { name: "isObjectVru", type: "boolean", label: "Is Object VRU?" },
            { name: "absVelOverGround", type: "number", label: "Absolute Velocity Over Ground" }
        ],
        condition: (inputs) => {
            const { probHasBeenObservedMoving, updatesSinceLastSensor, isRecentlyUsedVideoHandleValid, isObjectVru, absVelOverGround } = inputs;
            return isObjectVru && absVelOverGround < 0.5 && probHasBeenObservedMoving < 0.2 && updatesSinceLastSensor < 3 && isRecentlyUsedVideoHandleValid;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 28,
        name: "modifyBadSensorBasedInnoCount",
        description: "Updates bad sensor innovation counter",
        inputs: [
            { name: "referencePointX", type: "number", label: "Reference Point X" },
            { name: "referencePointY", type: "number", label: "Reference Point Y" },
            { name: "radarBasedInnovation", type: "number", label: "Radar Based Innovation" },
            { name: "videoBasedInnovation", type: "number", label: "Video Based Innovation" },
            { name: "updatesSinceLastSensor", type: "number", label: "Updates Since Last Sensor" }
        ],
        condition: (inputs) => {
            const { referencePointX, referencePointY, radarBasedInnovation, videoBasedInnovation, updatesSinceLastSensor } = inputs;
            const range = Math.sqrt(referencePointX * referencePointX + referencePointY * referencePointY);
            return range < 80 && radarBasedInnovation > 2.0 && videoBasedInnovation > 1.5 && updatesSinceLastSensor < 5;
        },
        action: "Updates bad sensor innovation counter"
    },
    {
        id: 29,
        name: "applyRadarOnlyNLDCheck",
        description: "Checks radar-only objects that are NLD candidates",
        inputs: [
            { name: "totalNumSensorUpdates", type: "number", label: "Total Number of Sensor Updates" },
            { name: "stateX", type: "number", label: "State X Position" },
            { name: "stateY", type: "number", label: "State Y Position" },
            { name: "velocityX", type: "number", label: "Velocity X" },
            { name: "velocityY", type: "number", label: "Velocity Y" },
            { name: "numCyclesExisting", type: "number", label: "Number of Cycles Existing" }
        ],
        condition: (inputs) => {
            const { totalNumSensorUpdates, stateX, stateY, velocityX, velocityY, numCyclesExisting } = inputs;
            const range = Math.sqrt(stateX * stateX + stateY * stateY);
            const speed = Math.sqrt(velocityX * velocityX + velocityY * velocityY);
            return totalNumSensorUpdates > 10 && range > 120 && speed < 1.0 && numCyclesExisting > 15;
        },
        action: "Disqualifies for AEB and ACC"
    },
    {
        id: 30,
        name: "applyRadarOnlyStationaryCheck",
        description: "Checks radar-only stationary objects",
        inputs: [
            { name: "absVelOverGround", type: "number", label: "Absolute Velocity Over Ground" },
            { name: "totalNumSensorUpdates", type: "number", label: "Total Number of Sensor Updates" },
            { name: "isOnlyRadarUpdated", type: "boolean", label: "Is Only Radar Updated?" }
        ],
        condition: (inputs) => {
            const { absVelOverGround, totalNumSensorUpdates, isOnlyRadarUpdated } = inputs;
            return absVelOverGround < 0.5 && totalNumSensorUpdates > 8 && isOnlyRadarUpdated;
        },
        action: "Disqualifies for AEB and ACC"
    },
    {
        id: 31,
        name: "applyIsMeasuredRatioCheckForStandingLongiVru",
        description: "Checks standing longitudinal VRU with poor measurement history",
        inputs: [
            { name: "filterType", type: "select", label: "Filter Type", options: ["CA", "CV", "LA"] },
            { name: "probHasBeenObservedMoving", type: "number", label: "Probability Has Been Observed Moving (0-1)" },
            { name: "probIsCurrentlyMoving", type: "number", label: "Probability Is Currently Moving (0-1)" },
            { name: "numCyclesExisting", type: "number", label: "Number of Cycles Existing" },
            { name: "numSingleUpdateBySensorX", type: "number", label: "Number of Single Update By Sensor X" },
            { name: "isObjectVru", type: "boolean", label: "Is Object VRU?" }
        ],
        condition: (inputs) => {
            const { filterType, probHasBeenObservedMoving, probIsCurrentlyMoving, numCyclesExisting, numSingleUpdateBySensorX, isObjectVru } = inputs;
            const measurementRatio = numSingleUpdateBySensorX / numCyclesExisting;
            return isObjectVru && filterType === "LA" && probHasBeenObservedMoving < 0.2 && probIsCurrentlyMoving < 0.1 && measurementRatio < 0.3;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 32,
        name: "applyImplausiblyAcceleratingVruCheck",
        description: "Checks close, young VRU with implausibly high forward acceleration",
        inputs: [
            { name: "stateX", type: "number", label: "State X Position" },
            { name: "numCyclesExisting", type: "number", label: "Number of Cycles Existing" },
            { name: "accelerationX", type: "number", label: "Acceleration X" },
            { name: "isObjectVru", type: "boolean", label: "Is Object VRU?" }
        ],
        condition: (inputs) => {
            const { stateX, numCyclesExisting, accelerationX, isObjectVru } = inputs;
            return isObjectVru && stateX < 20 && numCyclesExisting < 10 && accelerationX > 5.0;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 33,
        name: "applyUndefinedCrossingVruFromCorner",
        description: "Checks corner-detected VRU with undefined crossing behavior",
        inputs: [
            { name: "totalNumSensorUpdates", type: "number", label: "Total Number of Sensor Updates" },
            { name: "stateX", type: "number", label: "State X Position" },
            { name: "stateY", type: "number", label: "State Y Position" },
            { name: "numCyclesExisting", type: "number", label: "Number of Cycles Existing" },
            { name: "filterType", type: "select", label: "Filter Type", options: ["CA", "CV", "LA"] },
            { name: "probHasBeenObservedMoving", type: "number", label: "Probability Has Been Observed Moving (0-1)" },
            { name: "isObjectVru", type: "boolean", label: "Is Object VRU?" }
        ],
        condition: (inputs) => {
            const { totalNumSensorUpdates, stateX, stateY, numCyclesExisting, filterType, probHasBeenObservedMoving, isObjectVru } = inputs;
            return isObjectVru && Math.abs(stateY) > 3.0 && stateX < 30 && totalNumSensorUpdates > 5 && numCyclesExisting < 15 && filterType === "CA" && probHasBeenObservedMoving < 0.3;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 34,
        name: "applyImplausiblePedCheck",
        description: "Checks pedestrians with implausible characteristics",
        inputs: [
            { name: "stateY", type: "number", label: "State Y Position" },
            { name: "isObjectPedestrian", type: "boolean", label: "Is Object Pedestrian?" },
            { name: "numMicroDopplerCycles", type: "number", label: "Number of Micro Doppler Cycles" },
            { name: "rcs", type: "number", label: "RCS (Radar Cross Section)" },
            { name: "totalNumSensorUpdates", type: "number", label: "Total Number of Sensor Updates" },
            { name: "stationaryLocationsOnlyCounter", type: "number", label: "Stationary Locations Only Counter" },
            { name: "elevation", type: "number", label: "Elevation (degrees)" }
        ],
        condition: (inputs) => {
            const { stateY, isObjectPedestrian, numMicroDopplerCycles, rcs, totalNumSensorUpdates, stationaryLocationsOnlyCounter, elevation } = inputs;
            return isObjectPedestrian && Math.abs(stateY) > 4.0 && numMicroDopplerCycles < 2 && rcs > 10 && totalNumSensorUpdates > 15 && stationaryLocationsOnlyCounter > 8 && elevation > 10;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 35,
        name: "applyImplausiblePedCheckLRR",
        description: "LRR-specific implausible pedestrian checks",
        inputs: [
            { name: "stateY", type: "number", label: "State Y Position" },
            { name: "updatesSinceLastSensor", type: "number", label: "Updates Since Last Sensor" },
            { name: "isObjectPedestrian", type: "boolean", label: "Is Object Pedestrian?" },
            { name: "numMicroDopplerCycles", type: "number", label: "Number of Micro Doppler Cycles" },
            { name: "rcs", type: "number", label: "RCS (Radar Cross Section)" },
            { name: "expectedVrHighEnough", type: "boolean", label: "Expected VR High Enough?" },
            { name: "stationaryLocationsOnlyCounter", type: "number", label: "Stationary Locations Only Counter" }
        ],
        condition: (inputs) => {
            const { stateY, updatesSinceLastSensor, isObjectPedestrian, numMicroDopplerCycles, rcs, expectedVrHighEnough, stationaryLocationsOnlyCounter } = inputs;
            return isObjectPedestrian && Math.abs(stateY) > 5.0 && updatesSinceLastSensor < 3 && numMicroDopplerCycles < 1 && rcs > 15 && !expectedVrHighEnough && stationaryLocationsOnlyCounter > 10;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 36,
        name: "applyImplausibleCarAtCloseRangeCheck",
        description: "Checks long cars at close range updated only by front center radar",
        inputs: [
            { name: "stateX", type: "number", label: "State X Position" },
            { name: "dimensionLength", type: "number", label: "Dimension Length" },
            { name: "isObjectCar", type: "boolean", label: "Is Object Car?" },
            { name: "updatesSinceLastSensor", type: "number", label: "Updates Since Last Sensor" },
            { name: "isOnlyFrontCenterRadar", type: "boolean", label: "Is Only Front Center Radar?" }
        ],
        condition: (inputs) => {
            const { stateX, dimensionLength, isObjectCar, updatesSinceLastSensor, isOnlyFrontCenterRadar } = inputs;
            return isObjectCar && stateX < 15 && dimensionLength > 6.0 && updatesSinceLastSensor < 3 && isOnlyFrontCenterRadar;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 37,
        name: "applyElevatedObjectCheck",
        description: "Checks elevated objects with unreliable characteristics",
        inputs: [
            { name: "rcs", type: "number", label: "RCS (Radar Cross Section)" },
            { name: "elevationIsValid", type: "boolean", label: "Elevation Is Valid?" },
            { name: "elevation", type: "number", label: "Elevation (degrees)" },
            { name: "updatesSinceLastSensor", type: "number", label: "Updates Since Last Sensor" },
            { name: "numCyclesExisting", type: "number", label: "Number of Cycles Existing" },
            { name: "stationaryLocationsOnlyCounter", type: "number", label: "Stationary Locations Only Counter" },
            { name: "stateX", type: "number", label: "State X Position" }
        ],
        condition: (inputs) => {
            const { rcs, elevationIsValid, elevation, updatesSinceLastSensor, numCyclesExisting, stationaryLocationsOnlyCounter, stateX } = inputs;
            return elevationIsValid && elevation > 25 && rcs < -5 && updatesSinceLastSensor < 5 && numCyclesExisting > 8 && stationaryLocationsOnlyCounter > 5 && stateX < 80;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 38,
        name: "applyBridgeCheck",
        description: "Checks objects with bridge-like characteristics",
        inputs: [
            { name: "videoBasedInnovation", type: "number", label: "Video Based Innovation" },
            { name: "elevationIsValid", type: "boolean", label: "Elevation Is Valid?" },
            { name: "elevation", type: "number", label: "Elevation (degrees)" },
            { name: "stationaryLocationsOnlyCounter", type: "number", label: "Stationary Locations Only Counter" },
            { name: "updatesSinceLastSensor", type: "number", label: "Updates Since Last Sensor" }
        ],
        condition: (inputs) => {
            const { videoBasedInnovation, elevationIsValid, elevation, stationaryLocationsOnlyCounter, updatesSinceLastSensor } = inputs;
            return videoBasedInnovation > 3.0 && elevationIsValid && elevation > 30 && stationaryLocationsOnlyCounter > 10 && updatesSinceLastSensor < 4;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 39,
        name: "applyCornerRadarAssoWithStationaryLocationsForTheFirstTime",
        description: "Checks young VRU first detected by corner radar with stationary locations",
        inputs: [
            { name: "isObjectVru", type: "boolean", label: "Is Object VRU?" },
            { name: "numCyclesExisting", type: "number", label: "Number of Cycles Existing" },
            { name: "totalNumSensorUpdates", type: "number", label: "Total Number of Sensor Updates" },
            { name: "stationaryLocationsOnlyCounter", type: "number", label: "Stationary Locations Only Counter" },
            { name: "isCornerRadarDetected", type: "boolean", label: "Is Corner Radar Detected?" }
        ],
        condition: (inputs) => {
            const { isObjectVru, numCyclesExisting, totalNumSensorUpdates, stationaryLocationsOnlyCounter, isCornerRadarDetected } = inputs;
            return isObjectVru && numCyclesExisting < 8 && totalNumSensorUpdates < 5 && stationaryLocationsOnlyCounter > 2 && isCornerRadarDetected;
        },
        action: "Disqualifies for AEB"
    },
    {
        id: 40,
        name: "applyInconsistentAlphaCheck",
        description: "Checks objects with inconsistent radar and video alpha innovations",
        inputs: [
            { name: "radarRawAlphaInnovation", type: "number", label: "Radar Raw Alpha Innovation" },
            { name: "videoRawAlphaInnovation", type: "number", label: "Video Raw Alpha Innovation" },
            { name: "stateX", type: "number", label: "State X Position" },
            { name: "stateY", type: "number", label: "State Y Position" },
            { name: "updatesSinceLastSensor", type: "number", label: "Updates Since Last Sensor" }
        ],
        condition: (inputs) => {
            const { radarRawAlphaInnovation, videoRawAlphaInnovation, stateX, stateY, updatesSinceLastSensor } = inputs;
            const range = Math.sqrt(stateX * stateX + stateY * stateY);
            const alphaDiff = Math.abs(radarRawAlphaInnovation - videoRawAlphaInnovation);
            return alphaDiff > 0.2 && range < 60 && updatesSinceLastSensor < 5;
        },
        action: "Disqualifies for AEB"
    }
];

// Application State
let testResults = {};
let currentStats = { hit: 0, miss: 0, total: 40 };

// Initialize the application
document.addEventListener('DOMContentLoaded', function() {
    renderFunctionCards();
    updateStats();
    
    // Event listeners for main controls
    document.getElementById('test-all-btn').addEventListener('click', testAllFunctions);
    document.getElementById('reset-all-btn').addEventListener('click', resetAllFunctions);
    document.getElementById('random-values-btn').addEventListener('click', fillRandomValues);
});

function renderFunctionCards() {
    const grid = document.getElementById('functions-grid');
    grid.innerHTML = '';
    
    functions.forEach(func => {
        const card = createFunctionCard(func);
        grid.appendChild(card);
    });
}

function createFunctionCard(func) {
    const card = document.createElement('div');
    card.className = 'function-card';
    card.id = `function-${func.id}`;
    
    const inputsHtml = func.inputs.map(input => {
        if (input.type === 'select') {
            const options = input.options.map(opt => `<option value="${opt}">${opt}</option>`).join('');
            return `
                <div class="input-group">
                    <label for="${func.id}-${input.name}">${input.label}</label>
                    <select id="${func.id}-${input.name}" name="${input.name}">
                        <option value="">Select...</option>
                        ${options}
                    </select>
                </div>
            `;
        } else if (input.type === 'boolean') {
            return `
                <div class="input-group">
                    <label for="${func.id}-${input.name}">${input.label}</label>
                    <select id="${func.id}-${input.name}" name="${input.name}">
                        <option value="">Select...</option>
                        <option value="true">True</option>
                        <option value="false">False</option>
                    </select>
                </div>
            `;
        } else {
            return `
                <div class="input-group">
                    <label for="${func.id}-${input.name}">${input.label}</label>
                    <input type="number" id="${func.id}-${input.name}" name="${input.name}" step="0.1" placeholder="Enter value">
                </div>
            `;
        }
    }).join('');
    
    card.innerHTML = `
        <div class="function-header">
            <h3 class="function-title">${func.name}</h3>
            <span class="function-status status-pending">Pending</span>
        </div>
        <p class="function-description">${func.description}</p>
        <div class="function-inputs">
            ${inputsHtml}
        </div>
        <button class="test-btn" onclick="testFunction(${func.id})">Test Function</button>
        <div class="result" id="result-${func.id}" style="display: none;"></div>
    `;
    
    return card;
}

function testFunction(funcId) {
    const func = functions.find(f => f.id === funcId);
    const inputs = {};
    let allInputsValid = true;
    
    // Collect input values
    func.inputs.forEach(input => {
        const element = document.getElementById(`${funcId}-${input.name}`);
        let value = element.value;
        
        if (value === '') {
            allInputsValid = false;
            return;
        }
        
        // Convert values based on type
        if (input.type === 'number') {
            value = parseFloat(value);
            if (isNaN(value)) {
                allInputsValid = false;
                return;
            }
        } else if (input.type === 'boolean') {
            value = value === 'true';
        }
        
        inputs[input.name] = value;
    });
    
    if (!allInputsValid) {
        alert('Please fill in all input fields with valid values.');
        return;
    }
    
    // Test the function condition
    const isHit = func.condition(inputs);
    testResults[funcId] = { isHit, inputs, func };
    
    // Update the UI
    updateFunctionCard(funcId, isHit);
    updateStats();
    updateResultsSummary();
}

function updateFunctionCard(funcId, isHit) {
    const card = document.getElementById(`function-${funcId}`);
    const status = card.querySelector('.function-status');
    const result = document.getElementById(`result-${funcId}`);
    
    // Update card styling
    card.className = `function-card ${isHit ? 'hit' : 'miss'}`;
    
    // Update status badge
    status.className = `function-status ${isHit ? 'status-hit' : 'status-miss'}`;
    status.textContent = isHit ? 'Hit' : 'Miss';
    
    // Update result display
    result.className = `result ${isHit ? 'hit' : 'miss'}`;
    result.textContent = isHit ? 
        `✅ Condition HIT! ${testResults[funcId].func.action}` : 
        `❌ Condition MISS. Function not triggered.`;
    result.style.display = 'block';
}

function testAllFunctions() {
    let successCount = 0;
    
    functions.forEach(func => {
        const inputs = {};
        let allInputsValid = true;
        
        // Check if all inputs are filled
        func.inputs.forEach(input => {
            const element = document.getElementById(`${func.id}-${input.name}`);
            let value = element.value;
            
            if (value === '') {
                allInputsValid = false;
                return;
            }
            
            // Convert values based on type
            if (input.type === 'number') {
                value = parseFloat(value);
                if (isNaN(value)) {
                    allInputsValid = false;
                    return;
                }
            } else if (input.type === 'boolean') {
                value = value === 'true';
            }
            
            inputs[input.name] = value;
        });
        
        if (allInputsValid) {
            const isHit = func.condition(inputs);
            testResults[func.id] = { isHit, inputs, func };
            updateFunctionCard(func.id, isHit);
            if (isHit) successCount++;
        }
    });
    
    updateStats();
    updateResultsSummary();
    
    // Show summary alert
    const totalTested = Object.keys(testResults).length;
    alert(`Testing complete!\n${successCount} functions hit out of ${totalTested} tested.\nSuccess rate: ${((successCount / totalTested) * 100).toFixed(1)}%`);
}

function resetAllFunctions() {
    testResults = {};
    
    functions.forEach(func => {
        const card = document.getElementById(`function-${func.id}`);
        const status = card.querySelector('.function-status');
        const result = document.getElementById(`result-${func.id}`);
        
        // Reset card styling
        card.className = 'function-card';
        
        // Reset status badge
        status.className = 'function-status status-pending';
        status.textContent = 'Pending';
        
        // Hide result
        result.style.display = 'none';
        
        // Clear input values
        func.inputs.forEach(input => {
            const element = document.getElementById(`${func.id}-${input.name}`);
            element.value = '';
        });
    });
    
    updateStats();
    updateResultsSummary();
}

function fillRandomValues() {
    functions.forEach(func => {
        func.inputs.forEach(input => {
            const element = document.getElementById(`${func.id}-${input.name}`);
            
            if (input.type === 'number') {
                // Generate random numbers based on typical ranges for automotive perception
                let randomValue;
                if (input.name.includes('rcs') || input.name.includes('RCS')) {
                    randomValue = (Math.random() - 0.5) * 40; // -20 to 20 dBsm
                } else if (input.name.includes('elevation') || input.name.includes('Elevation')) {
                    randomValue = (Math.random() - 0.5) * 60; // -30 to 30 degrees
                } else if (input.name.includes('prob') || input.name.includes('Prob')) {
                    randomValue = Math.random(); // 0 to 1
                } else if (input.name.includes('angle') || input.name.includes('Angle')) {
                    randomValue = (Math.random() - 0.5) * 360; // -180 to 180 degrees
                } else if (input.name.includes('velocity') || input.name.includes('Velocity')) {
                    randomValue = (Math.random() - 0.5) * 20; // -10 to 10 m/s
                } else if (input.name.includes('position') || input.name.includes('Position') || input.name.includes('state')) {
                    randomValue = (Math.random() - 0.5) * 200; // -100 to 100 m
                } else if (input.name.includes('innovation') || input.name.includes('Innovation')) {
                    randomValue = Math.random() * 5; // 0 to 5
                } else if (input.name.includes('count') || input.name.includes('Count') || input.name.includes('cycles') || input.name.includes('Cycles')) {
                    randomValue = Math.floor(Math.random() * 20); // 0 to 19
                } else {
                    randomValue = Math.random() * 10; // Default: 0 to 10
                }
                element.value = randomValue.toFixed(2);
            } else if (input.type === 'boolean') {
                element.value = Math.random() > 0.5 ? 'true' : 'false';
            } else if (input.type === 'select') {
                const options = input.options;
                const randomIndex = Math.floor(Math.random() * options.length);
                element.value = options[randomIndex];
            }
        });
    });
}

function updateStats() {
    const totalTested = Object.keys(testResults).length;
    const hitCount = Object.values(testResults).filter(result => result.isHit).length;
    const missCount = totalTested - hitCount;
    const successRate = totalTested > 0 ? ((hitCount / totalTested) * 100).toFixed(1) : 0;
    
    document.getElementById('total-functions').textContent = functions.length;
    document.getElementById('functions-hit').textContent = hitCount;
    document.getElementById('success-rate').textContent = `${successRate}%`;
}

function updateResultsSummary() {
    const summaryContent = document.querySelector('.summary-content');
    
    if (Object.keys(testResults).length === 0) {
        summaryContent.innerHTML = '<p>Click "Test All Functions" to see detailed results</p>';
        return;
    }
    
    const hitResults = Object.values(testResults).filter(result => result.isHit);
    const missResults = Object.values(testResults).filter(result => !result.isHit);
    
    let html = '<div class="summary-stats">';
    html += `<p><strong>Functions Hit:</strong> ${hitResults.length}</p>`;
    html += `<p><strong>Functions Miss:</strong> ${missResults.length}</p>`;
    html += `<p><strong>Total Tested:</strong> ${Object.keys(testResults).length}</p>`;
    html += '</div>';
    
    if (hitResults.length > 0) {
        html += '<h4>Functions That Hit:</h4>';
        html += '<ul class="summary-list">';
        hitResults.forEach(result => {
            html += `<li class="summary-hit">✅ ${result.func.name} - ${result.func.action}</li>`;
        });
        html += '</ul>';
    }
    
    if (missResults.length > 0) {
        html += '<h4>Functions That Missed:</h4>';
        html += '<ul class="summary-list">';
        missResults.forEach(result => {
            html += `<li class="summary-miss">❌ ${result.func.name} - Condition not met</li>`;
        });
        html += '</ul>';
    }
    
    summaryContent.innerHTML = html;
}