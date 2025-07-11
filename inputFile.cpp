
#include "per/variation_points/depPolicies/inc/postProcessing/per_depPostProcessingAlgos.hpp"

#include "envmodel_lib/postProcessing/per_postProcessingHelper.hpp"

#include "prm/gen/PerParam/api/prm_PerParam_cfg.hpp"

#if defined(RB_DC_DEBUG_STATES_ENABLED)
#   include "per/modules/types/root/src/per_debugTypes.hpp"
#endif

void Dc::Per::Dep::applySuppressionUntilNextVideoUpdateCheck(
   const ::Per::InterfaceDep::DepObject& obj, vfc::uint16_t& functionRelevanceBitField)
{
   if (obj.getIsSuppressedUntilNextVideoUpdate())
   {
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(
         ::Per::Parameters::PerPostProcessTypes::applySuppressionUntilNextVideoUpdateCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);
   }
}

void Dc::Per::Dep::applyPostProcessVideoOtcCheck(
   const ::Per::InterfaceDep::DepObject& obj, vfc::uint16_t& functionRelevanceBitField)
{
   if (obj.getIsSuppressedDueToVideoOtcPostProcessing())
   {
      //  debug post-processing bit have already been set in video OTC post-processing
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);
   }
}

bool Dc::Per::Dep::isMovingTowardsEgoLane(
   const vfc::float32_t dyObj, const vfc::float32_t vyObjRel, const vfc::float32_t vyObjOverGround)
{
   return vfc::isNegative(dyObj * vyObjRel) || vfc::isNegative(dyObj * vyObjOverGround);
}

bool Dc::Per::Dep::isDepObjProbablyVideoGhost(const ::Per::InterfaceDep::DepObject& depObj)
{
   const bool isInitialRadarUpdatePhase{
      depObj.sensorFilterFusHelper.getTotalNumSensorUpdates(
         ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar)
         > 0U
      && depObj.sensorFilterFusHelper.getTotalNumSensorUpdates(
            ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar)
            < 3U
      && depObj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
            ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar)
            == 0U};
   const bool isTrackedByVideo{
      depObj.sensorFilterFusHelper.getTotalNumSensorUpdates(
         ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo)
      > 3U};
   const bool hasNotBeenUpdatedByCornerRadar{
      depObj.sensorFilterFusHelper.getTotalNumSensorUpdates(
         ::Per::Parameters::ObstacleSensorIndex::frontLeftCornerObjectRadar)
         == 0U
      && depObj.sensorFilterFusHelper.getTotalNumSensorUpdates(
            ::Per::Parameters::ObstacleSensorIndex::frontRightCornerObjectRadar)
            == 0U};
   const bool isAlmostVideoOnly{
      isTrackedByVideo && hasNotBeenUpdatedByCornerRadar && isInitialRadarUpdatePhase};
   const bool hasNoMicroDoppler{
      depObj.getNumberMicroDopplerCycles() == 0U && depObj.getExpectedVrHighEnoughForMuDopplerCounter() > 0U};
   const bool isVeryLowRcs{depObj.get_rcs() < -15.F};

   return isAlmostVideoOnly && hasNoMicroDoppler && isVeryLowRcs;
}

void Dc::Per::Dep::applyUnreliableAngularVelocityCheck(
   const ::Per::InterfaceDep::DepObject&          obj,
   const ::Per::Frames::Rot1dAcc2dFor&            hvmFor,
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround,
   const bool                                     isObjectVru,
   const bool                                     isDepObjProbablyVideoGhost,
   vfc::uint16_t&                                 functionRelevanceBitField)
{
   constexpr vfc::float32_t pMovingThreshold{0.15F};
   const bool               isStandingCar{
      (obj.get_probHasBeenObservedMoving() < pMovingThreshold)
      && (obj.get_probIsCurrentlyMoving() < pMovingThreshold)
      && obj.get_objectTypeTreeReadOnly().isOfTypeOrSubType4PlusWheeler()};

   constexpr vfc::float32_t turningEgoYawRateThreshold{10.0F * vfc::typedDegree2Radian<vfc::float32_t>()};
   const bool               isEgoTurning{vfc::abs(hvmFor.getAngleDt(0U)) > turningEgoYawRateThreshold};

   constexpr vfc::uint16_t numCyclesExistingThreshold{7U};
   const bool hasBeenCreatedVeryRecently{obj.get_numCyclesExisting() < numCyclesExistingThreshold};

   const vfc::float32_t isCrossingFastThreshold = isObjectVru && isEgoTurning ? 0.88F :
                                                  isObjectVru                 ? 0.68F :
                                                  isStandingCar               ? 0.35F :
                                                  hasBeenCreatedVeryRecently ?
                                                                  0.5F :
                                                                  vfc::numeric_limits<vfc::float32_t>::max();

   const bool isCrossingFast{absVelOverGround[1] > isCrossingFastThreshold};

   const bool wasMeasuredByFrontCorner =
      obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
         ::Per::Parameters::ObstacleSensorIndex::frontLeftCornerObjectRadar)
         > 0U
      || obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
            ::Per::Parameters::ObstacleSensorIndex::frontRightCornerObjectRadar)
            > 0U;

   const vfc::uint8_t cyclesSinceLastVideoUpdateWithAngularVelocityThreshold{
      isObjectVru ? static_cast<vfc::uint8_t>(1) : static_cast<vfc::uint8_t>(0)};
   const bool hasRecentlyBeenUpdatedWithAngularVelocity{
      obj.getNumCyclesSinceLastVideoUpdateWithAngularVelocity()
      <= cyclesSinceLastVideoUpdateWithAngularVelocityThreshold};

   const vfc::linalg::TVectorN<vfc::float32_t, 2> videoBasedInno{obj.getVideoBasedInnovation()};

   constexpr vfc::float32_t drInnoThreshold{3.F};
   constexpr vfc::float32_t alphaInnoThreshold{4.F * vfc::typedDegree2Radian<vfc::float32_t>()};

   const bool areInnosUnreliable{
      videoBasedInno[0] > drInnoThreshold && videoBasedInno[1] > alphaInnoThreshold};

   const bool areVideoUpdatesSuspicious{
      hasRecentlyBeenUpdatedWithAngularVelocity || obj.getCreatedByVideoWithHighVy() || areInnosUnreliable};

   const vfc::float32_t                           dxObj{obj.getState().getX()};
   const vfc::float32_t                           dyObj{obj.getState().getY()};
   const vfc::float32_t                           vyObjRel{obj.getState().getVelocity(1U)};
   const vfc::linalg::TVectorN<vfc::float32_t, 2> velOverGround =
      ::Per::Algos::ObjectFunctions::calcVelocityOverGround(obj, hvmFor);
   const vfc::float32_t vyObjOverGround{velOverGround[1]};

   constexpr vfc::uint8_t lowerRadarConfirmationThresholdDefault{1U};
   constexpr vfc::uint8_t lowerRadarConfirmationThresholdForVideoGhost{0U};
   const vfc::uint8_t     lowerRadarConfirmationThreshold{
      isDepObjProbablyVideoGhost ? lowerRadarConfirmationThresholdForVideoGhost :
                                       lowerRadarConfirmationThresholdDefault};

   // Check if object is a unreliable moving pedestrian
   // Check if object is a young pedestrian
   const bool isPedestrian{
      obj.get_objectTypeTreeReadOnly().get_mostProbableConditionalType()
      == ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_OBSTACLE_MOBILE_PEDESTRIAN};
   constexpr vfc::uint8_t objYoungThresh{20U};
   const bool             objIsYoung{obj.get_numCyclesExisting() < objYoungThresh};

   // Check if object is considerably moving fast and not in ego lane
   constexpr vfc::float32_t movingVelocityThreshold{2.0F};
   const bool               isConsiderablyMovingFast{absVelOverGround[1] > movingVelocityThreshold};
   constexpr vfc::float32_t laneWidthThresh{2.5F};
   const bool               objectNotInDrivingCorridor = vfc::abs(obj.getState().getY()) > laneWidthThresh;

   // check if object has enough static location and not good enough micro doppler
   constexpr vfc::uint16_t statLocThreshold{4U};
   const bool              hasHighStatLocCount = obj.getStationaryLocationsOnlyCounter() >= statLocThreshold;
   const bool              isMicroDopplerNotGoodEnough =
      obj.getExpectedVrHighEnoughForMuDopplerCounter() >= 5U && obj.getNumberMicroDopplerCycles() <= 2U;

   // Check if object is a pedestrian with unreliable moving characteristics
   const bool unreliableMovingVru{
      isPedestrian && isConsiderablyMovingFast && hasHighStatLocCount
      && obj.get_probIsCurrentlyMoving() < 0.35F && objIsYoung && objectNotInDrivingCorridor
      && isMicroDopplerNotGoodEnough};

   // check if Association suspicious for pedestrian object
   const bool isAssociationSuspicious{
      isPedestrian && isConsiderablyMovingFast && isMovingTowardsEgoLane && (absVelOverGround[0] > 1.F)
      && (obj.getStationaryLocationsOnlyCounter() >= 2U) && obj.get_probIsCurrentlyMoving() < 0.2F
      && (obj.get_probHasBeenObservedMoving() < 0.2F) && (obj.getBadSensorBasedInnoCount() > 0U)};

   const vfc::uint8_t oncomingLocationThreshold = unreliableMovingVru ? 1U : 0U;

   // oncoming locations expected but not seen
   const bool areRadarUpdatesSuspicious{
      isMovingTowardsEgoLane(dyObj, vyObjRel, vyObjOverGround)
      && obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
            ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar)
            > lowerRadarConfirmationThreshold
      && dxObj < 50.F  // consistency: totalNumCyclesWithOncomingLocations active for dx < 45m in sensor FOR
      && (obj.getTotalNumCyclesWithOncomingLocations() <= oncomingLocationThreshold || (obj.getNumConsecutiveCyclesWithoutOncomingLocations() > 12U && obj.getTotalNumCyclesWithOncomingLocations() < 5U))};

   const bool isUnreliable =
      isCrossingFast && !wasMeasuredByFrontCorner && areVideoUpdatesSuspicious && areRadarUpdatesSuspicious;

   if (isUnreliable)
   {
      constexpr vfc::uint16_t numCyclesWithEnoughSensorUpdate{10U};
      const bool              isWellTracked =
         (obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
             ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo)
             > numCyclesWithEnoughSensorUpdate
          && obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
                ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar)
                > numCyclesWithEnoughSensorUpdate);

      if (hasBeenCreatedVeryRecently || unreliableMovingVru || isAssociationSuspicious)
      {
         PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
         obj.setSingleBitInPostProcessBitField(
            ::Per::Parameters::PerPostProcessTypes::applyUnreliableAngularVelocityCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
      }
      else if (!isWellTracked)
      {
         PostProcessing::disqualifyForVyDependentFunctions(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
         obj.setSingleBitInPostProcessBitField(
            ::Per::Parameters::PerPostProcessTypes::applyUnreliableAngularVelocityCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
      }
      else
      {
         // Do nothing
      }
   }
}

void Dc::Per::Dep::applyUpdatedWithStatLocWithHighMDopplerWithOutgoingVrCheck(
   const ::Per::InterfaceDep::DepObject&          obj,
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround,
   const bool                                     isObjectVru,
   vfc::uint16_t&                                 functionRelevanceBitField)
{
   if (
      isObjectVru && absVelOverGround[0] < 0.2F && absVelOverGround[1] > 1.F
      && vfc::abs(obj.getState().getY()) < 0.5F
      && obj.getIsUpdatedWithStatLocWithHighMDopplerWithOutgoingVr())
   {
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(
         ::Per::Parameters::PerPostProcessTypes::applyUpdatedWithStatLocWithHighMDopplerWithOutgoingVrCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
   }
}

void Dc::Per::Dep::applyIsMeasuredRatioCheckForFastWnj(
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround,
   const ::Per::InterfaceDep::DepObject&          obj,
   vfc::uint16_t&                                 functionRelevanceBitField)
{
   constexpr vfc::float32_t minSpeedForCrossingWNJ        = 4.6F;
   constexpr vfc::float32_t minRatioOfUpdatesToPrediction = 0.7F;
   constexpr vfc::uint16_t  minCyclesAlive                = 1U;
   constexpr vfc::uint16_t  maxCyclesAlive =
      static_cast<vfc::uint16_t>(vfc::numeric_limits<vfc::uint8_t>::max());
   constexpr vfc::uint16_t minCyclesWithVideoConfirmation = 5U;
   const vfc::uint16_t     objAliveCnt                    = obj.get_numCyclesExisting();

   VFC_STATIC_ASSERT2(minCyclesAlive > 0U, "Min cycle threshold cannot be zero to avoid div/0");

   // check if WNJ objects with high crossing speed
   // have been measured often enough to trust them for an activation
   // only apply check if object age is below 255, because number of radar updates is implemented in uint8
   if (
      obj.get_filterType() == ::Per::Parameters::PER_ALGOS_MBF_TYPES::TYPE_WNJ
      && vfc::abs(absVelOverGround[1]) > minSpeedForCrossingWNJ && (objAliveCnt < maxCyclesAlive))
   {
      if (objAliveCnt > minCyclesAlive)
      {
         const vfc::uint8_t numVideoUpdates = obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
            ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo);
         const vfc::uint8_t numRadarUpdates =
            obj.sensorFilterFusHelper.getTotalNumSensorUpdates(::Per::Parameters::PerSensorTechs::techRadar);

         const vfc::float32_t numRadarUpdatesFloat{static_cast<vfc::float32_t>(numRadarUpdates)};
         const vfc::float32_t objAliveCntFloat{static_cast<vfc::float32_t>(objAliveCnt) + 1.F};

         // calc ratio of radar update since object exists
         const vfc::float32_t ratioOfRadarUpdatesSinceCreation{
            vfc::divide(numRadarUpdatesFloat, objAliveCntFloat)};

         if (
            ratioOfRadarUpdatesSinceCreation < minRatioOfUpdatesToPrediction
            && numVideoUpdates <= minCyclesWithVideoConfirmation)
         {
            PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
            obj.setSingleBitInPostProcessBitField(
               ::Per::Parameters::PerPostProcessTypes::applyIsMeasuredRatioCheckForFastWnj);
#endif  // RB_DC_DEBUG_STATES_ENABLED
         }
      }
   }
}

void Dc::Per::Dep::applyWaterSprinklersCheckAcc(
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround, ::Per::InterfaceDep::DepObject& obj)
{
   constexpr vfc::float32_t drInnovationThreshold      = 1.2F;
   constexpr vfc::float32_t rcsThreshold               = -8.1F;
   constexpr vfc::float32_t groundReflexTreshold       = 0.8F;
   constexpr vfc::float32_t drInnovationRelaxThreshold = 0.78F;
   constexpr vfc::float32_t xVelAbsThreshold           = 4.0F;
   constexpr vfc::float32_t yVelAbsThreshold           = 0.7F;
   constexpr vfc::float32_t hasBeenObservedMovingLow   = 0.5F;
   constexpr vfc::float32_t isCurrentlyMoving          = 0.01F;

   const bool isRadarOnlyObject =
      obj.sensorFilterFusHelper.isOnlyUpdatedBySensorX(::Per::Parameters::PerSensorTechs::techRadar);
   const bool isDrInnovationExceeded       = vfc::abs(obj.getAvgDxInnovation()) > drInnovationThreshold;
   const bool isRcsTooLow                  = obj.get_rcs() < rcsThreshold;
   const bool isDrInnovationLittleExceeded = vfc::abs(obj.getAvgDxInnovation()) > drInnovationRelaxThreshold;
   const bool isGroundReflexExceeded       = obj.get_pNonObstacleRCSOnlyClassifier() > groundReflexTreshold;
   const bool isabsVelOverGroundLow =
      ((absVelOverGround[0] < xVelAbsThreshold) && (absVelOverGround[1] < yVelAbsThreshold));
   const ::Per::Parameters::PER_OBJECT_TYPES mostProbableType{
      obj.get_objectTypeTree().get_mostProbableConditionalType()};
   const bool isObjectOfInterest =
      ((mostProbableType == ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_UNKNOWN)
       || (mostProbableType == ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_OBSTACLE)
       || (mostProbableType == ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_OBSTACLE_MOBILE));


   if (isRadarOnlyObject && isRcsTooLow && isObjectOfInterest)
   {
      if (
         (isDrInnovationExceeded) || (isDrInnovationLittleExceeded && isGroundReflexExceeded)
         || (isabsVelOverGroundLow))
      {
         // The low moving probabilities will affect the ACC braking
         obj.set_probHasBeenObservedMoving(
            vfc::min(hasBeenObservedMovingLow, obj.get_probHasBeenObservedMoving()));
         obj.set_probIsCurrentlyMoving(vfc::min(isCurrentlyMoving, obj.get_probIsCurrentlyMoving()));
      }
   }
}


void Dc::Per::Dep::applyNonCrossingObjectCheck(
   const ::Per::InterfaceDep::DepObject&          obj,
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround,
   vfc::uint16_t&                                 functionRelevanceBitField)
{
   const bool appearsCrossing{absVelOverGround[1] > 0.5F};

   const bool isProbMovingLow{
      obj.get_probIsCurrentlyMoving() < 0.1F && obj.get_probHasBeenObservedMoving() < 0.1F};

   const bool hasBeenUpdatedByRadar{
      obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
         ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar)
      > 0U};
   const bool hasNoMicroDoppler{obj.getNumberMicroDopplerCycles() == 0U};
   const bool hasNoOncomingLocations{obj.getTotalNumCyclesWithOncomingLocations() == 0U};
   const bool isNotPerceivedAsMovingByRadar{
      hasBeenUpdatedByRadar && hasNoMicroDoppler && hasNoOncomingLocations};

   const bool appearsCrossingButIsProbablyNot{
      appearsCrossing && isProbMovingLow && isNotPerceivedAsMovingByRadar};
   if (appearsCrossingButIsProbablyNot)
   {
      PostProcessing::disqualifyForVyDependentFunctions(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(
         ::Per::Parameters::PerPostProcessTypes::applyNonCrossingObjectCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
   }
}

void Dc::Per::Dep::applyWaterSprinklesCheck(
   const ::Per::InterfaceDep::DepObject&          obj,
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround,
   const ::Per::Classifiers::ObjectTypeTree&      objectTypeTree,
   vfc::uint16_t&                                 functionRelevanceBitField)
{
   constexpr vfc::float32_t drInnovationThreshold      = 1.2F;
   constexpr vfc::float32_t rcsThreshold               = -7.5F;
   constexpr vfc::float32_t groundReflexTreshold       = 0.8F;
   constexpr vfc::float32_t drInnovationRelaxThreshold = 0.78F;
   constexpr vfc::float32_t xVelAbsThreshold           = 3.9F;
   constexpr vfc::float32_t yVelAbsThreshold           = 0.7F;
   constexpr vfc::float32_t dzThresholdforSMO          = 0.3F;

   constexpr vfc::uint8_t recentRadarOnlyObjectThreshold{40U};
   const bool isRecentRadarOnlyObject = obj.sensorFilterFusHelper.hasOnlyBeenUpdatedBySensorXInLastNCycles(
      ::Per::Parameters::PerSensorTechs::techRadar, recentRadarOnlyObjectThreshold);

   const bool notRecentlyUpdatedByVideo =
      (obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(::Per::Parameters::PerSensorTechs::techVideo)
       > 3U);
   // Check to see if camera contribution relative to radar contribution is insignificant, less than 5%.
   const bool hasObjectVeryLessCameraContribution =
      notRecentlyUpdatedByVideo
      && static_cast<vfc::float32_t>(obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
            ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo))
            < (0.05F
               * static_cast<vfc::float32_t>(obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
                  ::Per::Parameters::PerSensorTechs::techRadar)));
   const bool isDrInnovationExceeded       = vfc::abs(obj.getAvgDxInnovation()) > drInnovationThreshold;
   const bool isRcsTooLow                  = obj.get_rcs() < rcsThreshold;
   const bool isDrInnovationLittleExceeded = vfc::abs(obj.getAvgDxInnovation()) > drInnovationRelaxThreshold;
   const bool isGroundReflexExceeded       = obj.get_pNonObstacleRCSOnlyClassifier() > groundReflexTreshold;
   const bool isabsVelOverGroundLow =
      ((absVelOverGround[0] < xVelAbsThreshold) && (absVelOverGround[1] < yVelAbsThreshold));
   const ::Per::Parameters::PER_OBJECT_TYPES mostProbableType{
      objectTypeTree.get_mostProbableConditionalType()};
   const bool isObjectOfInterest =
      ((mostProbableType == ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_UNKNOWN)
       || (mostProbableType == ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_OBSTACLE)
       || (mostProbableType == ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_OBSTACLE_MOBILE));


   if (
      (isRecentRadarOnlyObject || hasObjectVeryLessCameraContribution)
      && (isRcsTooLow || obj.get_elevation() < dzThresholdforSMO) && isObjectOfInterest)
   {
      if (
         (isDrInnovationExceeded) || (isDrInnovationLittleExceeded && isGroundReflexExceeded)
         || (isabsVelOverGroundLow))
      {
         PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
         obj.setSingleBitInPostProcessBitField(
            ::Per::Parameters::PerPostProcessTypes::applyWaterSprinklesCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
      }
   }
}

void Dc::Per::Dep::applyIsMeasuredRatioCheckForRadarOnlyLongitudinallyMoving(
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround,
   const ::Per::InterfaceDep::DepObject&          obj,
   vfc::uint16_t&                                 functionRelevanceBitField)
{
   constexpr vfc::float32_t vxAbsThreshold = 2.F;
   constexpr vfc::float32_t dyThreshold    = 3.F;
   constexpr vfc::uint16_t  minCyclesAlive = 4U;
   constexpr vfc::uint16_t  maxCyclesAlive =
      static_cast<vfc::uint16_t>(vfc::numeric_limits<vfc::uint8_t>::max());
   const vfc::uint16_t objAliveCnt = obj.get_numCyclesExisting();
   const bool          isFrontCenterLocationRadarOnlyObject =
      obj.sensorFilterFusHelper.isOnlyUpdatedBySensorXIgnoringEnvModelSensor(
         ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar);
   const vfc::float32_t absDy = vfc::abs(obj.getState().getY());

   // only apply check if object age is below 255, because number of radar updates is implemented in uint8
   if (
      isFrontCenterLocationRadarOnlyObject && (absDy < dyThreshold) && (objAliveCnt > minCyclesAlive)
      && (objAliveCnt < maxCyclesAlive) && (absVelOverGround[0] > vxAbsThreshold))
   {
      constexpr vfc::float32_t minRatioOfUpdatesToAliveCount = 0.7F;
      const vfc::uint8_t       numRadarUpdates = obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
         ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar);
      // calc ratio of radar update since object exists
      const vfc::float32_t ratioOfRadarUpdatesSinceCreation = vfc::divide(
         static_cast<vfc::float32_t>(numRadarUpdates), (static_cast<vfc::float32_t>(objAliveCnt) + 1.F));

      if (ratioOfRadarUpdatesSinceCreation < minRatioOfUpdatesToAliveCount)
      {
         PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
         obj.setSingleBitInPostProcessBitField(::Per::Parameters::PerPostProcessTypes::
                                                  applyIsMeasuredRatioCheckForRadarOnlyLongitudinallyMoving);
#endif  // RB_DC_DEBUG_STATES_ENABLED
      }
   }
}

void Dc::Per::Dep::applyMicroDopplerCheck(
   const bool                                     isObjectVru,
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround,
   const ::Per::InterfaceDep::DepObject&          obj,
   const ::Per::Parameters::DepParameters&        params,
   vfc::uint16_t&                                 functionRelevanceBitField)
{

   // Check if object is Vru, but micro doppler never has been seen for an object (which is not video-only but
   // has not been observed by front corner object radars)
   if (
      params.getIsMicroDopplerCheckEnabled() && isObjectVru
      && (obj.getExpectedVrHighEnoughForMuDopplerCounter() >= 2U)
      && (obj.sensorFilterFusHelper.getTotalNumSensorUpdates(::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar) > 0U)
      && (obj.sensorFilterFusHelper.getTotalNumSensorUpdates(::Per::Parameters::ObstacleSensorIndex::frontLeftCornerObjectRadar) < 1U)
      && (obj.sensorFilterFusHelper.getTotalNumSensorUpdates(::Per::Parameters::ObstacleSensorIndex::frontRightCornerObjectRadar) < 1U)
      && (obj.getNumberMicroDopplerCycles() < params.getMinVruMicroDopplerCycles()))
   {
      // Fast crossing cyclists often do not show microDoppler. FP objects can also have similarly high
      // crossing velocities but are often much younger/less confirmed. Apply the microDoppler check also for
      // young quickly crossing objects but exclude older ones because their existence and their crossing
      // movement have been confirmed often by measurements and also they are less likely to be ghost objects
      // or wrongly tracked objects.
      constexpr vfc::uint16_t objectAgeThreshold = 12U;
      const bool              isObjectOld =
         (obj.get_numCyclesExisting() > objectAgeThreshold)
         && (obj.sensorFilterFusHelper.getTotalNumSensorUpdates(::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar) > 8U);
      const vfc::float32_t upperAbsVyThreshold = isObjectOld ? 3.2F : 99.F;

      constexpr vfc::float32_t lowerAbsSpeedThreshold = 0.5F;
      constexpr vfc::float32_t upperAbsVxThreshold    = 4.F;
      // Activate balcony only if object is not clearly moving above ground in x direction
      // (relevant for CMFTap scenario for 1R1V: video delivers an object of type 2Wheeler, radar
      // classification sets high pObstacle due to moving boost but radar does not measure microDoppler.
      // The object shall in this case not be disqualified.)
      const bool isCrossingVru = (absVelOverGround[1] > lowerAbsSpeedThreshold)
                                 && (absVelOverGround[1] < upperAbsVyThreshold)
                                 && (absVelOverGround[0] < upperAbsVxThreshold);
      // An object is considered moving laterally, if its absolute dy and absolute vy are above a
      // threshold.
      const bool areCrossingVruConditionsSatisfied{
         isCrossingVru && params.getIsMicroDopplerCheckOnCrossingVruApplied()};

      // check for stationary vrus whether radar did not confirm; note that this limitation is activated by a
      // parameter
      const bool isStationaryVru =
         (absVelOverGround[0] < lowerAbsSpeedThreshold) && (absVelOverGround[1] < lowerAbsSpeedThreshold);

      const bool areStationaryVruConditionsSatisfied{
         isStationaryVru && prm::PerParam::CCfg::get_isMicroDopplerCheckOnStationaryVruApplied()};

      // disqualify the object if the conditions for crossing or for stationary VRUs are satisfied
      if (areCrossingVruConditionsSatisfied || areStationaryVruConditionsSatisfied)
      {
         PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
         obj.setSingleBitInPostProcessBitField(
            ::Per::Parameters::PerPostProcessTypes::applyMicroDopplerCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
      }
      else
      {
         // No disqualification is applied
      }
   }
}

void Dc::Per::Dep::applyRadarOnlyRcsAndDrInnovationLimit(
   const ::Per::InterfaceDep::DepObject& obj, vfc::uint16_t& functionRelevanceBitField)
{
   constexpr vfc::float32_t drInnovationThreshold = 1.2F;
   constexpr vfc::float32_t rcsThreshold          = -15.F;

   const bool isFrontCenterRadarOnlyObject =
      obj.sensorFilterFusHelper.isOnlyUpdatedBySensorXIgnoringEnvModelSensor(
         ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar);
   const bool isDrInnovationExceeded = vfc::abs(obj.getAvgDxInnovation()) > drInnovationThreshold;
   const bool isRcsTooLow            = obj.get_rcs() < rcsThreshold;

   if (isFrontCenterRadarOnlyObject && isDrInnovationExceeded && isRcsTooLow)
   {
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(
         ::Per::Parameters::PerPostProcessTypes::applyRadarOnlyRcsAndDrInnovationLimit);
#endif  // RB_DC_DEBUG_STATES_ENABLED
   }
}

// Disqualify DEP objects for AEB in case of suspicious elevation
void Dc::Per::Dep::applyElevationCheck(
   const ::Per::InterfaceDep::DepObject&          obj,
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround,
   const bool                                     isObjectVru,
   const ::Per::Parameters::DepParameters&        params,
   vfc::uint16_t&                                 functionRelevanceBitField)
{
   // Gather object dx position to later check if dx is positive, as only objects in front are
   // in interest for the applied FP-countermeasure.
   const vfc::float32_t objDx = obj.getState().getX();

   // Default value is, that elevation is appropriate
   bool isDzInappropriate = false;

   // Check if non-VRU is a stationary object based on abs velocities (long & lat).
   constexpr vfc::float32_t stationaryVelocityThreshold = 1.F;  // m/s
   const bool               isObjectStationary          = (absVelOverGround[0] < stationaryVelocityThreshold)
                                   && (absVelOverGround[1] < stationaryVelocityThreshold);
   const bool hasBeenUpdatedByVideoRecently{
      obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
         ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo)
      < 10U};
   const bool isStationaryVideoConfirmedObject{isObjectStationary && hasBeenUpdatedByVideoRecently};

   // Preconditions for elevation check
   if (vfc::isPositive(objDx) && obj.elevation_isValid() && (isStationaryVideoConfirmedObject || isObjectVru))
   {
      // Check that elevation absolute value is within interpolated / extrapolated param range.
      // Therefore first get the allowed dz threshold
      const vfc::float32_t allowedDzThreshold =
         ::Per::Algos::InterpolatingFcts::linearInterpolateConstantExtrapolate(
            params.getElevationCheckDxLimits(), params.getElevationCheckDzThresholds(), objDx);

      // Then compare absolute value of object elevation (=dz) with allowed dz threshold to
      // determine if dz is too high (based on Dep Params).
      isDzInappropriate = obj.get_elevation() > allowedDzThreshold;
   }

   // If the elevation is not appropriate (=too high according to Dep Params)...
   if (isDzInappropriate)
   {
      // ... then set bit representing AEB to 0
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);

// Set PER-debugging flag for SELENA env
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(::Per::Parameters::PerPostProcessTypes::applyElevationCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
   }
}

void Dc::Per::Dep::applyNonPlausibleLocationChecks(
   const ::Per::InterfaceDep::DepObject&     obj,
   const ::Per::Classifiers::ObjectTypeTree& objectTypeTree,
   const bool&                               isMPC3Used,
   vfc::uint16_t&                            functionRelevanceBitField)
{
   if (
      objectTypeTree.get_mostProbableConditionalType()
         == ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_OBSTACLE_MOBILE_4PLUSWHEELER_TRUCK
      && !isMPC3Used)
   {
      const vfc::CSI::si_radian_f32_t yawAngle{obj.getYawAngle()};
      const bool                      isObjectOrientedStraight{
         (vfc::abs(Dc::Maths::wrapToPi2(yawAngle).value())
          < vfc::typedDegree2Radian<vfc::float32_t>() * 10.F)};
      const bool isNonPlausibleLocationCntHigh{obj.getNonPlausibleLocationCnt() > 4U};
      const bool isFilterTypeLa{obj.get_filterType() == ::Per::Parameters::PER_ALGOS_MBF_TYPES::TYPE_LA};
      // Is always true, since videoWExist is 0, in the case of no VideoUpdate
      const bool lowWExistVideo{
         obj.getWExistOfAssociatedVideoObject() < 0.9F
         && obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
               ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo)
               < 3U};

      if (isObjectOrientedStraight && isNonPlausibleLocationCntHigh && isFilterTypeLa && lowWExistVideo)
      {
         PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
         obj.setSingleBitInPostProcessBitField(
            ::Per::Parameters::PerPostProcessTypes::applyNonPlausibleLocationCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
      }
   }
}

void Dc::Per::Dep::applyFourpluswheelerChecks(
   const ::Per::InterfaceDep::DepObject&     obj,
   const ::Per::Frames::Rot1dAcc2dFor&       hvmFor,
   const ::Per::Parameters::DepParameters&   params,
   const ::Per::Classifiers::ObjectTypeTree& objectTypeTree,
   vfc::uint16_t&                            functionRelevanceBitField)
{
   // Adapt obstacle probabilities of 4-plus-wheeler with implausible velocities or implausible RCS.
   if (objectTypeTree.isOfTypeOrSubType4PlusWheeler())
   {
      // disqualify the object in case:
      // - object has received a lot of updates from radar and video
      // - object is slowly oncoming and in close range
      // - drInnovation is showing larger fluctuations
      constexpr vfc::uint32_t  requiredNumUpdates = 20U;
      constexpr vfc::float32_t lowerAbsVelThresh  = -2.5F;
      constexpr vfc::float32_t upperAbsVelThresh  = -0.5F;
      constexpr vfc::float32_t longDistThresh     = 10.0F;
      constexpr vfc::float32_t drInnovThresh      = 0.7F;

      const bool implausibleDxInnovation =
         (obj.sensorFilterFusHelper.getTotalNumSensorUpdates(::Per::Parameters::PerSensorTechs::techRadar)
          > requiredNumUpdates)
         && (obj.sensorFilterFusHelper.getTotalNumSensorUpdates(::Per::Parameters::PerSensorTechs::techVideo) > requiredNumUpdates)
         && (obj.getState().getVelocity(0U) + hvmFor.getVelocity(0U) >= lowerAbsVelThresh)
         && (obj.getState().getVelocity(0U) + hvmFor.getVelocity(0U) <= upperAbsVelThresh)
         && (obj.getState().getX() < longDistThresh) && (obj.getAvgDxInnovation() > drInnovThresh)
         && !obj.sensorFilterFusHelper.isGoodQualityFusedObject(
            ::Per::Parameters::PerSensorTechs::techRadar, ::Per::Parameters::PerSensorTechs::techVideo);

      // evaluate whether the object should be disqualified due to implausible RCS values -> we
      // suspect that we see a ghost object here
      constexpr vfc::uint8_t minCornerRadarUpdatesThresholdForPlausibleRcs = 11U;
      const bool             isLowestRcsThresholdCheck =
         ((obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
              ::Per::Parameters::PerSensorTechs::techVideo)
           < 2U)
          && (obj.elevation_isValid() && vfc::abs(obj.get_elevation()) < 2.5F)
          && (obj.getWExistOfAssociatedVideoObject() > 0.4F));
      const bool rcsThresholdLow = isLowestRcsThresholdCheck ?
                                      (obj.get_rcs() < (params.getImplausibleRcsThresh() - 8.5F)) :
                                      (obj.get_rcs() < params.getImplausibleRcsThresh());

      constexpr vfc::uint8_t maxCyclesThresholdSinceLastCornerSensorUpdate = 19U;

      // the front left corner radar allow for disqualification of the object in case:
      // - the object as been updated by it less than 11 times OR
      // - the last update was more than 19 cycles ago
      const bool numFrontLeftCornerRadarUpdatesIsSignificant{
         obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
            ::Per::Parameters::ObstacleSensorIndex::frontLeftCornerObjectRadar)
         < minCornerRadarUpdatesThresholdForPlausibleRcs};
      const bool numCyclesSinceLastFrontLeftCornerRadarUpdateIsSignificant{
         obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
            ::Per::Parameters::ObstacleSensorIndex::frontLeftCornerObjectRadar)
         > maxCyclesThresholdSinceLastCornerSensorUpdate};
      const bool frontLeftCornerRadarIsRelevant4ImplausibleRcs{
         numFrontLeftCornerRadarUpdatesIsSignificant
         || numCyclesSinceLastFrontLeftCornerRadarUpdateIsSignificant};

      // the front right corner radar updates allow for disqualification of the object in case:
      // - the object as been updated by it less than 11 times OR
      // - the last update was more than 19 cycles ago
      const bool numFrontRightCornerRadarUpdatesIsSignificant{
         obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
            ::Per::Parameters::ObstacleSensorIndex::frontRightCornerObjectRadar)
         < minCornerRadarUpdatesThresholdForPlausibleRcs};
      const bool numCyclesSinceLastFrontRightCornerRadarUpdateIsSignificant{
         obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
            ::Per::Parameters::ObstacleSensorIndex::frontRightCornerObjectRadar)
         > maxCyclesThresholdSinceLastCornerSensorUpdate};
      const bool frontRightCornerRadarIsRelevant4ImplausibleRcs{
         numFrontRightCornerRadarUpdatesIsSignificant
         || numCyclesSinceLastFrontRightCornerRadarUpdateIsSignificant};

      // implausible RCS values -> disqualify the object in case:
      // - the object is close enough (<20m)
      // - the rcs value of the object is lower than the given threshold (-9.5dBm^2)
      // - video has contributed to the update in last cycle
      // - front right and front left corner radar updates allow for disqualification of the object
      const bool disqualifyObjectDueToImplausibleRcsValues =
         (obj.getState().getX() < params.getMaxLongitudinalDistanceForRcsCountermeasure())
         && (rcsThresholdLow)
         && (obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(::Per::Parameters::PerSensorTechs::techVideo) < prm::PerParam::CCfg::get_maxCyclesSinceLastVideoUpdate())
         && frontLeftCornerRadarIsRelevant4ImplausibleRcs && frontRightCornerRadarIsRelevant4ImplausibleRcs;

      // disqualify the object in case:
      // - video has contributed to the update in current or last cycle
      // - the object updated by radar is untrusty at least since last 15 updates
      // - the probHBOMoving of object is lower than 0.1
      constexpr vfc::uint8_t   thresholdTrustworthyRadar                      = 15U;
      constexpr vfc::uint8_t   maxCyclesSinceLastVideoUpdate4TrustworthyCheck = 1U;
      constexpr vfc::float32_t standingObjectProb                             = 0.1F;
      const bool               untrustworthyObject =
         (obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
             ::Per::Parameters::PerSensorTechs::techVideo)
          <= maxCyclesSinceLastVideoUpdate4TrustworthyCheck)
         && !obj.sensorFilterFusHelper
                .isTrustworthyObject(::Per::Parameters::PerSensorTechs::techRadar, thresholdTrustworthyRadar)
         && (obj.get_probHasBeenObservedMoving() < standingObjectProb);

      const bool hasHighVr = obj.getExpectedVrHighEnoughForMuDopplerCounter() > 3U;
      const bool hasOnlyStationaryLocationsAssociatedForAtLeastTwoCycles =
         obj.getStationaryLocationsOnlyCounter() > 1U;
      const vfc::float32_t facingAngle = vfc::abs(obj.getFacingAngle().value());
      const bool           isSideways  = facingAngle > 45.F * vfc::typedDegree2Radian<vfc::float32_t>()
                              && facingAngle < 135.F * vfc::typedDegree2Radian<vfc::float32_t>();
      const bool hadNeverMicroDoppler = obj.getNumberMicroDopplerCycles() == 0U;
      const bool hasMultipleFrontLocationSensorUpdates =
         obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
            ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar)
         > 3U;
      // A car for which we should be able to measure some vr should not have stationary locations counter > 1
      // And additionally, if it's facing us sideways we should be able to measure micro doppler as well
      const bool implausibleCarWithStationaryLocationsAndNoMicrodopplerWhenFacingSideways =
         hasHighVr && hasOnlyStationaryLocationsAssociatedForAtLeastTwoCycles && isSideways
         && hadNeverMicroDoppler && hasMultipleFrontLocationSensorUpdates;
      if (implausibleDxInnovation || untrustworthyObject)
      {
         PostProcessing::disqualifyForAebAndAcc(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
         obj.setSingleBitInPostProcessBitField(
            ::Per::Parameters::PerPostProcessTypes::applyFourpluswheelerChecks);
#endif  // RB_DC_DEBUG_STATES_ENABLED
      }
      else if (
         disqualifyObjectDueToImplausibleRcsValues
         || implausibleCarWithStationaryLocationsAndNoMicrodopplerWhenFacingSideways)
      {
         PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
         obj.setSingleBitInPostProcessBitField(
            ::Per::Parameters::PerPostProcessTypes::applyFourpluswheelerChecks);
#endif  // RB_DC_DEBUG_STATES_ENABLED
      }
      else
      {
         // don't disqualify the object for any function
      }
   }
}

void Dc::Per::Dep::applySplitCheckDisqualifyObjectForFunctions(
   const bool                              isObjectVru,
   const ::Per::InterfaceDep::DepObject&   obj,
   const ::Per::Parameters::DepParameters& params,
   vfc::uint16_t&                          functionRelevanceBitField)
{
   // Handle wrong object inheritance/split scenarios (e.g. if a car stops on a manhole cover and starts
   // moving again)
   constexpr vfc::int32_t thresholdReductionStoppingSplit = 2;
   const bool             isSplitDetected = (obj.getSplitCounter() >= params.getSplitDetectionCntMaxVal())
                                || ((obj.getStoppingSplitCounter() >= params.getSplitDetectionCntMaxVal())
                                    && (obj.getSplitCounter() >= (params.getSplitDetectionCntMaxVal()
                                                                  - thresholdReductionStoppingSplit)));
   if ((!isObjectVru) && isSplitDetected)
   {
      PostProcessing::disqualifyForAebAndAcc(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(::Per::Parameters::PerPostProcessTypes::applySplitCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
   }
}

void Dc::Per::Dep::applyOrientationConsistencyCheck(
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround,
   const ::Per::Frames::Rot1dAcc2dFor&            hvmFor,
   const ::Per::InterfaceDep::DepObject&          obj,
   const ::Per::Classifiers::ObjectTypeTree&      objectTypeTree,
   const bool                                     isObjectVru,
   vfc::uint16_t&                                 functionRelevanceBitField)
{
   const vfc::float32_t objLength{obj.getDimension().getPosition(0U)};
   const bool           isTooLongForVru{objLength > 4.F};
   const vfc::float32_t objWidth{obj.getDimension().getPosition(1U)};
   const bool           isTooWideForVru{objWidth > 1.5F};

   const bool isNotVruByDimensions = isTooLongForVru && isTooWideForVru;

   const vfc::uint8_t missingVideoCyclesThreshold{
      static_cast<vfc::uint8_t>(obj.get_numCyclesExisting() > 15U ? 8U : 4U)};

   const bool wasNotUpdatedByVideoRecently =
      obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(::Per::Parameters::PerSensorTechs::techVideo)
      > missingVideoCyclesThreshold;
   const bool wasNotUpdatedByCorner =
      (obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
          ::Per::Parameters::ObstacleSensorIndex::frontLeftCornerObjectRadar)
       < 1U)
      && (obj.sensorFilterFusHelper.getTotalNumSensorUpdates(::Per::Parameters::ObstacleSensorIndex::frontRightCornerObjectRadar) < 1U);

   const bool hasNoStrongVruIndicatorsFromVideoAndCornerRadar{
      !isObjectVru && isNotVruByDimensions && wasNotUpdatedByVideoRecently && wasNotUpdatedByCorner};

   const bool isMotorcycle{
      obj.get_objectTypeTreeReadOnly().get_mostProbableConditionalType()
      == ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_OBSTACLE_MOBILE_2WHEELER_MOTORCYCLE};
   const bool isLA{obj.get_filterType() == ::Per::Parameters::PER_ALGOS_MBF_TYPES::TYPE_LA};

   if (
      objectTypeTree.isOfTypeOrSubType4PlusWheeler() || hasNoStrongVruIndicatorsFromVideoAndCornerRadar
      || (isMotorcycle && isLA))
   {
      constexpr vfc::float32_t absVelThreshold = 1.7F;
      const bool               isObjSpeedHighEnough =
         (absVelOverGround[0] > absVelThreshold) || (absVelOverGround[1] > absVelThreshold);
      const bool wasUpdatedByFrontRadar = obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
                                             ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar)
                                          > 0U;

      if (isObjSpeedHighEnough && wasUpdatedByFrontRadar)
      {
         const vfc::CRadian32 objYawAngle{obj.getYawAngle()};

         const vfc::linalg::TVectorN<vfc::float32_t, 2> velOverGround =
            ::Per::Algos::ObjectFunctions::calcVelocityOverGround(obj, hvmFor);
         const vfc::CRadian32 objVelAngle =
            vfc::atan2<vfc::TRadian, vfc::float32_t>(velOverGround[1], velOverGround[0]);
         const vfc::CRadian32 orientationAngleDifference = Dc::Maths::wrapToPi(objYawAngle - objVelAngle);
         const vfc::float32_t absAngleDelta              = vfc::abs(orientationAngleDifference.value());

         const bool areYawAngleAndVelocityDirectionInconsistent = absAngleDelta > ::Per::G_PI_4_32;

         if (areYawAngleAndVelocityDirectionInconsistent)
         {
#if defined(RB_DC_DEBUG_STATES_ENABLED)
            obj.setSingleBitInPostProcessBitField(
               ::Per::Parameters::PerPostProcessTypes::applyOrientationConsistencyCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
            PostProcessing::disqualifyForAeb(functionRelevanceBitField);
         }
      }
      // logic for slow objects: if orientation is not reliable, check if orientation is plausible compared
      // to video orientation
      else if (
         obj.getObjectOrientationUnreliableCount() > 2U && obj.getIsOrientationImplausibleCompared2Vid())
      {
#if defined(RB_DC_DEBUG_STATES_ENABLED)
         obj.setSingleBitInPostProcessBitField(
            ::Per::Parameters::PerPostProcessTypes::applyOrientationConsistencyCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
         PostProcessing::disqualifyForAeb(functionRelevanceBitField);
      }
      else
      {
         // do nothing
      }
   }
}

void Dc::Per::Dep::modifyUnreliableOrientationCount(
   const ::Per::Frames::Rot1dAcc2dFor& hvmFor, const bool isObjectVru, ::Per::InterfaceDep::DepObject& obj)
{
   if (!isObjectVru)
   {
      constexpr vfc::float32_t angleDtThreshold = 0.087266F;  // ~5 deg/s

      const bool noOrientUpdateForLong = obj.get_numCyclesNoOrientationUpdate() > 1U;
      const bool isEgoYawRateHigh      = vfc::abs(hvmFor.getAngleDt(0U)) > angleDtThreshold;

      if (noOrientUpdateForLong && isEgoYawRateHigh)
      {
         if (obj.getObjectOrientationUnreliableCount() < 15U)
         {
            obj.setObjectOrientationUnreliableCount(
               static_cast<vfc::uint8_t>(obj.getObjectOrientationUnreliableCount() + 1U));
         }
      }
      else
      {
         if (obj.getObjectOrientationUnreliableCount() > 0U)
         {
            obj.setObjectOrientationUnreliableCount(
               static_cast<vfc::uint8_t>(obj.getObjectOrientationUnreliableCount() - 1U));
         }
      }
   }
}

void Dc::Per::Dep::applyStationaryVruVideoGhostCheck(
   const ::Per::InterfaceDep::DepObject&          obj,
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround,
   const ::Per::Classifiers::ObjectTypeTree&      objectTypeTree,
   vfc::uint16_t&                                 functionRelevanceBitField)
{
   constexpr vfc::float32_t wExistThreshold{0.4F};
   constexpr vfc::float32_t rcsThresholdPed{4.F};
   constexpr vfc::float32_t rcsThresholdTwhoWheeler{6.5F};

   const ::Per::Parameters::PER_OBJECT_TYPES mostProbableType{
      objectTypeTree.get_mostProbableConditionalType()};
   const vfc::float32_t objRcs{obj.get_rcs()};
   bool                 isRcsHigh{false};
   if (
      (mostProbableType == ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_OBSTACLE_MOBILE_2WHEELER)
      || (mostProbableType == ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_OBSTACLE_MOBILE_2WHEELER_BICYCLE)
      || (mostProbableType == ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_OBSTACLE_MOBILE_2WHEELER_MOTORCYCLE))
   {
      isRcsHigh = objRcs > rcsThresholdTwhoWheeler;
   }
   else if (mostProbableType == ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_OBSTACLE_MOBILE_PEDESTRIAN)
   {
      isRcsHigh = objRcs > rcsThresholdPed;
   }
   else
   {
      // balcony only active for VRUs
   }
   const bool isAssociatedExistProbabilityLow{obj.getWExistOfAssociatedVideoObject() < wExistThreshold};
   const bool isFusedInCurrentCycle{
      vfc::isZero(obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
         ::Per::Parameters::PerSensorTechs::techRadar))
      && vfc::isZero(obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
         ::Per::Parameters::PerSensorTechs::techVideo))};
   const bool isStationryObject = absVelOverGround[0] < 0.5F && absVelOverGround[1] < 0.75F;

   if (isRcsHigh && isAssociatedExistProbabilityLow && isFusedInCurrentCycle && isStationryObject)
   {
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(
         ::Per::Parameters::PerPostProcessTypes::applyStationaryVruVideoGhostCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
   }
}

void Dc::Per::Dep::applyInnovationCheck(
   const ::Per::Frames::Rot1dAcc2dFor&     hvmFor,
   const ::Per::InterfaceDep::DepObject&   obj,
   const bool                              isVruObject,
   const ::Per::Parameters::DepParameters& params,
   vfc::uint16_t&                          functionRelevanceBitField)
{
   bool innovationRelevant =
      (vfc::abs(obj.getState().getPosition(0U)) < params.getInnovationCheckDxThreshold());
   innovationRelevant =
      innovationRelevant
      && (vfc::abs(obj.getState().getPosition(1U)) < params.getInnovationCheckDyThreshold());

   const vfc::float32_t absAvgInnovationDx    = vfc::abs(obj.getAvgDxInnovation());
   const vfc::float32_t dxInnovationThreshold = calcDxInnovationThreshold(hvmFor, obj, isVruObject);
   // If the innovation is relevant and the absolute average innovation is
   // over a given threshold, the object is disqualified for AEB
   if (innovationRelevant && (absAvgInnovationDx > dxInnovationThreshold))
   {
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(::Per::Parameters::PerPostProcessTypes::applyInnovationCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
   }
   else
   {
      // do nothing
   }
}

vfc::float32_t Dc::Per::Dep::calcDxInnovationThreshold(
   const ::Per::Frames::Rot1dAcc2dFor&   hvmFor,
   const ::Per::InterfaceDep::DepObject& obj,
   const bool                            isVruObject)
{
   constexpr vfc::float32_t dxInnovationThresholdDefault = 1.6F;
   bool                     isDxInnovationThresholdSet   = false;
   // determine dxInnovationThreshold dependent on object type, rcs-value, vyUnreliable, available fusion
   // and velocity
   vfc::float32_t dxInnovationThreshold = dxInnovationThresholdDefault;
   // coco begin validated: DC_00013 DECN "branch FALSE"
   if (!isDxInnovationThresholdSet)  // #COVDEV DC_00013 DECN "branch FALSE"
                                     // coco end
   {
      constexpr vfc::float32_t dxInnovationThresholdVru = 1.5F;
      constexpr vfc::float32_t dxThresholdVruCloseRange = 20.F;
      const vfc::float32_t     objAbsDx                 = vfc::abs(obj.getState().getPosition(0U));
      if ((objAbsDx < dxThresholdVruCloseRange) && isVruObject)
      {
         dxInnovationThreshold      = dxInnovationThresholdVru;
         isDxInnovationThresholdSet = true;
      }
      else
      {
         // do nothing
      }
   }

   if (!isDxInnovationThresholdSet)
   {
      constexpr vfc::float32_t dxInnovationThresholdIfVyUnreliable = 1.1F;
      constexpr vfc::float32_t rcsThresholdForVyUnreliableCheck    = -5.F;
      constexpr vfc::float32_t vyUnreliableThreshold               = 1.9F;
      // object that associates weak locations and has unreliable velocity jumps
      if (
         obj.get_rcs() < rcsThresholdForVyUnreliableCheck
         && obj.getVyUnreliableAccumulated() > vyUnreliableThreshold)
      {
         dxInnovationThreshold      = dxInnovationThresholdIfVyUnreliable;
         isDxInnovationThresholdSet = true;
      }
      else
      {
         // do nothing
      }
   }

   if (!isDxInnovationThresholdSet)
   {
      constexpr vfc::float32_t dxInnovationThresholdIfVeryLowRCSOnly = 1.5F;
      constexpr vfc::float32_t rcsThresholdForRcsOnlyCheck           = -15.F;
      // object that associates measuerements from the ground
      if (obj.get_rcs() < rcsThresholdForRcsOnlyCheck)
      {
         dxInnovationThreshold      = dxInnovationThresholdIfVeryLowRCSOnly;
         isDxInnovationThresholdSet = true;
      }
      else
      {
         // do nothing
      }
   }

   if (!isDxInnovationThresholdSet)
   {
      constexpr vfc::float32_t dxInnovationThresholdIfStandingFusedObj = 6.F;
      constexpr vfc::float32_t rcsThresholdForSlowGoodFusedObjectCheck = -10.F;
      const vfc::float32_t     nrContribution =
         0.8F
         * vfc::min(
            static_cast<vfc::float32_t>(vfc::numeric_limits<vfc::uint8_t>::max()),
            static_cast<vfc::float32_t>(obj.get_numCyclesExisting()));  // 80% of cycles
      constexpr vfc::uint16_t  minAgeOfGoodFusedObject = 5U;
      constexpr vfc::float32_t verySlowObjThreshold    = 1.F;
      const bool               isGoodFusedObject =
         obj.get_numCyclesExisting() > minAgeOfGoodFusedObject
         && obj.sensorFilterFusHelper.isGoodQualityFusedObject(
            ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar,
            ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo)
         && static_cast<vfc::float32_t>(obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
               ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar))
               >= nrContribution
         && static_cast<vfc::float32_t>(obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
               ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo))
               >= nrContribution;
      const bool isVerySlowObject =
         ::Per::Algos::ObjectFunctions::calcTotalAbsVelocityOverGround(obj, hvmFor) < verySlowObjThreshold;
      // very slow object with high rcs was confirmed by radar and video
      // (this can happen in case of slow object (standing car) that associates any locations reflected by
      // road boundary. The object remains at correct angle due to the video update)
      if (isGoodFusedObject && isVerySlowObject && obj.get_rcs() > rcsThresholdForSlowGoodFusedObjectCheck)
      {
         dxInnovationThreshold      = dxInnovationThresholdIfStandingFusedObj;
         isDxInnovationThresholdSet = true;
      }
      else
      {
         // do nothing
      }
   }

   if (!isDxInnovationThresholdSet)
   {
      constexpr vfc::float32_t dxInnovationThresholdIfVruLowRCSInCloseRange = 1.4F;
      constexpr vfc::float32_t rcsThreshold                                 = -5.F;
      constexpr vfc::float32_t dxThresholdVruCloseRange                     = 35.F;
      constexpr vfc::float32_t verySlowObjThreshold                         = 1.F;
      const vfc::float32_t     objAbsDx = vfc::abs(obj.getState().getPosition(0U));
      const bool               isRcsLow = obj.get_rcs() < rcsThreshold;
      const bool               isVerySlowObject =
         ::Per::Algos::ObjectFunctions::calcTotalAbsVelocityOverGround(obj, hvmFor) < verySlowObjThreshold;
      // Slow moving VRU in close range with low RCS
      if ((objAbsDx < dxThresholdVruCloseRange) && isVruObject && isRcsLow && isVerySlowObject)
      {
         dxInnovationThreshold      = dxInnovationThresholdIfVruLowRCSInCloseRange;
         isDxInnovationThresholdSet = true;
      }
      else
      {
         // do nothing
      }
   }

   return dxInnovationThreshold;
}


void Dc::Per::Dep::applyImplausibleVyVruCheck(
   const bool                                     isObjectVru,
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround,
   const ::Per::Frames::Rot1dAcc2dFor&            hvmFor,
   const ::Per::InterfaceDep::DepObject&          obj,
   const ::Per::Parameters::DepParameters&        params,
   vfc::uint16_t&                                 functionRelevanceBitField)
{
   // threshold for implausible vy in case of LA-model motion type
   const vfc::float32_t     implausibleVyThresholdForLAHypo = params.getImplausibleVyThreshLAHypo();
   constexpr vfc::float32_t turningEgoYawRateThreshold{10.0F * vfc::typedDegree2Radian<vfc::float32_t>()};
   const bool               isEgoTurning{vfc::abs(hvmFor.getAngleDt(0U)) > turningEgoYawRateThreshold};
   // If the object is VRU and its motion model type is LA and the object gets a high vy-velocity above
   // threshold, and ego vehicle is not turning,  disqualify the object for AEB and ACC
   if (
      isObjectVru && absVelOverGround[1] > implausibleVyThresholdForLAHypo
      && (obj.get_filterType() == ::Per::Parameters::PER_ALGOS_MBF_TYPES::TYPE_LA) && !isEgoTurning)
   {
      PostProcessing::disqualifyForAebAndAcc(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(
         ::Per::Parameters::PerPostProcessTypes::applyImplausibleVyVruCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
   }
}

void Dc::Per::Dep::applySensorBasedInnoCheck(
   const ::Per::InterfaceDep::DepObject&          obj,
   const bool                                     isObjectVru,
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround,
   const ::Per::Parameters::DepParameters& /*params*/,
   const ::Per::Classifiers::ObjectTypeTree& objectTypeTree,
   vfc::uint16_t&                            functionRelevanceBitField)
{
   constexpr vfc::float32_t maxVelocityOverGroundThreshold{0.98F};
   vfc::float32_t           vyAbsThreshold{0.F};
   vyAbsThreshold                = obj.getBadSensorBasedInnoCount() >= 5U ? 4.4F : 3.F;
   const vfc::float32_t objectDr = obj.getState().getPosition(0U);

   // prevent division by 0 (only consider objects with more than 10cm distance)
   if (objectDr > 0.1F)
   {

      // do not apply the check on crossing cars: empirically it was observed that fast crossing cars
      // have often high innovations and it is very hard to track these objects without high innovations.
      // Therefore a high innovation for a crossing car is not a good indicator for an implausible object.
      const auto isCrossingCar = (objectTypeTree.isOfTypeOrSubType4PlusWheeler())
                                 && (absVelOverGround[0] < maxVelocityOverGroundThreshold)
                                 && (absVelOverGround[1] > vyAbsThreshold);
      // apply the actual limit if necessary
      if (obj.getBadSensorBasedInnoCount() >= 2U && isObjectVru)
      {
         PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
         obj.setSingleBitInPostProcessBitField(
            ::Per::Parameters::PerPostProcessTypes::applySensorBasedInnoCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
      }
      else if (obj.getBadSensorBasedInnoCount() >= 2U && !isCrossingCar)
      {
         PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
         obj.setSingleBitInPostProcessBitField(
            ::Per::Parameters::PerPostProcessTypes::applySensorBasedInnoCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
      }
      else
      {
         // do nothing
      }
   }
}

void Dc::Per::Dep::applyImplausibleVideoTtcForVru(
   const bool isVru, const ::Per::InterfaceDep::DepObject& obj, vfc::uint16_t& functionRelevanceBitField)
{
   if (
      isVru
      && obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
            ::Per::Parameters::PerSensorTechs::techVideo)
            < 1U
      && vfc::isEqual(
         obj.getVideoInvTtc(),
         vfc::numeric_limits<vfc::float32_t>::max(),
         vfc::numeric_limits<vfc::float32_t>::epsilon()))
   {
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(
         ::Per::Parameters::PerPostProcessTypes::applyImplausibleVideoTtcForVru);
#endif  // RB_DC_DEBUG_STATES_ENABLED
   }
}

void Dc::Per::Dep::applyVyInconsistentCheck(
   const ::Per::InterfaceDep::DepObject& obj,
   const ::Per::Frames::Rot1dAcc2dFor&   hvmFor,
   const bool                            isVru,
   vfc::uint16_t&                        functionRelevanceBitField)
{
   constexpr vfc::uint8_t   vyInconsistentThreshold = 3U;
   constexpr vfc::float32_t dyThreshold             = 1.2F;
   constexpr vfc::float32_t angleDtThreshold        = 0.0262F;  // ~1.5 deg/s

   constexpr vfc::float32_t vyUnreliableAccumulatedThreshold = 2.6F;

   if (
      ((obj.getVyInconsistent() > vyInconsistentThreshold)
       && (obj.get_filterType() == ::Per::Parameters::PER_ALGOS_MBF_TYPES::TYPE_LA) && isVru
       && vfc::abs(obj.getState().getPosition(1U)) > dyThreshold
       && vfc::abs(hvmFor.getAngleDt(0U)) < angleDtThreshold)
      || (obj.getVyUnreliableAccumulated() > vyUnreliableAccumulatedThreshold && isVru && obj.get_filterType() == ::Per::Parameters::PER_ALGOS_MBF_TYPES::TYPE_WNJ))
   {
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(::Per::Parameters::PerPostProcessTypes::applyVyInconsistentCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
   }
}

void Dc::Per::Dep::applyVideoHandleSharedCheck(
   const ::Per::Frames::Rot1dAcc2dFor&            hvmFor,
   const bool                                     isObjectVru,
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround,
   const ::Per::InterfaceDep::DepObject&          obj,
   const ::Per::InterfaceDep::DepCollection&      depCollection,
   vfc::uint16_t&                                 functionRelevanceBitField)
{
   constexpr vfc::float32_t statThresh = 0.5F;
   constexpr vfc::float32_t hbomThresh = 0.9F;

   // first check all attributes of obj in order only to iterate over dep list if needed
   if (
      isObjectVru && (absVelOverGround[0] < statThresh) && (absVelOverGround[1] < statThresh)
      && (obj.get_probHasBeenObservedMoving() > hbomThresh)
      && obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
            ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo)
            > 1U)
   {
      for (auto& objOther : depCollection.get_constCollection())
      {
         // make sure that object is not compared to itself
         if (
            (objOther.get10bitObjectId() != obj.get10bitObjectId())
            && objOther.isRecentlyUsedVideoMeasurementHandleValid()
            && obj.isRecentlyUsedVideoMeasurementHandleValid()
            && (objOther.getRecentlyUsedVideoMeasurementHandle() == obj.getRecentlyUsedVideoMeasurementHandle()))
         {
            constexpr vfc::float32_t                       movingThresh = 2.F;
            const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGroundOther =
               ::Per::Algos::ObjectFunctions::calcAbsVelocityOverGround(objOther, hvmFor);

            if (
               (absVelOverGroundOther[0] > movingThresh)
               && (objOther.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
                      ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo)
                   < obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
                        ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo)))
            {
               PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
               obj.setSingleBitInPostProcessBitField(
                  ::Per::Parameters::PerPostProcessTypes::applyVideoHandleSharedCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
            }
         }
      }
   }
}

void Dc::Per::Dep::modifyBadSensorBasedInnoCount(
   ::Per::InterfaceDep::DepObject&                 obj,
   const vfc::linalg::TVectorN<vfc::float32_t, 2>& absVelOverGround,
   const bool                                      isMPC3Used)
{
   const auto ObjCenter =
      obj.getReferencePointPosition(::Per::Parameters::PER_REFERENCE_POINTS::REF_POS_CENTER);

   // the angle innovations are compared to a threshold that scales with 1/radius. One should actually use
   // the distance of point on the object that was updated to the updating sensor. However here the objects
   // are in the ego frame of reference. For close range objects the difference between sensor FOR and
   // ego FOR is quite important. As we do not want to access the exact sensor mountings here,
   // we deliberately over-compensate and take into account that in close range the check is less
   // sensitive. In far distances, the correction does not really matter.
   constexpr vfc::float32_t overCompensatedMounting = Dc::Per::PostProcessing::getOverCompensatedMounting();
   const vfc::float32_t     distanceToCenter = vfc::linalg::norm_L2(ObjCenter) - overCompensatedMounting;

   if (distanceToCenter > 0.1F)
   {
      const auto radarBasedInnovation = obj.getRadarBasedInnovation();
      const auto videoBasedInnovation = obj.getVideoBasedInnovation();

      // norming of radar dr innovation: 2m (due to empirical evaluations)
      constexpr vfc::float32_t normInnoRadarDr = 2.0F;
      // norming of radar alpha innovation: dy extent of object, orthogonal to viewing direction
      // transformed to angle (based on distance dr).
      const vfc::float32_t estimatedDyExtent = ::Per::Algos::ObjectFunctions::calcDyExtent(obj);

      // increase radar alpha innovation for stationary objects
      constexpr vfc::float32_t velThreshold = 0.5F;
      const bool isStationary = absVelOverGround[0] < velThreshold && absVelOverGround[1] < velThreshold;
      const vfc::float32_t normFactorInnoRadarAlpha = isStationary ? 0.5F : 1.F;

      const vfc::float32_t normInnoRadarAlpha = vfc::max(
         normFactorInnoRadarAlpha
            * vfc::atan<vfc::TRadian, vfc::float32_t>(estimatedDyExtent / distanceToCenter).value(),
         2.F * vfc::typedDegree2Radian<vfc::float32_t>());

      // norming of video alpha innovation: 1m accuracy, orthogonal to viewing direction
      // transformed to angle (based on distance dr).
      const vfc::float32_t normInnoVideoAlpha =
         vfc::atan<vfc::TRadian, vfc::float32_t>(1.F / distanceToCenter).value();
      // Consider video dr innovation for stationary objects that are closer than 20m
      const vfc::float32_t normFactorInnoVideoDr = isStationary && (distanceToCenter < 20.F) ? 0.6F : 1000.F;
      const vfc::float32_t normInnoVideoDr       = normFactorInnoVideoDr * distanceToCenter;

      vfc::float32_t innoResult = vfc::abs(radarBasedInnovation[0] / normInnoRadarDr)
                                  + vfc::abs(radarBasedInnovation[1] / normInnoRadarAlpha)
                                  + vfc::abs(videoBasedInnovation[1] / normInnoVideoAlpha);
      if (!isMPC3Used)  // Currently MPC3 radar dr innovations is higher than expected so it would not be
                        // worth to consider dr innovations as it could push us over the badInnoUpdates
                        // threshold
      {
         innoResult += vfc::abs(videoBasedInnovation[0] / normInnoVideoDr);
      }

      constexpr vfc::uint8_t numCyclesRecentContribution{3U};

      const bool hasRadarRecentlyContributed =
         obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
            ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar)
         <= numCyclesRecentContribution;
      const bool hasVideoRecentlyContributed = obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
                                                  ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo)
                                               <= numCyclesRecentContribution;


      innoResult =
         (hasRadarRecentlyContributed && hasVideoRecentlyContributed) ? innoResult : innoResult * 2.F;
      // update the counter for bad sensor based innovation
      obj.updateBadSensorBasedInnoCount(innoResult, isMPC3Used);
   }
}

void Dc::Per::Dep::applyRadarOnlyNLDCheck(
   const ::Per::Frames::Rot1dAcc2dFor&   hvmFor,
   const ::Per::InterfaceDep::DepObject& obj,
   vfc::uint16_t&                        functionRelevanceBitField)
{

   // apply this postProc measure only in case the object is radar only
   if (0U == obj.sensorFilterFusHelper.getTotalNumSensorUpdates(::Per::Parameters::PerSensorTechs::techVideo))
   {
      // Radar only object is not a NLD candidate if:
      // 1. ego is driving straight Or (small ay_ego AND small yawRate_ego)
      // AND
      // 2. (dx < 120m And dy <= 1.25m) OR (dx < 10m And dy <= 6m)
      // AND
      // 3. the object is existing at least 3 cycles
      // AND
      // 4. the object is continuously measured by radar OR existing for long
      // cycles (30 cycles)

      // init parameter thresholds to check ego and target against:
      constexpr vfc::float32_t accelerationThreshold{0.15F};
      constexpr vfc::float32_t angleDtThreshold{0.012F};
      constexpr vfc::float32_t lateralPositionThresholdFar{1.25F};
      constexpr vfc::float32_t longitudinalPositionThresholdFar{120.F};
      constexpr vfc::float32_t lateralPositionThresholdClose{6.0F};
      constexpr vfc::float32_t longitudinalPositionThresholdClose{10.F};
      constexpr vfc::uint16_t  numCyclesExistingThreshold{3U};
      constexpr vfc::uint16_t  numLongCyclesExistingThreshold{30U};
      constexpr vfc::float32_t egoDrivingStraight{2500.F};

      // init necessary variables for ego and object
      const vfc::float32_t vEgo{hvmFor.getVelocity(0U)};
      const vfc::float32_t ayEgo{hvmFor.getAcceleration(1U)};
      const vfc::float32_t yawRate{hvmFor.getAngleDt(0U)};
      const vfc::float32_t dxObj{obj.getState().getPosition(0U)};
      const vfc::float32_t dyObj{obj.getState().getPosition(1U)};
      const vfc::float32_t vyObj{obj.getState().getVelocity(1U)};
      const vfc::uint16_t  objExistCycles{obj.get_numCyclesExisting()};

      // check if ego is driving in straight driving situation
      bool isEgoDrivingStraight{true};
      if (vfc::notZero(yawRate))
      {
         const vfc::float32_t egoRadius{vEgo / yawRate};
         isEgoDrivingStraight =
            ((vfc::abs(egoRadius) > egoDrivingStraight)
             || ((vfc::abs(ayEgo) < accelerationThreshold) && (yawRate < angleDtThreshold)));
      }

      // check if object is inside relevant area
      const bool isObjectInRelevantArea{(
         ((vfc::abs(dyObj) <= lateralPositionThresholdFar) && (dxObj < longitudinalPositionThresholdFar))
         || ((vfc::abs(dyObj) <= lateralPositionThresholdClose) && (dxObj < longitudinalPositionThresholdClose)))};

      // check if object is old enough
      const bool isObjectOldEnough{(objExistCycles >= numCyclesExistingThreshold)};

      // check if object is measured sufficiently
      const bool isObjectMeasuredSufficiently{
         (obj.sensorFilterFusHelper.getTotalNumSensorUpdates(::Per::Parameters::PerSensorTechs::techRadar)
          >= objExistCycles)
         || (objExistCycles >= numLongCyclesExistingThreshold)};

      // if at least one condition is not met, then the object is a NLD candidate
      const bool isRadarOnlyNldCandidate{
         (!isEgoDrivingStraight || !isObjectInRelevantArea || !isObjectOldEnough
          || !isObjectMeasuredSufficiently)};

      // Check if object is close to ego (dx < 8m AND dy < 4m)
      // with a high lateral velocity (vy > 3m/s)
      constexpr vfc::float32_t highLateralVelocityThreshold  = 3.F;
      constexpr vfc::float32_t longitudinalPositionThreshold = 8.F;
      constexpr vfc::float32_t lateralPositionThreshold      = 4.F;

      const bool isObjectCloseWithHighLateralVelocity{
         (vfc::abs(vyObj) > highLateralVelocityThreshold) && (dxObj < longitudinalPositionThreshold)
         && (vfc::abs(dyObj) < lateralPositionThreshold)};

      if (isRadarOnlyNldCandidate || isObjectCloseWithHighLateralVelocity)
      {
         PostProcessing::disqualifyForAebAndAcc(functionRelevanceBitField);

#if defined(RB_DC_DEBUG_STATES_ENABLED)
         obj.setSingleBitInPostProcessBitField(
            ::Per::Parameters::PerPostProcessTypes::applyRadarOnlyNLDCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
      }
   }
}

void Dc::Per::Dep::applyRadarOnlyStationaryCheck(
   const vfc::linalg::TVectorN<vfc::float32_t, 2>& absVelOverGround,
   const ::Per::InterfaceDep::DepObject&           obj,
   vfc::uint16_t&                                  functionRelevanceBitField)
{
   constexpr vfc::float32_t absVelocityThreshold{0.3F};  // m/s

   // apply this postProc measure only in case the object is radar only
   if (0U == obj.sensorFilterFusHelper.getTotalNumSensorUpdates(::Per::Parameters::PerSensorTechs::techVideo))
   {

      // Check if the object is considered as a stationary object based on absolute velocity over ground
      if ((absVelOverGround[0] < absVelocityThreshold) && (absVelOverGround[1] < absVelocityThreshold))
      {
         PostProcessing::disqualifyForAebAndAcc(functionRelevanceBitField);

#if defined(RB_DC_DEBUG_STATES_ENABLED)
         obj.setSingleBitInPostProcessBitField(
            ::Per::Parameters::PerPostProcessTypes::applyRadarOnlyNLDCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
      }
   }
}

void Dc::Per::Dep::applyIsMeasuredRatioCheckForStandingLongiVru(
   const ::Per::InterfaceDep::DepObject& obj,
   const bool                            isObjectVru,
   vfc::uint16_t&                        functionRelevanceBitField)
{
   constexpr vfc::float32_t minProbHasBeenObsMoving = 0.95F;
   constexpr vfc::float32_t maxProbMoving           = 0.15F;
   constexpr vfc::uint16_t  minObjectAlive          = 25U;
   constexpr vfc::uint8_t   numOfRelevantCycles     = 8U;
   const vfc::uint8_t numOfNonUpdates{obj.m_sensorMeasuredHistory.numberOfSingleUpdateBySensorXOrNonUpdate(
      ::Per::Parameters::ObstacleSensorIndex::invalidObstacleSensorIndex, numOfRelevantCycles)};
   const bool         isObjBadlyMeasured{
      numOfRelevantCycles <= 2U * numOfNonUpdates};  // no update at least in half of the last cycles
   const bool isObjectOldEnough{obj.get_numCyclesExisting() >= minObjectAlive};

   if (
      (obj.get_filterType() == ::Per::Parameters::PER_ALGOS_MBF_TYPES::TYPE_LA)
      && (obj.get_probHasBeenObservedMoving() > minProbHasBeenObsMoving)
      && (obj.get_probIsCurrentlyMoving() < maxProbMoving) && isObjectOldEnough && isObjectVru
      && isObjBadlyMeasured)
   {
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(
         ::Per::Parameters::PerPostProcessTypes::applyIsMeasuredRatioCheckForStandingLongiVru);
#endif  // RB_DC_DEBUG_STATES_ENABLED
   }
}

void Dc::Per::Dep::applyImplausiblyAcceleratingVruCheck(
   const ::Per::InterfaceDep::DepObject& obj,
   const bool                            isObjectVru,
   const ::Per::Frames::Rot1dAcc2dFor&   hvmFor,
   vfc::uint16_t&                        functionRelevanceBitField)
{
   constexpr vfc::float32_t dxThreshold           = 35.0F;  // only work in close range
   constexpr vfc::float32_t minAbsVelOverGround   = 4.5F;   // abs vel is already high
   constexpr vfc::float32_t forwardAccelThreshold = 3.0F;   // high acceleration in the direction of movement
   constexpr vfc::uint16_t  objectAgeThreshold    = 10U;    // only for objects of age less than 10 cycles

   const bool isObjClose{obj.getState().getX() < dxThreshold};
   const bool isObjYoung{obj.get_numCyclesExisting() < objectAgeThreshold};

   const auto           stateOverGround  = obj.getCurrentStateInAbsoluteCoordinates(hvmFor);
   const vfc::float32_t absVelOverGround = vfc::sqrt(
      vfc::sqr(stateOverGround.get_vectorEntry(2U)) + vfc::sqr(stateOverGround.get_vectorEntry(3U)));

   if (absVelOverGround < minAbsVelOverGround)
   {
      // early return if velocity over ground is too slow
      // also prevent div by zero
      return;
   }

   const vfc::float32_t forwardAccel = vfc::divide(
      stateOverGround.get_vectorEntry(2U) * stateOverGround.get_vectorEntry(4U)
         + stateOverGround.get_vectorEntry(3U) * stateOverGround.get_vectorEntry(5U),
      absVelOverGround);

   if (isObjClose && isObjYoung && isObjectVru && (forwardAccel > forwardAccelThreshold))
   {
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(
         ::Per::Parameters::PerPostProcessTypes::applyImplausiblyAcceleratingVruCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
   }
}

void Dc::Per::Dep::applyUndefinedCrossingVruFromCorner(
   const ::Per::InterfaceDep::DepObject&          obj,
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround,
   vfc::uint16_t&                                 functionRelevanceBitField)
{
   const bool isObjMostlyCornerOnly{
      ((obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
           ::Per::Parameters::ObstacleSensorIndex::frontLeftCornerObjectRadar)
        >= 10U)
       || (obj.sensorFilterFusHelper.getTotalNumSensorUpdates(::Per::Parameters::ObstacleSensorIndex::frontRightCornerObjectRadar) >= 10U))
      && (obj.sensorFilterFusHelper.getTotalNumSensorUpdates(::Per::Parameters::ObstacleSensorIndex::frontCenterVideo) <= 1U)
      && (obj.sensorFilterFusHelper.getTotalNumSensorUpdates(::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar) <= 1U)};

   const bool isObjClose{obj.getState().getPosition(0U) < 10.0F};
   const bool isObjYoung{obj.get_numCyclesExisting() <= 15U};
   const bool hasMotionModelWnj{obj.get_filterType() == ::Per::Parameters::PER_ALGOS_MBF_TYPES::TYPE_WNJ};
   const bool movingLaterallyWithLowProb{
      obj.get_probHasBeenObservedMoving() < 0.3F && absVelOverGround[1] > 3.0F};

   const vfc::float32_t probPedestrian{obj.get_objectTypeTreeReadOnly().get_condProbOfType(
      ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_OBSTACLE_MOBILE_PEDESTRIAN)};
   const vfc::float32_t prob2Wheeler{obj.get_objectTypeTreeReadOnly().get_condProbOfType(
      ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_OBSTACLE_MOBILE_2WHEELER)};
   const bool           isVruTypeUndecided{
      (probPedestrian > 0.3F) && (prob2Wheeler > 0.3F) && ((probPedestrian + prob2Wheeler) > 0.8F)};

   if (
      isObjMostlyCornerOnly && isObjClose && isObjYoung && hasMotionModelWnj && movingLaterallyWithLowProb
      && isVruTypeUndecided)
   {
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(
         ::Per::Parameters::PerPostProcessTypes::applyUndefinedCrossingVruFromCorner);
#endif  // RB_DC_DEBUG_STATES_ENABLED
   }
}

void Dc::Per::Dep::applyImplausiblePedCheck(
   const ::Per::InterfaceDep::DepObject&          obj,
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround,
   vfc::uint16_t&                                 functionRelevanceBitField)
{
   constexpr vfc::float32_t rcsThresholdPed{3.5F};
   constexpr vfc::float32_t rcsSuspiciousThresholdPed{-14.F};
   constexpr vfc::float32_t minVxThreshold{2.0F};
   constexpr vfc::uint8_t   maxNumberMicroDopplerCycles{2U};
   constexpr vfc::float32_t maxAbsDy{0.5F};
   constexpr vfc::uint8_t   youngObjThreshold_1{10U};
   constexpr vfc::uint8_t   youngObjThreshold_2{15U};
   constexpr vfc::float32_t radarAngleInnoThreshold{5.0F};
   constexpr vfc::float32_t videoAngleInnoThreshold{2.0F};
   constexpr vfc::float32_t dzThreshold{0.1F};

   const vfc::float32_t objDy{obj.getState().getY()};

   const bool hasHighVx{absVelOverGround[0] > minVxThreshold};
   const bool isPedestrian{
      obj.get_objectTypeTreeReadOnly().get_mostProbableConditionalType()
      == ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_OBSTACLE_MOBILE_PEDESTRIAN};
   const bool hasLowMicroDopplerCycles{obj.getNumberMicroDopplerCycles() <= maxNumberMicroDopplerCycles};
   const bool isRcsHighForPed{(obj.get_rcs() > rcsThresholdPed)};
   const bool isRcsSuspiciouslyLow{obj.get_rcs() < rcsSuspiciousThresholdPed};
   const bool isInEgoLane{vfc::abs(objDy) < maxAbsDy};
   const bool lowNumberOfRadarUpdatesForRCS{
      obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
         ::Per::Parameters::ObstacleSensorIndex::frontLeftCornerObjectRadar)
         < 10U
      && obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
            ::Per::Parameters::ObstacleSensorIndex::frontRightCornerObjectRadar)
            < 10U
      && obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
            ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar)
            < 3U};
   const bool highStationaryLocNum{obj.getStationaryLocationsOnlyCounter() >= 4};
   const bool hasNoMicroDoppler{obj.getNumberMicroDopplerCycles() == 0U};
   const bool isYoungObject_1 = obj.get_numCyclesExisting() < youngObjThreshold_1;
   const bool isYoungObject_2 = obj.get_numCyclesExisting() < youngObjThreshold_2;
   const bool hasBeenMeasuredByFrontCornerRadar =
      obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
         ::Per::Parameters::ObstacleSensorIndex::frontLeftCornerObjectRadar)
         > 0U
      || obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
            ::Per::Parameters::ObstacleSensorIndex::frontRightCornerObjectRadar)
            > 0U;
   const bool isRadarAngleInnoHigh{
      vfc::abs(obj.getRadarBasedInnovation()[1] * vfc::typedRadian2Degree<vfc::float32_t>())
      > radarAngleInnoThreshold};
   const bool isVideoAngleInnoHigh{
      vfc::abs(obj.getVideoBasedInnovation()[1] * vfc::typedRadian2Degree<vfc::float32_t>())
      > videoAngleInnoThreshold};
   const bool isElevationTooLow{obj.elevation_isValid() && obj.get_elevation() < dzThreshold};

   if (
      isPedestrian
      && (((isRcsHighForPed || isRcsSuspiciouslyLow) && hasHighVx && hasLowMicroDopplerCycles) || (!isInEgoLane && hasNoMicroDoppler && highStationaryLocNum && lowNumberOfRadarUpdatesForRCS && isYoungObject_1 && hasBeenMeasuredByFrontCornerRadar) || (highStationaryLocNum && isRadarAngleInnoHigh && isVideoAngleInnoHigh && isElevationTooLow && isYoungObject_2)))
   {
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(::Per::Parameters::PerPostProcessTypes::applyImplausiblePedCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);
   }
}

void Dc::Per::Dep::applyImplausiblePedCheckLRR(
   const ::Per::InterfaceDep::DepObject&          obj,
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround,
   vfc::uint16_t&                                 functionRelevanceBitField)
{
   constexpr vfc::float32_t rcsThresholdPed{0.0F};
   constexpr vfc::uint16_t  statLocThreshold{5U};
   constexpr vfc::float32_t drVideoInnoThreshold{6.0F};
   constexpr vfc::float32_t maxAbsDy{0.5F};
   constexpr vfc::float32_t minVxThreshold{2.0F};
   constexpr vfc::float32_t maxVyThreshold{0.4F};
   constexpr vfc::uint8_t   maxNumberMicroDopplerCycles{2U};
   constexpr vfc::uint8_t   youngObjThreshold{10U};

   const vfc::float32_t objDy{obj.getState().getY()};

   const bool isInEgoLane{vfc::abs(objDy) < maxAbsDy};
   const bool hasHighVx{absVelOverGround[0] > minVxThreshold};
   const bool hasLowVy{absVelOverGround[1] < maxVyThreshold};
   const bool objectHasBeenObservedByVideo{
      obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
         ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo)
      == 0U};
   const bool isPedestrian{
      obj.get_objectTypeTreeReadOnly().get_mostProbableConditionalType()
      == ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_OBSTACLE_MOBILE_PEDESTRIAN};
   const bool hasLowMicroDopplerCycles{obj.getNumberMicroDopplerCycles() <= maxNumberMicroDopplerCycles};
   const bool isRcsHighForPed{(obj.get_rcs() > rcsThresholdPed)};
   const bool hasBeenMeasuredByfrontCenterLocationRadar =
      obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
         ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar)
      > 0U;
   const bool hasNoMicroDoppler{
      obj.getNumberMicroDopplerCycles() == 0U && obj.getExpectedVrHighEnoughForMuDopplerCounter() > 0U
      && hasBeenMeasuredByfrontCenterLocationRadar};
   const bool hasHighStatLocCount = obj.getStationaryLocationsOnlyCounter() > statLocThreshold;
   const bool isVideoDrInnoHigh   = (vfc::abs(obj.getVideoBasedInnovation()[0U]) > drVideoInnoThreshold);
   const bool hasBeenUpdatedByAnySensor{obj.sensorFilterFusHelper.getUpdatesSinceLastUpdate() == 0};
   const bool isYoungObject = obj.get_numCyclesExisting() < youngObjThreshold;
   const bool hasBeenMeasuredByFrontCornerRadar =
      obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
         ::Per::Parameters::ObstacleSensorIndex::frontLeftCornerObjectRadar)
         > 0U
      || obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
            ::Per::Parameters::ObstacleSensorIndex::frontRightCornerObjectRadar)
            > 0U;

   if (
      isPedestrian
      && ((isInEgoLane && hasLowVy && objectHasBeenObservedByVideo && hasHighVx && hasLowMicroDopplerCycles) || (hasHighStatLocCount && (isVideoDrInnoHigh || !hasBeenUpdatedByAnySensor)) || (hasNoMicroDoppler && isRcsHighForPed && isYoungObject && hasBeenMeasuredByFrontCornerRadar)))
   {
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(
         ::Per::Parameters::PerPostProcessTypes::applyImplausiblePedCheckLRR);
#endif  // RB_DC_DEBUG_STATES_ENABLED
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);
   }
}

void Dc::Per::Dep::applyImplausibleCarAtCloseRangeCheck(
   const ::Per::InterfaceDep::DepObject& obj, vfc::uint16_t& functionRelevanceBitField)
{
   constexpr vfc::float32_t dxThreshold                    = 17.F;
   constexpr vfc::float32_t lengthThreshold                = 7.F;
   constexpr vfc::uint8_t   minCyclesSinceLastSensorUpdate = 10U;

   const bool isObjClose{obj.getState().getX() < dxThreshold};
   const bool isObjLong{obj.getDimension().getPosition(0U) > lengthThreshold};
   const bool isCar{
      obj.get_objectTypeTreeReadOnly().get_mostProbableConditionalType()
      == ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_OBSTACLE_MOBILE_4PLUSWHEELER_CAR};

   const bool hasBeenUpdatedRecentlyByFcRadarOnly{
      obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
         ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar)
         == 0U
      && obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
            ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo)
            > minCyclesSinceLastSensorUpdate
      && obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
            ::Per::Parameters::ObstacleSensorIndex::frontLeftCornerObjectRadar)
            > minCyclesSinceLastSensorUpdate
      && obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
            ::Per::Parameters::ObstacleSensorIndex::frontRightCornerObjectRadar)
            > minCyclesSinceLastSensorUpdate};

   if (isObjClose && isObjLong && isCar && hasBeenUpdatedRecentlyByFcRadarOnly)
   {
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(
         ::Per::Parameters::PerPostProcessTypes::applyImplausibleCarAtCloseRangeCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);
   }
}

void Dc::Per::Dep::applyElevatedObjectCheck(
   const ::Per::InterfaceDep::DepObject&          obj,
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround,
   const bool                                     isObjectVru,
   vfc::uint16_t&                                 functionRelevanceBitField)
{
   constexpr vfc::float32_t rcsHighThreshold              = -3.0F;
   constexpr vfc::float32_t rcsLowThreshold               = -12.0F;
   constexpr vfc::float32_t dzThreshold                   = 2.1F;
   constexpr vfc::float32_t vxThreshold                   = 2.2F;
   constexpr vfc::float32_t vyThreshold                   = 0.4F;
   constexpr vfc::uint8_t   minCyclesSinceLastVideoUpdate = 5U;

   // Check for non or low velocity. Velocity thresholds have to account for virtual vx increase due to ego
   // passing underneath the object
   const bool isObjectStationaryOrSlow{
      (absVelOverGround[0] < vxThreshold) && (absVelOverGround[1] < vyThreshold)};

   // Check for no recent video update
   const bool hasNotBeenUpdatedByVideoRecently{
      obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
         ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo)
      > minCyclesSinceLastVideoUpdate};

   // Check for low RCS
   const bool isRcsLow{obj.get_rcs() < rcsHighThreshold};
   const bool isRcsVeryLow{obj.get_rcs() < rcsLowThreshold};

   // Check for high elevation
   const bool isElevatedObj{obj.elevation_isValid() && obj.get_elevation() > dzThreshold};

   // Additional case for VRUs
   constexpr vfc::float32_t                 absVelThreshold = 0.4F;
   constexpr vfc::float32_t                 rcsThreshold    = -6.0F;
   vfc::linalg::TVectorN<vfc::float32_t, 2> dxThresholds{};
   dxThresholds[0] = 0.0F;
   dxThresholds[1] = 40.0F;
   vfc::linalg::TVectorN<vfc::float32_t, 2> dzThresholds{};
   dzThresholds[0] = 2.0F;
   dzThresholds[1] = 3.0F;

   const bool isObjectStationary{
      (absVelOverGround[0] < absVelThreshold) && (absVelOverGround[1] < absVelThreshold)};

   const bool hasLowRcs{obj.get_rcs() < rcsThreshold};

   const vfc::float32_t objDx = obj.getState().getX();
   const vfc::float32_t allowedDzThreshold{
      ::Per::Algos::InterpolatingFcts::linearInterpolateLinearExtrapolate(dxThresholds, dzThresholds, objDx)};
   const bool isElevationTooHigh{obj.elevation_isValid() && obj.get_elevation() > allowedDzThreshold};

   // check for bad detection ratio
   constexpr vfc::float32_t ratioBadDetectionThreshold{0.375F};
   const vfc::uint32_t      numCycleExistingInDep{
      (obj.get_numCyclesExisting() - obj.getTransferredFromSepCycle()) + 1U};
   const vfc::uint32_t numOfRelevantCycles{vfc::min(numCycleExistingInDep, 8U)};
   const vfc::uint32_t numVideoOnlyOrNonUpdates{
      obj.m_sensorMeasuredHistory.numberOfSingleUpdateBySensorXOrNonUpdate(
         ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo,
         static_cast<vfc::uint8_t>(numOfRelevantCycles))};
   vfc::float32_t ratioVideoToNumRelevantCycles{0.F};
   if (numOfRelevantCycles != 0U && obj.get_numCyclesExisting() >= obj.getTransferredFromSepCycle())
   {
      ratioVideoToNumRelevantCycles = vfc::divide<vfc::float32_t>(
         static_cast<vfc::float32_t>(numVideoOnlyOrNonUpdates),
         static_cast<vfc::float32_t>(numOfRelevantCycles));
   }
   else
   // coco begin validated: PER_00060
   {
      VFC_ASSERT2(
         obj.get_numCyclesExisting() >= obj.getTransferredFromSepCycle(),
         "From expected logic obj.get_numCyclesExisting() should be at least equal as "
         "obj.getTransferredFromSepCycle()");
   }
   // coco end

   // static pedestrian
   constexpr vfc::float32_t pPedestrianThresholdHigh{0.7F};
   constexpr vfc::uint16_t  staticLocationCntThreshold{4U};
   const vfc::float32_t     pPedestrian{obj.get_objectTypeTree().get_condProbOfType(
      ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_OBSTACLE_MOBILE_PEDESTRIAN)};
   const bool               isStaticPedestrian{
      pPedestrian > pPedestrianThresholdHigh
      && obj.getStationaryLocationsOnlyCounter() >= staticLocationCntThreshold};
   const bool hasBadDetectionRatio{ratioVideoToNumRelevantCycles >= ratioBadDetectionThreshold};

   // additional case for objects with high video dr innovation
   // Check for video and radar update
   const bool hasBeenUpdatedByVideoAndRadar{
      obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
         ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo)
         == 0U
      && obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
            ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar)
            == 0U};
   // video innovation
   constexpr vfc::float32_t                       videoDrInnoThreshold{2.2F};
   constexpr vfc::uint8_t                         youngObjThreshold{30U};
   const vfc::linalg::TVectorN<vfc::float32_t, 2> videoBasedInno{obj.getVideoBasedInnovation()};
   const bool isVideoDrInnoUnreliable{videoBasedInno[0] > videoDrInnoThreshold};
   const bool isObjectYoung{obj.get_numCyclesExisting() < youngObjThreshold};
   const bool isHighVideoInno{hasBeenUpdatedByVideoAndRadar && isVideoDrInnoUnreliable && isObjectYoung};

   // Elevated car
   // Check if the object is of type car at a far distance with radar angle innovation is high
   const bool isObjTypeCar{
      obj.get_objectTypeTreeReadOnly().get_mostProbableConditionalType()
      == ::Per::Parameters::PER_OBJECT_TYPES::TYPE_OBJ_OBSTACLE_MOBILE_4PLUSWHEELER_CAR};
   constexpr vfc::float32_t closeDistanceThreshold   = 27.F;
   const bool               isObjectNotCloseDistance = obj.getState().getX() > closeDistanceThreshold;
   constexpr vfc::float32_t highAlphaThreshold       = 2.0F;
   const bool               isRadarAngleInnovationHigh =
      (obj.getRadarBasedInnovation()[1] * vfc::typedRadian2Degree<vfc::float32_t>()) > highAlphaThreshold;

   // Check if elevation of car is suspicious
   constexpr vfc::float32_t dzThreshForCar = 1.9F;
   const bool isElevationTooHighForCar{obj.elevation_isValid() && obj.get_elevation() > dzThreshForCar};

   // Check if camera contribution relative to radar contribution is insignificant, less than 20%.
   const bool notRecentlyUpdatedByVideo =
      (obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(::Per::Parameters::PerSensorTechs::techVideo)
       >= 2U);
   const bool objHasVeryLessVideoContribution =
      notRecentlyUpdatedByVideo
      && static_cast<vfc::float32_t>(obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
            ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo))
            < (0.2F * static_cast<vfc::float32_t>(obj.get_numCyclesExisting()));
   const bool isNotSeenByCornerRadar{
      (obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
         ::Per::Parameters::ObstacleSensorIndex::frontLeftCornerObjectRadar))
         == 0U
      && (obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
            ::Per::Parameters::ObstacleSensorIndex::frontRightCornerObjectRadar))
            == 0U};

   // Check if object is a low confident and elevated car
   const bool isLowConfidentCar{
      isObjTypeCar && isRadarAngleInnovationHigh && !isObjectYoung && objHasVeryLessVideoContribution
      && isNotSeenByCornerRadar && isObjectNotCloseDistance};
   const bool isElevatedCar{isObjTypeCar && isElevationTooHighForCar};

   // object either has very low rcs OR a low rcs with poor or outdated tracking OR is a static pedestrian
   // with high video innovation
   const bool isLowConfidenceObject{
      isRcsVeryLow
      || (isRcsLow && (hasNotBeenUpdatedByVideoRecently || (isStaticPedestrian && hasBadDetectionRatio)))
      || (isHighVideoInno && isStaticPedestrian) || isLowConfidentCar};

   // (a stationary/slow object or not updated by video recently) and it is elevated and it is a low
   // confidence object
   const bool isElevatedAndUnreliableDetection{
      (isObjectStationaryOrSlow || hasNotBeenUpdatedByVideoRecently) && (isElevatedObj || isElevatedCar)
      && isLowConfidenceObject};

   // stationary VRU with low rcs and high elevation
   const bool isStationaryElevatedVRU{isObjectVru && isObjectStationary && hasLowRcs && isElevationTooHigh};

   if (isElevatedAndUnreliableDetection || isStationaryElevatedVRU)
   {
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);

#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(::Per::Parameters::PerPostProcessTypes::applyElevatedObjectCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
   }
}

void Dc::Per::Dep::applyBridgeCheck(
   const ::Per::InterfaceDep::DepObject& obj, vfc::uint16_t& functionRelevanceBitField)
{
   // The thresholds were chosen based on a bridge scene and has no effect on NCAP.
   constexpr vfc::float32_t drVideoInnoThreshold           = 7.7F;
   constexpr vfc::float32_t dzThreshold_1                  = 1.7F;
   constexpr vfc::float32_t dzThreshold_2                  = 2.2F;
   constexpr vfc::uint16_t  stationaryLocationsThreshold_1 = 3U;
   constexpr vfc::uint16_t  stationaryLocationsThreshold_2 = 5U;

   const bool isVideoDrInnoHigh = (vfc::abs(obj.getVideoBasedInnovation()[0U]) > drVideoInnoThreshold);
   const bool isObjHigh_1       = (obj.elevation_isValid() && vfc::abs(obj.get_elevation()) > dzThreshold_1);
   const bool isObjHigh_2       = (obj.elevation_isValid() && vfc::abs(obj.get_elevation()) > dzThreshold_2);
   const bool doesObjHaveManyStationaryLocations_1 =
      obj.getStationaryLocationsOnlyCounter() > stationaryLocationsThreshold_1;
   const bool doesObjHaveManyStationaryLocations_2 =
      obj.getStationaryLocationsOnlyCounter() > stationaryLocationsThreshold_2;
   const bool hasBeenUpdatedByFcRadarOnly{
      obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
         ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar)
         == 0U
      && obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
            ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo)
            != 0U
      && obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
            ::Per::Parameters::ObstacleSensorIndex::frontLeftCornerObjectRadar)
            != 0U
      && obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
            ::Per::Parameters::ObstacleSensorIndex::frontRightCornerObjectRadar)
            != 0U};

   if (
      (isVideoDrInnoHigh && isObjHigh_1 && doesObjHaveManyStationaryLocations_1)
      || (isObjHigh_2 && doesObjHaveManyStationaryLocations_2 && hasBeenUpdatedByFcRadarOnly))
   {
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(::Per::Parameters::PerPostProcessTypes::applyBridgeCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);
   }
}

void Dc::Per::Dep::applyCornerRadarAssoWithStationaryLocationsForTheFirstTime(
   const bool                                     isObjectVru,
   const ::Per::Frames::Rot1dAcc2dFor&            hvmFor,
   const ::Per::InterfaceDep::DepObject&          obj,
   const vfc::linalg::TVectorN<vfc::float32_t, 2> absVelOverGround,
   vfc::uint16_t&                                 functionRelevanceBitField)
{
   constexpr vfc::float32_t lowerAbsSpeedThreshold = 0.8F;
   constexpr vfc::float32_t angleDtThreshold       = 0.523599F;  // ~30 deg/s

   const bool isEgoYawRateHigh = vfc::abs(hvmFor.getAngleDt(0U)) > angleDtThreshold;

   const bool isObjectTooYoung{obj.get_numCyclesExisting() < 15U};

   const bool objectIsStationary =
      (absVelOverGround[0] < lowerAbsSpeedThreshold) && (absVelOverGround[1] < lowerAbsSpeedThreshold);
   const bool mostlySeenByFLCornerRadar{
      (obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
         ::Per::Parameters::ObstacleSensorIndex::frontLeftCornerObjectRadar))
      > 11U};
   const bool mostlySeenByFRCornerRadar{
      (obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
         ::Per::Parameters::ObstacleSensorIndex::frontRightCornerObjectRadar))
      > 11U};
   const bool isFirstUpdateByRadar{
      (obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
         ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar))
      == 1U};
   const bool notObservedByVideo{
      obj.sensorFilterFusHelper.getTotalNumSensorUpdates(
         ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo)
      == 0U};
   const bool objectHasHighStationaryLocationCount{obj.getStationaryLocationsOnlyCounter() >= 2};

   const bool conditionsMet = isObjectVru && isObjectTooYoung && objectIsStationary
                              && (mostlySeenByFLCornerRadar || mostlySeenByFRCornerRadar)
                              && isFirstUpdateByRadar && notObservedByVideo
                              && objectHasHighStationaryLocationCount && isEgoYawRateHigh;

   if (conditionsMet)
   {
#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(
         ::Per::Parameters::PerPostProcessTypes::applyCornerRadarAssoWithStationaryLocationsForTheFirstTime);
#endif  // RB_DC_DEBUG_STATES_ENABLED
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);
   }
}

void Dc::Per::Dep::applyInconsistentAlphaCheck(
   const ::Per::InterfaceDep::DepObject& obj, vfc::uint16_t& functionRelevanceBitField)
{
   constexpr vfc::float32_t highAlphaThreshold     = 0.02F;  // ~1.15 degree
   constexpr vfc::float32_t closeDistanceThreshold = 25.F;
   constexpr vfc::float32_t lowDyThreshold         = 2.5F;

   const bool isRadarHighAlpha = vfc::abs(obj.getRadarRawAlphaInnovation()) > highAlphaThreshold;
   const bool isVideoHighAlpha = vfc::abs(obj.getVideoRawAlphaInnovation()) > highAlphaThreshold;
   const bool isVideoAndRadarAlphaInnoDifferentSign =
      (obj.getRadarRawAlphaInnovation() * obj.getVideoRawAlphaInnovation()) < 0.0F;

   const bool isAlphaInnoInconsistent{
      isRadarHighAlpha && isVideoHighAlpha && isVideoAndRadarAlphaInnoDifferentSign};

   const bool isObjectNotCloseDistance = obj.getState().getX() > closeDistanceThreshold;
   const bool isObjectDyLow            = vfc::abs(obj.getState().getY()) < lowDyThreshold;
   const bool isObjectUpdatedByVideoAndRadar =
      obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
         ::Per::Parameters::ObstacleSensorIndex::frontCenterVideo)
         == 0U
      && obj.sensorFilterFusHelper.getUpdatesSinceLastSensorUpdate(
            ::Per::Parameters::ObstacleSensorIndex::frontCenterLocationRadar)
            == 0U;
   if (isAlphaInnoInconsistent && isObjectNotCloseDistance && isObjectDyLow && isObjectUpdatedByVideoAndRadar)
   {
      PostProcessing::disqualifyForAeb(functionRelevanceBitField);

#if defined(RB_DC_DEBUG_STATES_ENABLED)
      obj.setSingleBitInPostProcessBitField(
         ::Per::Parameters::PerPostProcessTypes::applyInconsistentAlphaCheck);
#endif  // RB_DC_DEBUG_STATES_ENABLED
   }
}
