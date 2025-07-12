# Threshold Editing Guide

## Overview
The automotive perception function tester has been enhanced with **editable threshold parameters** in each function's window. Previously, threshold values were hardcoded in the function conditions, but now you can adjust them dynamically to experiment with different values.

## What Was Added

### 1. Threshold Parameters
All functions that previously had hardcoded threshold values now have corresponding threshold input fields. These include:

- **Distance/Range Thresholds**: For spatial boundaries (e.g., `stateXThreshold`, `rangeThreshold`)
- **Velocity Thresholds**: For speed-related checks (e.g., `absVelOverGroundThreshold`)
- **Count Thresholds**: For cycle/update counters (e.g., `numCyclesExistingThreshold`)
- **Probability Thresholds**: For probability-based decisions (e.g., `probHasBeenObservedMovingThreshold`)
- **Innovation Thresholds**: For sensor innovation checks (e.g., `avgDxInnovationThreshold`)
- **RCS Thresholds**: For radar cross-section limits (e.g., `rcsThreshold`)
- **Elevation Thresholds**: For elevation angle checks (e.g., `elevationThreshold`)

### 2. Pre-populated Default Values
Each threshold parameter comes with a default value that matches the original hardcoded value. This ensures backward compatibility while allowing experimentation.

### 3. Load Default Thresholds Button
A new **red "Load Default Thresholds"** button has been added to the controls section that will populate all threshold fields with their default values.

## How to Use the New Features

### Method 1: Manual Threshold Adjustment
1. **Select a function** you want to experiment with
2. **Locate the threshold parameters** (they're labeled with "Threshold" in the name)
3. **Modify the threshold values** to your desired settings
4. **Fill in the other required inputs** (sensor data, object properties, etc.)
5. **Click "Test Function"** to see how the new thresholds affect the results

### Method 2: Load Default Thresholds
1. **Click "Load Default Thresholds"** to populate all threshold fields with their original values
2. **Modify specific thresholds** you want to experiment with
3. **Fill in the other required inputs**
4. **Test individual functions** or **Test All Functions**

### Method 3: Random Values + Threshold Adjustment
1. **Click "Fill Random Values"** to populate all input fields with random data
2. **Manually adjust specific threshold values** you want to test
3. **Click "Test All Functions"** to see how your threshold changes affect the overall results

## Examples of Threshold Experimentation

### Example 1: Innovation Threshold Tuning
Function: `applyInnovationCheck`
- Original: `avgDxInnovationThreshold = 2.0`
- Experiment: Try values like `1.5`, `2.5`, `3.0` to see how it affects object disqualification

### Example 2: Velocity Threshold Adjustment
Function: `applyUnreliableAngularVelocityCheck`
- Original: `absVelOverGroundThreshold = 2.0`
- Experiment: Try values like `1.5`, `2.5`, `3.0` to see sensitivity to velocity

### Example 3: Range-Based Threshold Scaling
Function: `calcDxInnovationThreshold`
- Original: `rangeMultiplier = 100`, `baseThreshold = 1.5`
- Experiment: Adjust these to see how threshold calculation changes with distance

## Functions with Threshold Parameters (40 total)

### Functions with Innovation Thresholds:
- `applyWaterSprinklersCheckAcc`
- `applyWaterSprinklesCheck`
- `applyRadarOnlyRcsAndDrInnovationLimit`
- `applyInnovationCheck`
- `calcDxInnovationThreshold`
- `applySensorBasedInnoCheck`
- `modifyBadSensorBasedInnoCount`

### Functions with Velocity Thresholds:
- `applyUnreliableAngularVelocityCheck`
- `applyUpdatedWithStatLocWithHighMDopplerWithOutgoingVrCheck`
- `applyIsMeasuredRatioCheckForFastWnj`
- `applyMicroDopplerCheck`
- `applyImplausibleVyVruCheck`
- `applyVideoHandleSharedCheck`
- `applyRadarOnlyStationaryCheck`

### Functions with Position/Range Thresholds:
- `applyElevationCheck`
- `applySensorBasedInnoCheck`
- `applyImplausibleVideoTtcForVru`
- `applyVyInconsistentCheck`
- `modifyBadSensorBasedInnoCount`
- `applyRadarOnlyNLDCheck`
- `applyImplausiblyAcceleratingVruCheck`
- `applyUndefinedCrossingVruFromCorner`

### Functions with Count/Cycle Thresholds:
- `applyNonCrossingObjectCheck`
- `applyIsMeasuredRatioCheckForRadarOnlyLongitudinallyMoving`
- `applyIsMeasuredRatioCheckForStandingLongiVru`
- `applyRadarOnlyNLDCheck`
- `applyCornerRadarAssoWithStationaryLocationsForTheFirstTime`

## Tips for Threshold Experimentation

1. **Start with Default Values**: Always begin with the defaults to understand baseline behavior
2. **Change One Threshold at a Time**: This helps isolate the effect of each parameter
3. **Use Realistic Values**: Keep thresholds within reasonable ranges for automotive applications
4. **Test Multiple Scenarios**: Try different combinations of input data to see threshold sensitivity
5. **Document Your Findings**: Keep track of threshold values that produce interesting results

## Troubleshooting

### If a function isn't behaving as expected:
1. **Check all input fields are filled**: Missing inputs will prevent testing
2. **Verify threshold values are reasonable**: Extreme values might prevent conditions from being met
3. **Use "Load Default Thresholds"**: Reset to known good values and try again
4. **Check the function description**: Make sure you understand what the function is testing

## Technical Details

### How Thresholds Work:
- Each threshold parameter has a `defaultValue` property
- The `createFunctionCard` function pre-populates input fields with default values
- The `loadDefaultThresholds` function can reset all threshold fields to their defaults
- All threshold parameters are treated as `number` type inputs with `step="0.1"` precision

### Button Styling:
- The "Load Default Thresholds" button uses a red gradient (`btn-quaternary` class)
- It follows the same hover effects as other control buttons

This enhancement transforms the static threshold values into dynamic, experimental parameters, allowing you to explore how different threshold settings affect the automotive perception functions' behavior.