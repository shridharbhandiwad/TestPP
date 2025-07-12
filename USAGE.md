# Automotive Perception Function Tester

## Overview
This web application allows you to test all 40 automotive perception functions from the C++ codebase. Each function can be tested individually by providing input values and checking if the function conditions are "hit" (satisfied).

## Features

### 🚗 **40 Automotive Perception Functions**
- Complete implementation of all functions from the analysis
- Each function has its specific input parameters and conditions
- Real-time testing with visual feedback

### 📊 **Rich Statistics Dashboard**
- Total functions count
- Functions hit count
- Success rate percentage
- Real-time updates

### 🎨 **Modern UI Design**
- Beautiful gradient background
- Responsive design for all devices
- Hover effects and animations
- Color-coded results (green for hit, red for miss)

### 🔧 **Interactive Controls**
- **Test All Functions**: Tests all functions at once
- **Reset All**: Clears all inputs and results
- **Fill Random Values**: Automatically fills all inputs with realistic random values
- **Individual Testing**: Test each function separately

## How to Use

### 1. **Open the Application**
- Open `index.html` in your web browser
- Or serve it via HTTP server: `python -m http.server 8000`

### 2. **Input Values**
Each function card contains:
- **Function name** and description
- **Input fields** for all required parameters
- **Test button** to check the function condition

### 3. **Testing Methods**

#### **Individual Testing**
1. Fill in the input fields for a specific function
2. Click "Test Function" button
3. See immediate results (Hit/Miss)

#### **Bulk Testing**
1. Fill in values for multiple functions
2. Click "Test All Functions"
3. See summary statistics and detailed results

#### **Random Testing**
1. Click "Fill Random Values" to auto-populate all inputs
2. Click "Test All Functions" to test everything at once

### 4. **Understanding Results**

#### **Function Status**
- **🟠 Pending**: Not yet tested
- **🟢 Hit**: Condition satisfied (function triggered)
- **🔴 Miss**: Condition not satisfied

#### **Actions**
Each function shows what action it performs:
- **"Disqualifies for AEB"**: Automatic Emergency Braking disqualification
- **"Disqualifies for ACC"**: Adaptive Cruise Control disqualification
- **"Modifies object probabilities"**: Updates object classification
- **"Returns boolean"**: Provides true/false result

## Function Categories

### **Object Suppression Functions**
- `applySuppressionUntilNextVideoUpdateCheck`
- `applyPostProcessVideoOtcCheck`

### **Movement Analysis Functions**
- `isMovingTowardsEgoLane`
- `applyUnreliableAngularVelocityCheck`
- `applyNonCrossingObjectCheck`

### **Sensor Quality Functions**
- `applyMicroDopplerCheck`
- `applyRadarOnlyRcsAndDrInnovationLimit`
- `applySensorBasedInnoCheck`

### **Object Classification Functions**
- `applyFourpluswheelerChecks`
- `applyImplausiblePedCheck`
- `applyImplausibleCarAtCloseRangeCheck`

### **Environmental Checks**
- `applyElevationCheck`
- `applyBridgeCheck`
- `applyWaterSprinklesCheck`

### **And many more...**

## Input Parameter Types

### **Numerical Inputs**
- Positions (X, Y coordinates in meters)
- Velocities (m/s)
- Angles (degrees)
- Probabilities (0-1 range)
- Counts (integer values)
- RCS values (dBsm)
- Innovation values

### **Boolean Inputs**
- True/False selections
- Object type flags (VRU, Car, Pedestrian)
- Sensor update flags

### **Selection Inputs**
- Filter types (CA, CV, LA)
- Predefined option sets

## Tips for Testing

1. **Use Random Values**: Click "Fill Random Values" to get realistic test data
2. **Test Systematically**: Test all functions to see which conditions are most likely to be hit
3. **Analyze Results**: Use the summary section to understand which functions are triggered
4. **Experiment**: Try different value combinations to understand function behavior

## Technical Details

- **Frontend**: HTML5, CSS3, JavaScript (ES6+)
- **Responsive Design**: Works on desktop, tablet, and mobile
- **No Backend Required**: Pure client-side application
- **Real-time Updates**: Instant feedback on all interactions

## Browser Compatibility

- Chrome/Edge: Full support
- Firefox: Full support
- Safari: Full support
- Mobile browsers: Full support

Enjoy testing the automotive perception functions! 🚗✨