# Wall Following V2 Mission Checklist

> **Overall Settings**:
> - **Speed**: 0.1 m/s (Linear)
> - **Turn Speed**: 1.5 rad/s (Approx. 90 deg/s)
> - **Sensors Used**: ToF Front, ToF Right-Front (Mapped as Right), Encoders (for Turns/Arcs).
> - **Stop Distance Logic**: Uses Front ToF.

## Phase 1: Start & Top Section

### 1. Convex Corner RT (Start)
- **Action**: Drive Straight (0.1m/s).
- **Trigger**: Front ToF < **203mm (8 inches)**.
- **Reaction**: Stop -> **Turn Left 90°** (CCW).
- **Check**: If Front ToF still < 203mm -> **Turn Left 90° Again** (CCW).
- **Next**: Enter Trajectory Top.

### 2. Trajectory Top (Semi-Circle)
- **Action**: Drive **180° Arc** (Semi-circle).
  - **Radius**: 0.33m (13 inches).
  - **Speed**: 0.1 m/s.
  - **Direction**: Left Turn Arc (CCW).
- **Trigger**: Arc angle reaching 180° (via Encoders).
- **End Action**: Stop -> **Turn Left 90°** (CCW).
- **Next**: Enter Convex Corner LT.

## Phase 2: Left Side Long Run

### 3. Convex Corner LT
- **Enter Action**: **Turn Left 90°** (CCW) immediately upon entry logic (Step 0).
- **Action**: **Wall Follow Right**.
  - **Target Distance**: **50mm** from Right Wall.
  - **Correction**: P-Controller.
- **Trigger**: Front ToF < **130mm (13cm)**.
- **Reaction**: Stop -> **Turn Left 90°** (CCW).
- **Next**: Enter Long Edge Left.

### 4. Long Edge Left
- **Action**: **Wall Follow Right**.
  - **Target Distance**: **70mm** (7cm) from Right Wall.
  - **Blind Zone**: Ignore front sensor for first **0.5 meters**.
- **Trigger**: Front ToF < **100mm (10cm)** (Only active after >0.5m driven).
- **Reaction**: Stop -> **Turn Left 90°** (CCW).
- **Next**: Enter Convex Corner LB.

## Phase 3: Bottom Section

### 5. Convex Corner LB
- **Action**: Drive Straight (0.1m/s).
- **Trigger**: Front ToF < **80mm (8cm)**.
- **Reaction**: Stop -> **Turn Left 90°** (CCW).
- **Next**: Enter Trajectory Bottom.

### 6. Trajectory Bottom (Semi-Circle)
- **Action**: Drive **180° Arc** (Semi-circle).
  - **Radius**: 0.33m (13 inches).
  - **Speed**: 0.1 m/s.
  - **Direction**: Left Turn Arc (CCW).
- **Trigger**: Arc angle reaching 180° (via Encoders).
- **End Action**: Stop -> **Turn Left 90°** (CCW).
- **Next**: Enter Convex Corner RB.

## Phase 4: Right Side Return & Finish

### 7. Convex Corner RB
- **Action**: **Wall Follow Right**.
  - **Target Distance**: **50mm** from Right Wall.
- **Trigger**: Front ToF < **130mm (13cm)**.
- **Reaction**: Stop -> **Turn Left 90°** (CCW).
- **Next**: Enter Long Edge Right.

### 8. Long Edge Right (Finish)
- **Action**: **Wall Follow Right**.
  - **Target Distance**: **50mm** from Right Wall.
- **Trigger**: Front ToF < **80mm (8cm)**.
- **Reaction**: **FULL STOP**. Mission Complete.
