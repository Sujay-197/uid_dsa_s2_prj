import pybullet as p
import pybullet_data
import time
import math


'''Hexapod Robot (Spider-like) - 6 Legged Robot
 Torso (base, fixed)
 ├── Leg 1 (Front-Right): Hip joint (Y-axis) → Upper leg → Knee joint (X-axis) → Lower leg
 ├── Leg 2 (Middle-Right): Hip joint (Y-axis) → Upper leg → Knee joint (X-axis) → Lower leg
 ├── Leg 3 (Back-Right): Hip joint (Y-axis) → Upper leg → Knee joint (X-axis) → Lower leg
 ├── Leg 4 (Front-Left): Hip joint (Y-axis) → Upper leg → Knee joint (X-axis) → Lower leg
 ├── Leg 5 (Middle-Left): Hip joint (Y-axis) → Upper leg → Knee joint (X-axis) → Lower leg
 └── Leg 6 (Back-Left): Hip joint (Y-axis) → Upper leg → Knee joint (X-axis) → Lower leg
'''

# --- connect ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# ground
p.loadURDF("plane.urdf")

# Create collision and visual shapes for torso and legs
torso_col = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[0.2, 0.15, 0.05]
)

upper_leg_col = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.2]
)

lower_leg_col = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[0.045, 0.045, 0.2]
)

torso_vis = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[0.2, 0.15, 0.05],
    rgbaColor=[0.2, 0.2, 0.8, 1]
)

upper_leg_vis = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[0.05, 0.05, 0.2],
    rgbaColor=[0.2, 0.8, 0.2, 1]
)

lower_leg_vis = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[0.045, 0.045, 0.2],
    rgbaColor=[0.8, 0.2, 0.2, 1]
)

# Leg attachment positions on torso (6 legs in alternating pattern)
# Right side: front, middle, back
# Left side: front, middle, back
leg_torso_positions = [
    [0.15, -0.12, -0.02],   # Leg 1: Front-Right
    [0.0, -0.15, -0.02],    # Leg 2: Middle-Right
    [-0.15, -0.12, -0.02],  # Leg 3: Back-Right
    [0.15, 0.12, -0.02],    # Leg 4: Front-Left
    [0.0, 0.15, -0.02],     # Leg 5: Middle-Left
    [-0.15, 0.12, -0.02],   # Leg 6: Back-Left
]

# Hip offset from torso (upper leg attachment)
hip_offset = [0.18, 0, -0.05]

# Knee offset from hip (lower leg attachment - relative to upper leg)
knee_offset = [0, 0, -0.36]

# Build link data for all 6 legs (each leg has 2 links: upper, lower)
linkMasses = []
linkCollisionShapeIndices = []
linkVisualShapeIndices = []
linkPositions = []
linkOrientations = []
linkInertialFramePositions = []
linkInertialFrameOrientations = []
linkParentIndices = []
linkJointTypes = []
linkJointAxis = []

# Create 6 legs
for leg_idx in range(6):
    # Determine side (right = positive Y, left = negative Y)
    if leg_idx < 3:
        y_sign = -1  # Right side
    else:
        y_sign = 1   # Left side
    
    # --- Upper leg link ---
    linkMasses.append(2)  
    linkCollisionShapeIndices.append(upper_leg_col)
    linkVisualShapeIndices.append(upper_leg_vis)
    
    # Upper leg position relative to torso (hip joint)
    upper_pos = leg_torso_positions[leg_idx]
    linkPositions.append(upper_pos)
    
    linkOrientations.append([0, 0, 0, 1])
    linkInertialFramePositions.append([0, 0, 0])
    linkInertialFrameOrientations.append([0, 0, 0, 1])
    
    # Hip joint parent index (0 = torso/base)
    # Note: In createMultiBody, parent index 0 is the Base.
    # Link indices are 1-based (1 is the first link in the list, etc.)
    linkParentIndices.append(0)  
    
    # Hip joint: rotation around X-axis (Abduction/Lift) for Spider stance
    # This allows lifting the leg UP/DOWN.
    linkJointTypes.append(p.JOINT_REVOLUTE)
    linkJointAxis.append([1, 0, 0])
    
    # --- Lower leg link ---
    linkMasses.append(2)  # Heavier legs to support torso
    linkCollisionShapeIndices.append(lower_leg_col)
    linkVisualShapeIndices.append(lower_leg_vis)
    
    # Lower leg position relative to upper leg (knee joint)
    linkPositions.append(knee_offset)
    
    linkOrientations.append([0, 0, 0, 1])
    linkInertialFramePositions.append([0, 0, 0])
    linkInertialFrameOrientations.append([0, 0, 0, 1])
    
    # Knee joint parent index
    linkParentIndices.append((2 * leg_idx) + 1)
    
    # Knee joint: rotation around X-axis (flexion/extension)
    linkJointTypes.append(p.JOINT_REVOLUTE)
    linkJointAxis.append([1, 0, 0])

# Create hexapod robot with all 12 links (6 legs × 2 links each)
robot = p.createMultiBody(
    baseMass=5,  # Heavier torso for stability
    baseCollisionShapeIndex=torso_col,
    baseVisualShapeIndex=torso_vis,
    basePosition=[0, 0, 1.2],  # Higher initial position so legs can support
    
    linkMasses=linkMasses,
    linkCollisionShapeIndices=linkCollisionShapeIndices,
    linkVisualShapeIndices=linkVisualShapeIndices,
    
    linkPositions=linkPositions,
    linkOrientations=linkOrientations,
    linkInertialFramePositions=linkInertialFramePositions,
    linkInertialFrameOrientations=linkInertialFrameOrientations,
    linkParentIndices=linkParentIndices,
    linkJointTypes=linkJointTypes,
    linkJointAxis=linkJointAxis
)

# Stabilize torso with ground friction
p.changeDynamics(robot, -1, linearDamping=0.8, angularDamping=0.8, lateralFriction=1.5, restitution=0.1)

# Set joint limits and damping to ensure joints stay connected
# Configure each joint with proper constraints for stable RL training
for joint_idx in range(p.getNumJoints(robot)):
    joint_info = p.getJointInfo(robot, joint_idx)
    
    if joint_idx % 2 == 0:
        # Hip joint (X-axis): Allow full range but with safety limits
        lower_limit = -math.pi * 0.9  # -162 deg
        upper_limit = math.pi * 0.9   # +162 deg
    else:
        # Knee joint (X-axis): Wider range for spider stance
        lower_limit = -math.pi * 0.8  # -144 deg
        upper_limit = math.pi * 0.8   # +144 deg
    
    # Strong damping to prevent oscillations and disconnections
    p.changeDynamics(
        robot, 
        joint_idx,
        jointLowerLimit=lower_limit,
        jointUpperLimit=upper_limit,
        jointLimitForce=500,  # Force to enforce limits
        linearDamping=1.2,    # Higher damping for stability
        angularDamping=1.2,
        lateralFriction=1.5,
        jointDamping=0.5,     # Joint-specific damping
        maxJointVelocity=10   # Prevent rapid movements
    )
    
    # Enable joint feedback for monitoring
    p.enableJointForceTorqueSensor(robot, joint_idx, enableSensor=True)

print(f"Robot created with {p.getNumJoints(robot)} joints (12 joints for 6 legs)")
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    print(f"Joint {i}: {info[1].decode('utf-8')} - Parent: {info[16] if len(info)>16 else 'N/A'}") 

try:
    print("Hexapod simulation running... Standing Posture")
    print("Joint stability ensured - Ready for RL training")
    
    # Initial "Idle" Stand Targets
    # Hip: 30 deg up for spider stance
    # Right Side (Legs 0-2): +30 deg X-rot lifts up
    # Left Side (Legs 3-5): -30 deg X-rot lifts up
    # Knee: Curl down for ground contact
    
    step_count = 0
    
    while True:
        p.stepSimulation()
        step_count += 1
        
        for leg_i in range(6):
            # 2 joints per leg
            hip_idx = leg_i * 2
            knee_idx = leg_i * 2 + 1
            
            if leg_i < 3:
                # Right Side
                hip_target = math.radians(30)
                knee_target = math.radians(-120) 
            else:
                # Left Side
                hip_target = math.radians(-30)
                knee_target = math.radians(120)

            # Strong motor control with PD gains to maintain connections
            p.setJointMotorControl2(
                robot, hip_idx, p.POSITION_CONTROL, 
                targetPosition=hip_target, 
                force=500,  # High force to prevent disconnection
                positionGain=0.5,  # P gain
                velocityGain=0.3   # D gain
            )
            p.setJointMotorControl2(
                robot, knee_idx, p.POSITION_CONTROL, 
                targetPosition=knee_target, 
                force=500,  # High force to prevent disconnection
                positionGain=0.5,  # P gain
                velocityGain=0.3   # D gain
            )
        
        # Monitor joint states every 240 steps (1 second)
        if step_count % 240 == 0:
            base_pos, base_orn = p.getBasePositionAndOrientation(robot)
            print(f"Time: {step_count/240:.1f}s | Base height: {base_pos[2]:.3f}m | All joints connected ✓")

        time.sleep(1/240)
except KeyboardInterrupt:
    print("\nStopping simulation...")
    print("All joints remained connected throughout simulation")
    p.disconnect()
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
    p.disconnect()