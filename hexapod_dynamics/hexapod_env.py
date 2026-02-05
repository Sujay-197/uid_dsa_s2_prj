import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
import pybullet_data
import time
import math

class HexapodEnv(gym.Env):
    metadata = {'render_modes': ['human', 'rgb_array']}

    def __init__(self, render_mode=None):
        super(HexapodEnv, self).__init__()
        self.render_mode = render_mode
        
        # Connect to PyBullet
        if self.render_mode == 'human':
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)
            
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Action space: Target positions for 12 joints
        # Range: -pi to +pi (though practical limits are smaller)
        self.action_space = spaces.Box(
            low=-np.pi, high=np.pi, shape=(12,), dtype=np.float32
        )
        
        # Observation space: 
        # Base: Position (3), Orientation (4) = 7
        # Joints: Positions (12), Velocities (12) = 24
        # Total: 31
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(31,), dtype=np.float32
        )
        
        self.robot = None
        self.planeId = None

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        
        self.planeId = p.loadURDF("plane.urdf")
        
        # Recreate robot logic (condensed from bullet_joint.py)
        self.create_robot()
        
        # Randomize initial state slightly
        # Or just start in a "safe" idle pose
        
        # Step a few times to settle
        for _ in range(10):
            p.stepSimulation()
            
        observation = self._get_obs()
        info = {}
        return observation, info

    def create_robot(self):
        # --- SHAPE DEFINITIONS ---
        torso_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.15, 0.05])
        upper_leg_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.2])
        lower_leg_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.045, 0.045, 0.2])
        
        torso_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 0.15, 0.05], rgbaColor=[0.2, 0.2, 0.8, 1])
        upper_leg_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.2], rgbaColor=[0.2, 0.8, 0.2, 1])
        lower_leg_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.045, 0.045, 0.2], rgbaColor=[0.8, 0.2, 0.2, 1])

        leg_torso_positions = [
            [0.15, -0.12, -0.02],   # Leg 1: Front-Right
            [0.0, -0.15, -0.02],    # Leg 2: Middle-Right
            [-0.15, -0.12, -0.02],  # Leg 3: Back-Right
            [0.15, 0.12, -0.02],    # Leg 4: Front-Left
            [0.0, 0.15, -0.02],     # Leg 5: Middle-Left
            [-0.15, 0.12, -0.02]    # Leg 6: Back-Left
        ]
        knee_offset = [0, 0, -0.36]

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

        for leg_idx in range(6):
            # --- Upper leg ---
            linkMasses.append(2)
            linkCollisionShapeIndices.append(upper_leg_col)
            linkVisualShapeIndices.append(upper_leg_vis)
            linkPositions.append(leg_torso_positions[leg_idx])
            linkOrientations.append([0, 0, 0, 1])
            linkInertialFramePositions.append([0, 0, 0])
            linkInertialFrameOrientations.append([0, 0, 0, 1])
            linkParentIndices.append(0)
            linkJointTypes.append(p.JOINT_REVOLUTE)
            linkJointAxis.append([1, 0, 0]) # Hip X-axis

            # --- Lower leg ---
            linkMasses.append(2)
            linkCollisionShapeIndices.append(lower_leg_col)
            linkVisualShapeIndices.append(lower_leg_vis)
            linkPositions.append(knee_offset)
            linkOrientations.append([0, 0, 0, 1])
            linkInertialFramePositions.append([0, 0, 0])
            linkInertialFrameOrientations.append([0, 0, 0, 1])
            linkParentIndices.append((2 * leg_idx) + 1)
            linkJointTypes.append(p.JOINT_REVOLUTE)
            linkJointAxis.append([1, 0, 0]) # Knee X-axis

        self.robot = p.createMultiBody(
            baseMass=5,
            baseCollisionShapeIndex=torso_col,
            baseVisualShapeIndex=torso_vis,
            basePosition=[0, 0, 1.2],
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
        
        # Stability settings
        p.changeDynamics(self.robot, -1, linearDamping=0.8, angularDamping=0.8, lateralFriction=1.5, restitution=0.1)
        for i in range(p.getNumJoints(self.robot)):
            p.changeDynamics(self.robot, i, 
                linearDamping=1.2, angularDamping=1.2, lateralFriction=1.5, 
                jointLimitForce=500, maxJointVelocity=10, jointDamping=0.5
            )
            p.enableJointForceTorqueSensor(self.robot, i, enableSensor=True)

    def step(self, action):
        # Clip actions and apply
        # We can map action [-1, 1] to specific joint limits if we want, 
        # but here we assume the agent outputs radians directly.
        # It's safer to clip to our safe limits.
        
        clipped_action = np.clip(action, -np.pi, np.pi)
        
        for i in range(12):
            p.setJointMotorControl2(
                self.robot, i, 
                controlMode=p.POSITION_CONTROL, 
                targetPosition=clipped_action[i],
                force=500,
                positionGain=0.5,
                velocityGain=0.3
            )
            
        p.stepSimulation()
        
        if self.render_mode == "human":
            time.sleep(1./240.)
            
        observation = self._get_obs()
        reward = self._calculate_reward()
        terminated = self._check_terminated()
        truncated = False # Typically handled by TimeLimit wrapper
        info = {}
        
        return observation, reward, terminated, truncated, info

    def _get_obs(self):
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot)
        
        joint_states = p.getJointStates(self.robot, range(p.getNumJoints(self.robot)))
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        
        obs = np.concatenate([
            base_pos, base_orn, joint_positions, joint_velocities
        ])
        return obs.astype(np.float32)

    def _calculate_reward(self):
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot)
        
        # 1. Height Reward: Target height around 0.8m - 1.2m
        # It starts at 1.2m. 
        # If it falls, reward decreases.
        target_height = 0.8
        height_reward = -abs(base_pos[2] - target_height)
        
        # 2. Orientation Reward: Keep flat (upright)
        roll, pitch, yaw = p.getEulerFromQuaternion(base_orn)
        orientation_penalty = abs(roll) + abs(pitch)
        
        # 3. Survival Bonus
        survival_bonus = 1.0
        
        return survival_bonus + 2.0 * height_reward - 0.5 * orientation_penalty

    def _check_terminated(self):
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot)
        
        # Fall detection
        if base_pos[2] < 0.3:
            return True
            
        # Tilt detection
        roll, pitch, yaw = p.getEulerFromQuaternion(base_orn)
        if abs(roll) > 0.6 or abs(pitch) > 0.6: # ~35 degrees
            return True
            
        return False

    def close(self):
        p.disconnect()
