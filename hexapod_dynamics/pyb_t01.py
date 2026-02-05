import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")

box = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[0.2, 0.2, 0.2])

visual = p.createVisualShape(
    p.GEOM_BOX, 
    halfExtents=[0.2, 0.2, 0.2])

p.createMultiBody(
    baseMass=1,
    baseCollisionShapeIndex=box,
    baseVisualShapeIndex=visual,
    basePosition=[0, 0, 1]
)

try:
    while p.isConnected():
        p.stepSimulation()
        time.sleep(1 / 240)
except KeyboardInterrupt:
    pass

p.disconnect()
