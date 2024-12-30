import pybullet as p
import pybullet_data
import time

# Set up
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Load objs
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 1])

# Simulate 
for step in range(300):
    p.stepSimulation()
    time.sleep(1 / 240)  

p.disconnect()
