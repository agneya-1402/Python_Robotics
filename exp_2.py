import pybullet as p
import pybullet_data
import time
import math

# Set up
physicsClient = p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())  
p.setGravity(0, 0, -9.8)

# Load KUKA  
robotId = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0, 0, 0.5])

# joints 
num_joints = p.getNumJoints(robotId)
print(f"KUKA Robot has {num_joints} joints.")

# Slider Control
sliderIds = []
sliderMax = 2 * math.pi  # max joint angle
for i in range(num_joints):
    sliderId = p.addUserDebugParameter(f"Joint {i}", -sliderMax, sliderMax, 0)  
    sliderIds.append(sliderId)

# Simulate
for step in range(5000):  
    p.stepSimulation()

    # fetch slider value
    for i in range(num_joints):
        joint_angle = p.readUserDebugParameter(sliderIds[i])  
        p.setJointMotorControl2(robotId, jointIndex=i, controlMode=p.POSITION_CONTROL, targetPosition=joint_angle)

    time.sleep(1 / 240)  

p.disconnect()
