import pybullet as p
import time
import pybullet_data


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cheetahStartPos = [0,0,1]
cheetahStartOrientation = p.getQuaternionFromEuler([0,0,0])
cheetahId = p.loadURDF("cheetah.urdf", cheetahStartPos, cheetahStartOrientation)


#Orignal fps = 1/100
fps = 1/100

for i in range(p.getNumJoints(cheetahId)):
  print(p.getJointInfo(cheetahId,i))

#steering=[0,1,3,5,6,8,10,11,13,15,16,18]

angles=[0 for i in range(p.getNumJoints(cheetahId))]
print(angles)
front_right = [2,4] 
front_left = [8,10]
back_right = [14,16]
back_left = [20,22]

#0.15 degrees
steer = 0.3490669

directions=[1,1,1,1]


for i in range(10000):

    p.setJointMotorControl2(cheetahId, 2, p.POSITION_CONTROL, targetPosition=-steer*directions[0])
    p.setJointMotorControl2(cheetahId, 4, p.POSITION_CONTROL, targetPosition=2*steer*directions[0])

    p.setJointMotorControl2(cheetahId, 8, p.POSITION_CONTROL, targetPosition=-steer*directions[1])
    p.setJointMotorControl2(cheetahId, 10, p.POSITION_CONTROL, targetPosition=2*steer*directions[1])

    p.setJointMotorControl2(cheetahId, 14, p.POSITION_CONTROL, targetPosition=-steer*directions[2])
    p.setJointMotorControl2(cheetahId, 16, p.POSITION_CONTROL, targetPosition=2*steer*directions[2])

    p.setJointMotorControl2(cheetahId, 20, p.POSITION_CONTROL, targetPosition=-steer*directions[3])
    p.setJointMotorControl2(cheetahId, 22, p.POSITION_CONTROL, targetPosition=2*steer*directions[3])

    p.stepSimulation()
    time.sleep(fps)
    
cubePos, cubeOrn = p.getBasePositionAndOrientation(cheetahId)
print(cubePos,cubeOrn)
p.disconnect()



