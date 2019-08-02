import pybullet as p
import time
import pybullet_data
from math import radians

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("table.urdf")

cubeStartPos = [0,0,3]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("car2.urdf",cubeStartPos, cubeStartOrientation)

"""
cubeStartPos = [0,0,5]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("car2.urdf",cubeStartPos, cubeStartOrientation)
"""

for i in range(p.getNumJoints(planeId)):
  print(p.getJointInfo(planeId,i))

x_joint=2
y_joint=1



along_x = p.addUserDebugParameter("along_x", -30, 30, 0)
along_y = p.addUserDebugParameter("along_y", -30, 30, 0)


angle=0.01

one_radian=radians(angle)
steeringAngle=radians(angle)

curr_x=0
curr_y=0


for i in range (1000000):
  
   x_angle = radians(p.readUserDebugParameter(along_x))
   y_angle = radians(p.readUserDebugParameter(along_y))

   print(curr_x,curr_y)
   if x_angle-one_radian>curr_x:
       curr_x+=steeringAngle
       p.setJointMotorControl2(planeId, x_joint, p.POSITION_CONTROL, targetPosition=curr_x)
   elif x_angle+one_radian<curr_x:
       p.setJointMotorControl2(planeId, x_joint, p.POSITION_CONTROL, targetPosition=curr_x)
       curr_x-=steeringAngle

   if y_angle-one_radian>curr_y:
       curr_y+=steeringAngle
       p.setJointMotorControl2(planeId, y_joint, p.POSITION_CONTROL, targetPosition=curr_y)
   elif y_angle+one_radian<curr_y:
       curr_y-=steeringAngle
       p.setJointMotorControl2(planeId, y_joint, p.POSITION_CONTROL, targetPosition=curr_y)
	


   p.stepSimulation()
   time.sleep(angle/5)



#Autorotation
#arr=[radians(i) for i in range(0,60,5)]+[radians(i) for i in range(60,-60,5)]+[radians(i) for i in range(-60,0,5)]

"""
for i in range (10000):
  
    p.setJointMotorControl2(planeId, y_joint, p.POSITION_CONTROL, targetPosition=arr[i%len(arr)])
    p.stepSimulation()
    time.sleep(1/5)
"""

p.disconnect()

