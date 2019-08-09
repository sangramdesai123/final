import pybullet as p
import time
import pybullet_data


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
spiderStartPos = [0,0,0.05]
spiderStartOrientation = p.getQuaternionFromEuler([0,0,0])
spiderId = p.loadURDF("spider.urdf", spiderStartPos, spiderStartOrientation)


#Orignal fps = 1/100
fps = 1/100

for i in range(p.getNumJoints(spiderId)):
  print(p.getJointInfo(spiderId, i))

#steering=[0,1,3,5,6,8,10,11,13,15,16,18]

angles=[0 for i in range(p.getNumJoints(spiderId))]
print(angles)
front_right = [0,1,3]
front_left = [5,6,8]
back_right = [10,11,13]
back_left = [15,16,18]

#0.5 degrees
steer = 0.00872665

def stand(p, steer, fps, angles, spiderId, front_right, front_left, back_right, back_left):

  steering = [front_right[1],front_left[1],back_right[1],back_left[1]]
  direction = [-1,1,-1,1]
  
  for i in range(50):
    for j in range(4):
      angles[steering[j]] = steer*direction[j]*(i+1)
      p.setJointMotorControl2(spiderId, steering[j], p.POSITION_CONTROL, targetPosition=steer*direction[j]*(i+1))
      p.stepSimulation()
    time.sleep(fps)

  
  steering = [front_right[2],front_left[2],back_right[2],back_left[2]]
  direction = [1,-1,1,-1]
  
  for i in range(200):
    for j in range(4):
      angles[steering[j]] = steer*direction[j]*(i+1)
      p.setJointMotorControl2(spiderId, steering[j], p.POSITION_CONTROL, targetPosition=steer*direction[j]*(i+1))
      p.stepSimulation()
    time.sleep(fps)
    

def walk(p, steer, fps, angles, spiderId, front_right, front_left, back_right, back_left):

  #Lift Front Right Leg
  direction = -1
  leg = front_right
  joint = leg[2]

  lifting = 50
  rotating = 50
  
  for i in range(lifting):
    #print(p.getJointState(spiderId, joint))
    angles[joint]+=steer*direction
    p.setJointMotorControl2(spiderId, joint, p.POSITION_CONTROL, targetPosition=angles[joint])
    p.stepSimulation()
    time.sleep(fps)

  direction = 1
  joint = leg[0]
  
  for i in range(rotating):
    #print(p.getJointState(spiderId, joint))
    angles[joint]+=steer*direction
    p.setJointMotorControl2(spiderId, joint, p.POSITION_CONTROL, targetPosition=angles[joint])
    p.stepSimulation()
    time.sleep(fps)
  
  joint = leg[2]
  
  for i in range(lifting):
    #print(p.getJointState(spiderId, joint))
    angles[joint]+=steer*direction
    p.setJointMotorControl2(spiderId, joint, p.POSITION_CONTROL, targetPosition=angles[joint])
    p.stepSimulation()
    time.sleep(fps)


  #Lift Front Left Leg
  direction = 1
  leg = front_left
  joint = leg[2]
  
  for i in range(lifting):
    #print(p.getJointState(spiderId, joint))
    angles[joint]+=steer*direction
    p.setJointMotorControl2(spiderId, joint, p.POSITION_CONTROL, targetPosition=angles[joint])
    p.stepSimulation()
    time.sleep(fps)

  direction = -1
  joint = leg[0]
  
  for i in range(rotating):
    #print(p.getJointState(spiderId, joint))
    angles[joint]+=steer*direction
    p.setJointMotorControl2(spiderId, joint, p.POSITION_CONTROL, targetPosition=angles[joint])
    p.stepSimulation()
    time.sleep(fps)
  
  joint = leg[2]
  
  for i in range(lifting):
    #print(p.getJointState(spiderId, joint))
    angles[joint]+=steer*direction
    p.setJointMotorControl2(spiderId, joint, p.POSITION_CONTROL, targetPosition=angles[joint])
    p.stepSimulation()
    time.sleep(fps)

  
  #Lift Back Right Leg
  direction = -1
  leg = back_right
  joint = leg[2]
  
  for i in range(lifting):
    #print(p.getJointState(spiderId, joint))
    angles[joint]+=steer*direction
    p.setJointMotorControl2(spiderId, joint, p.POSITION_CONTROL, targetPosition=angles[joint])
    p.stepSimulation()
    time.sleep(fps)

  direction = 1
  joint = leg[0]
  
  for i in range(rotating):
    #print(p.getJointState(spiderId, joint))
    angles[joint]+=steer*direction
    p.setJointMotorControl2(spiderId, joint, p.POSITION_CONTROL, targetPosition=angles[joint])
    p.stepSimulation()
    time.sleep(fps)
  
  joint = leg[2]
  
  for i in range(lifting):
    #print(p.getJointState(spiderId, joint))
    angles[joint]+=steer*direction
    p.setJointMotorControl2(spiderId, joint, p.POSITION_CONTROL, targetPosition=angles[joint])
    p.stepSimulation()
    time.sleep(fps)


  #Lift Back Left Leg
  direction = 1
  leg = back_left
  joint = leg[2]
  
  for i in range(lifting):
    #print(p.getJointState(spiderId, joint))
    angles[joint]+=steer*direction
    p.setJointMotorControl2(spiderId, joint, p.POSITION_CONTROL, targetPosition=angles[joint])
    p.stepSimulation()
    time.sleep(fps)

  direction = -1
  joint = leg[0]
  
  for i in range(rotating):
    #print(p.getJointState(spiderId, joint))
    angles[joint]+=steer*direction
    p.setJointMotorControl2(spiderId, joint, p.POSITION_CONTROL, targetPosition=angles[joint])
    p.stepSimulation()
    time.sleep(fps)
  
  joint = leg[2]
  
  for i in range(lifting):
    #print(p.getJointState(spiderId, joint))
    angles[joint]+=steer*direction
    p.setJointMotorControl2(spiderId, joint, p.POSITION_CONTROL, targetPosition=angles[joint])
    p.stepSimulation()
    time.sleep(fps)

  
  #Move Body Ahead
  direction = [-1,1,-1,1]
  joints=[front_right[0], front_left[0], back_right[0], back_left[0]]
  #print(joints)
  for i in range(rotating):
    for j in range(len(joints)):
      angles[joints[j]]+=steer*direction[j]
      p.setJointMotorControl2(spiderId, joints[j], p.POSITION_CONTROL, targetPosition=angles[joints[j]])
    p.stepSimulation()
    time.sleep(fps)
  

  #time.sleep(100)
  


start = 0

for i in range(10000):

    #p.setJointMotorControl2(boxId, steer, p.POSITION_CONTROL, targetPosition=steeringAngle)
    if start == 0:
      stand(p, steer, fps, angles, spiderId, front_right, front_left, back_right, back_left)
      start = 1
    else:
      walk(p, steer, fps, angles, spiderId, front_right, front_left, back_right, back_left)
      #break

    p.stepSimulation()
    time.sleep(fps)
    
cubePos, cubeOrn = p.getBasePositionAndOrientation(spiderId)
print(cubePos,cubeOrn)
p.disconnect()


"""
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print("current_dir=" + currentdir)
parentdir = os.path.join(currentdir, "../gym")

os.sys.path.insert(0, parentdir)

import pybullet as p
import pybullet_data

import time

cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
  p.connect(p.GUI)

p.resetSimulation()
p.setGravity(0, 0, -10)

useRealTimeSim = 1

#for video recording (works best on Mac and Linux, not well on Windows)
#p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "racecar.mp4")
p.setRealTimeSimulation(useRealTimeSim)  # either this
#p.loadURDF("plane.urdf")
p.loadSDF(os.path.join(pybullet_data.getDataPath(), "stadium.sdf"))

car = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "racecar/racecar.urdf"))
for i in range(p.getNumJoints(car)):
  print(p.getJointInfo(car, i))

inactive_wheels = [3, 5, 7]
wheels = [2]

for wheel in inactive_wheels:
  p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

steering = [4, 6]

targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -10, 10, 0)
maxForceSlider = p.addUserDebugParameter("maxForce", 0, 10, 10)
steeringSlider = p.addUserDebugParameter("steering", -0.5, 0.5, 0)
while (True):
  maxForce = p.readUserDebugParameter(maxForceSlider)
  targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
  steeringAngle = p.readUserDebugParameter(steeringSlider)
  #print(targetVelocity)

  for wheel in wheels:
    p.setJointMotorControl2(car,
                            wheel,
                            p.VELOCITY_CONTROL,
                            targetVelocity=targetVelocity,
                            force=maxForce)

  for steer in steering:
    p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steeringAngle)

  steering
  if (useRealTimeSim == 0):
    p.stepSimulation()
  time.sleep(0.01)

  
"""
