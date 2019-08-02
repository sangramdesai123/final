import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,2]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
bipedalId = p.loadURDF("bipedal2.urdf",cubeStartPos, cubeStartOrientation)


for i in range(p.getNumJoints(bipedalId)):
  print(p.getJointInfo(bipedalId, i))

maxForce = 50

steering=[1,3]

steeringAngle = 3.1428/4

targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -50, 50, 0)
maxForceSlider = p.addUserDebugParameter("maxForce", 0, 10, 10)
steeringSlider = p.addUserDebugParameter("steering", -0.5, 0.5, 0)
angle=0
for i in range (1000000):
    angle+=0.01
    p.setJointMotorControl2(bipedalId, 0, p.POSITION_CONTROL, targetPosition=angle)
    #p.setJointMotorControl2(bipedalId, 3, p.POSITION_CONTROL, targetPosition=0)
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(bipedalId)
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
