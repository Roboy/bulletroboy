from msilib import sequence
import pybullet as p
import time
import numpy as np

import pybullet_data

import yaml

import subprocess
import sys
import os

from scipy.spatial.transform import Rotation

# TODO 
# + expose max force max velocity params
# save plot
# + get rid of relative paths
# + integrate IK control
# + change camera orientation
# cleaner plotting
# + endeffector weight
# + requirements.txt

ROOT_DIR=os.path.dirname(os.path.realpath(__file__))
MODELS_DIR="C:\\Users\\AlonaKharchenko\\Documents\\code\\roboy3_models"

p.connect(p.GUI_SERVER)
p.setRealTimeSimulation(1)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS,0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.resetDebugVisualizerCamera( cameraDistance=2, cameraYaw=-180, cameraPitch=0, cameraTargetPosition=[0,0,1])


maxForceParam = p.addUserDebugParameter("Actuator max force", 0,150,100)
maxForce = 100
maxVelocityParam = p.addUserDebugParameter("Actuator max velocity",0,10,1)
maxVelocity = 1
endeffectorWeightParam = p.addUserDebugParameter("Endeffector weight", 0,5,0.2)

humanoid = p.loadURDF(f"{MODELS_DIR}\\pre-robody\\model.urdf", useFixedBase=1)
p.changeDynamics(humanoid, -1, linearDamping=0, angularDamping=0)

# start other scripts
subprocess.Popen(['python', f"{ROOT_DIR}\\logger.py"])

saveButton = p.addUserDebugParameter("Save joint config",1,0,0)
# manualControlButton = p.addUserDebugParameter("Turn off trajectory mode",1,0,0)
manualControlIKInputButton = p.addUserDebugParameter("Toggle manual IK control",1,0,0)

playTrajectoryButton = p.addUserDebugParameter("Play trajectory",1,0,0)
gravId = p.addUserDebugParameter("gravity", -10, 10, -10)
jointIds = []
relevantJointIds = []
paramIds = []

for j in range(p.getNumJoints(humanoid)):
  p.changeDynamics(humanoid, j, linearDamping=0, angularDamping=0)
  info = p.getJointInfo(humanoid, j)
  #print(info)
  jointName = info[1]
  jointType = info[2]
  if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
    jointIds.append(j)
    if ("Right" in jointName.decode("utf-8")):
      p.enableJointForceTorqueSensor(humanoid,j)
      relevantJointIds.append(j)
      paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, 0))
  
joint_poses = {}
with open(f"{ROOT_DIR}\\..\\config\\saved_joint_states.yaml") as f:    
  loaded = yaml.load(f, Loader=yaml.FullLoader)
  if loaded is not None:
    joint_poses = loaded

j = 0
prevButton = 0
prevmanualControlButton = 0
prevTrajectoryButton = 0
manualControlMode = True
userDataMode = p.addUserData(humanoid, "manualControlMode", str(manualControlMode))
# print(f"userDataMode: {userDataMode}")

def setJointMotorControl(idx, targetPosition):
  if manualControlMode:
    p.setJointMotorControl2(humanoid, idx, p.POSITION_CONTROL, targetPosition) #, force=200, maxVelocity=10)
  else: 
    p.setJointMotorControl2(humanoid, idx, p.POSITION_CONTROL, targetPosition, force=maxForce, maxVelocity=maxVelocity)

def accurateJointControl(body, jointPoses, threshold=1, maxIter=1000):
  closeEnough = False
  iter = 0
  dist2 = 1e2
  while (not closeEnough): # and iter < maxIter):
    k=0
    for idx in relevantJointIds:
      # TODO: keep position setpoint until the joint position is reached
      setJointMotorControl(idx, float(jointPoses[k]))
      # p.setJointMotorControl2(body, idx, p.POSITION_CONTROL, float(jointPoses[k]), force=50, maxVelocity=1)
      # if k == 0:
      #   jointReactionForces = p.getJointState(humanoid, idx)[2]
      #   # print(jointReactionForces)
      k += 1
    newPos = np.array([p.getJointState(body,joint)[0] for joint in relevantJointIds])
    dist = np.linalg.norm(np.array(jointPoses)-newPos)
    
    # ls = p.getLinkState(ob, endEffectorId)
    # newPos = ls[4]
    # diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
    # dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
    closeEnough = (dist < threshold)
    # print(f"{closeEnough}\t{dist}\t{iter}")
    iter = iter + 1

def accurateCalculateInverseKinematics(ob, endEffectorId, targetPos, threshold=0.01, maxIter=80, targetOrn=None):
    closeEnough = False
    iter = 0
    dist2 = 1e10
    while (not closeEnough and iter < maxIter):
        if targetOrn is None:
            jointPoses = p.calculateInverseKinematics(ob, endEffectorId, targetPos)
        else:
            jointPoses = p.calculateInverseKinematics(ob, endEffectorId, targetPos, targetOrientation=targetOrn)
        
        for idx,pos in zip(freeJoints,jointPoses): 
            setJointMotorControl(idx,pos)
            # p.setJointMotorControl2(bodyIndex=ob,
            #                             jointIndex=idx,
            #                             controlMode=p.POSITION_CONTROL,
            #                             targetPosition=pos,
            #                             maxVelocity=8.0)#jointPoses[i])
            #                             # target_posVelocity=0,
            #                             # force=1,
            #                             # positionGain=5,
            #                             # velocityGain=0.1)
        ls = p.getLinkState(ob, endEffectorId)
        newPos = ls[4]
        diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
        dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
        closeEnough = (dist2 < threshold)
        iter = iter + 1
    #print ("Num iter: "+str(iter) + "threshold: "+str(dist2))
    return jointPoses

# IK keyboard control setup
freeJoints = []
joint_names = {}
efs = {}
numJoints = p.getNumJoints(humanoid)
for i in range(numJoints):
  info = p.getJointInfo(humanoid,i)
  if info[2] == p.JOINT_REVOLUTE:
      freeJoints.append(i)
      joint_names[i] = (p.getJointInfo(0, i)[1].decode("utf-8"))
  if info[12] == b'EndEffector_Right':
      endEffectorId = i
      print("EF id: " + str(i))
      efs["EndEffector_Right"] = i
  if info[12] == b'EndEffector_Left':
      efs["EndEffector_Left"] = i

_link_name_to_index = {p.getBodyInfo(humanoid)[0].decode('UTF-8'):-1,}
        
for _id in range(p.getNumJoints(humanoid)):
  _name = p.getJointInfo(humanoid, _id)[12].decode('UTF-8')
  _link_name_to_index[_name] = _id

def linkIdFromName(name):
  return _link_name_to_index[name]


rightHandId = linkIdFromName("EndEffector_Right")
target_pos = list(p.getLinkState(humanoid, rightHandId)[0])
start_pos = list(p.getLinkState(humanoid, rightHandId)[0])

target_orn = Rotation.from_quat((p.getLinkState(humanoid, rightHandId)[1])).as_euler('xyz', degrees=True)

pos_shift = 0.001
orn_shift = 1

debugLine = p.addUserDebugLine(start_pos, target_pos, [1,0,0], 1)
prevPos = None

IKControlKeys = ['d','a','w','x','q','c','y','n','e','g','j','m','t']
manualControlIKInput = False
prevManualControlIKInputButton = 0

def keyboardIKControl(keys):
  # for key in keys:
  if ord('d') in keys:
      target_pos[0] -= pos_shift
  if ord('a') in keys:
      target_pos[0] += pos_shift
  if ord('w') in keys: #p.B3G_LEFT_ARROW
      target_pos[1] -= pos_shift
  if ord('x') in keys: #p.B3G_RIGHT_ARROW
      target_pos[1] += pos_shift
  if ord('q') in keys: #p.B3G_UP_ARROW
      target_pos[2] += pos_shift
  if ord('c') in keys: # p.B3G_DOWN_ARROW
      target_pos[2] -= pos_shift


  if ord('y') in keys:
      target_orn[0] -= orn_shift
  if ord('n') in keys:
      target_orn[0] += orn_shift
  if ord('g') in keys: #p.B3G_LEFT_ARROW
      target_orn[1] -= orn_shift
  if ord('j') in keys: #p.B3G_RIGHT_ARROW
      target_orn[1] += orn_shift
  if ord('m') in keys: #p.B3G_UP_ARROW
      target_orn[2] += orn_shift
  if ord('t') in keys: # p.B3G_DOWN_ARROW
      print(target_orn)
      target_orn[2] -= orn_shift
      print(target_orn)

  accurateCalculateInverseKinematics(humanoid, rightHandId, target_pos, targetOrn=Rotation.from_euler('xyz',target_orn, degrees=True).as_quat())
      
  debugLine = p.addUserDebugLine(start_pos, target_pos, [1,0,0], 1, lifeTime=0.05)

while (1): 

  p.setGravity(0, 0, p.readUserDebugParameter(gravId))
  maxForce = p.readUserDebugParameter(maxForceParam)
  maxVelocity = p.readUserDebugParameter(maxVelocityParam)
  p.changeDynamics(humanoid, rightHandId, mass=p.readUserDebugParameter(endeffectorWeightParam))

  # if p.readUserDebugParameter(manualControlButton) > prevmanualControlButton:
  #   manualControlMode = True
  #   p.addUserData(humanoid, "manualControlMode", str(manualControlMode))
  #   prevmanualControlButton = p.readUserDebugParameter(manualControlButton)
  #   print(f"Manual control: {manualControlMode}")

  if manualControlMode:
    keys = p.getKeyboardEvents()
    if p.readUserDebugParameter(manualControlIKInputButton) > prevManualControlIKInputButton:
      manualControlIKInput = not manualControlIKInput
      prevManualControlIKInputButton = p.readUserDebugParameter(manualControlIKInputButton)
      print(f"IK control input enabled: {manualControlIKInput}")
    
    if manualControlIKInput:
      keyboardIKControl(keys)
    else:
      for i in range(len(paramIds)):
        c = paramIds[i]
        targetPos = p.readUserDebugParameter(c)
        setJointMotorControl(relevantJointIds[i], targetPos)
      
  if p.readUserDebugParameter(saveButton) > prevButton:
    j += 1
    name = input("Enter config name: ")
    prevButton = p.readUserDebugParameter(saveButton)
    poses = p.getJointStates(humanoid, relevantJointIds)[0]
    joint_poses[name] = [p.getJointState(humanoid, id)[0] for id in relevantJointIds]

    with open(f"{ROOT_DIR}//..//config//saved_joint_states.yaml", "w") as f:
      data = yaml.dump(joint_poses, f)
    
    print(f"Saved joint config as {name}")

  if p.readUserDebugParameter(playTrajectoryButton) > prevTrajectoryButton:
    prevTrajectoryButton = p.readUserDebugParameter(playTrajectoryButton)
    with open(f"{ROOT_DIR}\\..\\config\\trajectory.yaml") as file:
      trajectory = yaml.load(file, Loader=yaml.FullLoader)

    time.sleep(1)
    manualControlMode = False
    p.addUserData(humanoid, "manualControlMode", str(manualControlMode))
    
    for pose in trajectory:
      pose = str(pose)
      k=0
      selected_joint_poses = joint_poses[pose]
      accurateJointControl(humanoid, selected_joint_poses)
      
      time.sleep(1)
    manualControlMode = True
    p.addUserData(humanoid, "manualControlMode", str(manualControlMode))

with open('saved_joint_states.yaml', "w") as f:
  data = yaml.dump(joint_poses, f)
  time.sleep(0.01)
