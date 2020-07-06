import argparse
import os
import pybullet as p
import pybullet_data
import rclpy
import time
import math
import numpy as np
import time



def is_valid_file(parser, arg):
    file_path = os.path.dirname(os.path.realpath(__file__))
    file_path += "/" + arg
    if not os.path.exists(file_path):
        parser.error("The file %s does not exist!" % file_path)
    else:
        return file_path #return open(arg, 'r')  # return an open file handle


def get_joints(model):
    joints = []
    for i in range(p.getNumJoints(model)):
        joint = {}
        joint['name'] = str(p.getJointInfo(model, i)[1], 'utf-8')
        joint['id'] = i
        joint['type'] = p.getJointInfo(model, i)[2]
        if joint['type'] is p.JOINT_REVOLUTE:
            joints.append(joint)
    return joints


def get_joint_indices(joint_list, joint_names):
    indices = []
    for j, joint_name in enumerate(joint_names):
        for i, joint in enumerate(joint_list):
            if joint['name'] == joint_name:
                indices.append(joint['id'])
                break
    return indices


def get_joint_positions(model, joints):

    joint_states = p.getJointStates(human_model, joints)
    return joint_states


def get_link_positions(model, link_indices):

    link_states = p.getLinkStates(model, link_indices)
    link_positions = []
    for link_state in link_states:
        link_positions.append(link_state[0])

    return np.array(link_positions)


def move_human(human_model, human_joint_indices, case):
    
    t = time.time()
    
    if case == 1:
        p.resetJointState(human_model, human_joint_indices[5], -1.75*math.pi/4)
        p.resetJointState(human_model, human_joint_indices[1], 1.75*math.pi/4)
        p.resetJointState(human_model, human_joint_indices[7], (((math.sin(3*t)+1)/8) + (11/8))*math.pi)
        p.resetJointState(human_model, human_joint_indices[4], ((-(math.cos(3*t)+1)/8) + (1/8))*math.pi)
        p.resetJointState(human_model, human_joint_indices[3], -(((math.sin(3*t+math.pi)+1)/8) + (11/8))*math.pi)
        p.resetJointState(human_model, human_joint_indices[0], ((-(math.cos(3*t+math.pi)+1)/8) + (1/8))*math.pi)
    
    elif case == 2:
        p.resetJointState(human_model, human_joint_indices[5], math.sin(t))
        p.resetJointState(human_model, human_joint_indices[4], math.sin(t+math.pi/2))
        p.resetJointState(human_model, human_joint_indices[1], -math.sin(t))
        p.resetJointState(human_model, human_joint_indices[0], math.sin(t+math.pi/2))
    
    elif case == 3:
        p.setJointMotorControlArray(human_model, [human_joint_indices[0], human_joint_indices[1], human_joint_indices[2]], p.POSITION_CONTROL, [40.0,20.0,0.0])


def move_roboy(roboy_model, roboy_joint_indices, roboy_joint_targets):
    
    p.setJointMotorControlArray(roboy_model, roboy_joint_indices, p.POSITION_CONTROL, roboy_joint_targets)


def main():

    #Parse for optional arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mode", dest="mode", default="debug", help="execution mode: [debug]: uses pybullet debug forces [tendon]: uses tendon forces [forces]: uses link forces")
    parser.add_argument("--config-path", dest="config_path", default="../config/cageConfiguration.xml", metavar="FILE", help="path to the cage configuration XML file", type=lambda x: is_valid_file(parser, x) )
    parser.add_argument("--model-path", dest="model_path", default="../models/human.urdf", metavar="FILE", help="path to the human model URDF description", type=lambda x: is_valid_file(parser, x) )
    args = parser.parse_args()

    parser2 = argparse.ArgumentParser()
    parser2.add_argument("--model-path", dest="filename", default="../../roboy3_models/upper_body/bullet.urdf", metavar="FILE", help="path to the model URDF description", type=lambda x: is_valid_file(parser, x) )
    args2 = parser2.parse_args()

    #Set up PyBullet simulation
    p.connect(p.GUI)
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)
    p.setRealTimeSimulation(1)

    # Import models
    p.loadURDF("plane.urdf")
    human_model = p.loadURDF(args.model_path, [0, 0, 1], useFixedBase=1)
    roboy_model = p.loadURDF(args2.filename, [1, 0, 1],  useFixedBase=1)

    #Place models at the same coordinates
    p.resetBasePositionAndOrientation(roboy_model, [0, 0, 1.2], p.getQuaternionFromEuler([0.0,0.0,math.pi/2]))
    
    #Define usable movements
    FOREARM_ROLL = 1
    ARM_SWING = 2
    POSE = 3
    
    #Define used joints and links
    human_link_list = ["right_elbow_0","left_elbow_0","right_wrist_0","left_wrist_0"]
    human_joint_list = ["right_shoulder_0", "right_shoulder_1", "right_shoulder_2", "right_elbow_0", "left_shoulder_0", "left_shoulder_1", "left_shoulder_2", "left_elbow_0"]
    roboy_link_list = ["elbow_right_axis0","elbow_left_axis0","wrist_right_axis0","wrist_left_axis0"]
    
    #Get all non fixed joints from models 
    human_joints = get_joints(human_model)
    roboy_joints = get_joints(roboy_model)
    
    #Get human link/joint indices for the custom link list
    human_joint_indices = get_joint_indices(human_joints, human_joint_list)
    human_link_indices = get_joint_indices(human_joints, human_link_list)
    
    #Colorize human
    for j, joint in enumerate(human_joints):
        p.changeVisualShape(human_model, joint['id'], rgbaColor=[1,0,0,1])
    
    #Get Roboy link/joint indices for the custom link list
    roboy_link_indices = get_joint_indices(roboy_joints, roboy_link_list)
    roboy_joint_indices = []
    for joint in roboy_joints:
        roboy_joint_indices.append(joint['id'])
    
    #Colorize Roboy
    for j, joint in enumerate(roboy_joints):
        p.changeVisualShape(roboy_model, joint['id'], rgbaColor=[1,1,0,1])    

    counter = 0
    distances = []

    # RUN SIM    
    try:
        while True:
            
            #Move human and get feature point positions
            move_human(human_model, human_joint_indices, FOREARM_ROLL)           
            human_link_positions = get_link_positions(human_model, human_link_indices)
            
            #Calculate and set joint positions for right arm
            jointPoses1 = p.calculateInverseKinematics(roboy_model, roboy_link_indices[0], human_link_positions[0])            
            jointPoses2 = p.calculateInverseKinematics(roboy_model, roboy_link_indices[2], human_link_positions[2])
            jointPosesMean1 = (np.array(jointPoses1) + np.array(jointPoses2))/2.0
            for i in range(len(roboy_joint_indices)):
                p.resetJointState(roboy_model, roboy_joint_indices[i], jointPosesMean1[i])
            #Calculate and set joint positions for left arm
            jointPoses3 = p.calculateInverseKinematics(roboy_model, roboy_link_indices[1], human_link_positions[1])            
            jointPoses4 = p.calculateInverseKinematics(roboy_model, roboy_link_indices[3], human_link_positions[3])
            jointPosesMean2 = (np.array(jointPoses3) + np.array(jointPoses4))/2.0
            for i in range(len(roboy_joint_indices)):
                p.resetJointState(roboy_model, roboy_joint_indices[i], jointPosesMean2[i])
            
            #Calculate and display sum of distanced between feature points
            link_positions = get_link_positions(roboy_model, roboy_link_indices)
            euclidean_distances = []
            for i in range(len(link_positions)):
                euclidean_distances.append(np.linalg.norm(link_positions[i] - human_link_positions[i]))
            distance = np.linalg.norm(np.array(euclidean_distances))
            print("Sum of distance bewteen feature points " + str(distance))
            
            distances.append(distance)
            counter = counter+1
            if counter is 99:
                counter = 0
                mean = np.mean(distances)
                print("Mean value over 100 cycles " + str(mean))
            
            
                    

    except KeyboardInterrupt:
        rclpy.shutdown()
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()
