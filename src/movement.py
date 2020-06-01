import os
import time
import math
import pybullet as p
import pybullet_data

from exoforce import CageConfiguration, Operator, ExoForce, Movements

			
if __name__ == '__main__':	
    file_path = os.path.dirname(os.path.realpath(__file__))

    # SIMULATION SETUP
    p.connect(p.GUI)
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)     # Don't change. Else p.applyExternalForce() won't work.

    # LOADING SIM BODIES
    plane = p.loadURDF("plane.urdf")
    human_model = p.loadURDF(file_path + "/../models/human.urdf", [0, 0, 1], useFixedBase=1)

    # EXOFORCE SETUP
    initial_cage_conf = CageConfiguration(file_path + "/../config/cageConfiguration.xml")
    operator = Operator(human_model)

    exoforce = ExoForce(initial_cage_conf, operator)

    # TESTING CHANGE IN INITIAL SETTING POSE: (TODO: delete before merge)
    test_link = operator.get_link_index('human/left_shoulder_1')
    for i in range(p.getNumJoints(human_model)):
       print(p.getJointInfo(human_model, i))
    
    print('The link for left_elbow is: 	', test_link)
    print('\n')

    link_state = p.getLinkState(human_model, test_link)
    print('The link_state for left_shoulder is: 	', link_state)
    print('\n')

    joint_state = p.getJointState(human_model, test_link)
    print('The joint_state for left_shoulder is: 	', joint_state)
    print('\n')

    joint_state = p.getJointState(human_model, test_link)
    print('The reset_joint_state for left_shoulder is: 	', joint_state)
    print('\n')

    # DEBUG PARAMETERS
    for tendon in exoforce.get_tendons():
        tendon.forceId = p.addUserDebugParameter("Force in " + tendon.name, 0, 200, 0)

    cage_angle_id = p.addUserDebugParameter("Cage Angle", -180, 180, 0)

    # RUN SIM
    try:
        while True:
            t = time.time()
            mv = Movements(human_model)

	    # Define link (one string or a list of 2 strings)
            """link = []
            link.append(b'human/left_hand')
            link.append(b'human/right_hand')"""
            link = b'human/left_hand'

            # Define position (a vec3 or a list of two vec3s):
            """pos = []
            pos.append([0.2 * math.cos(t), 0.7, 0.2 * math.sin(t) + 1.2])
            pos.append([0.2 * math.cos(t),-0.7, 0.2 * math.sin(t) + 1.2])"""
            pos = [0.7, 0.2 * math.cos(t), 0.2 * math.sin(t) + 1.2]

            maxIter = 100

            # Call movement function:
            """mv.multiple_end_effectors(link, pos, maxIter, chest_constraint = True)"""
            # mv.one_end_effector(link, pos, maxIter, chest_constraint = True)
            mv.simple_move('side_swing')

            motor_forces = []
            for tendon in exoforce.get_tendons():
                motor_forces.append(p.readUserDebugParameter(tendon.forceId))

            cage_angle = p.readUserDebugParameter(cage_angle_id)
            exoforce.update(cage_angle, motor_forces)

            p.stepSimulation()

    except KeyboardInterrupt:
        pass

