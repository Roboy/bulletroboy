import os
import sys
import time
import math
import numpy as np
import pybullet as p
import pybullet_data

sys.path.append("..")

from bulletroboy.operator import Operator, Movements
from bulletroboy.exoforce import CageConfiguration, ExoForce


if __name__ == '__main__':
    file_path = os.path.dirname(os.path.realpath(__file__))

    # SIMULATION SETUP
    p.connect(p.GUI)
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)     # Don't change. Else p.applyExternalForce() won't work.

    # LOADING SIM BODIES
    plane = p.loadURDF("plane.urdf")
    human_model = p.loadURDF(file_path + "/../models/human.urdf", [0, 0, 1], useFixedBase=1)

    # EXOFORCE SETUP
    initial_cage_conf = CageConfiguration(file_path + "/../config/cageConfiguration.xml")
    operator = Operator(human_model)

    exoforce = ExoForce(initial_cage_conf, operator)

    # DEBUG PARAMETERS
    for tendon in exoforce.get_tendons():
        tendon.forceId = p.addUserDebugParameter("Force in " + tendon.name, 0, 200, 0)

    automatic_cage_rotation = True
    issue = False

    if automatic_cage_rotation == False:
        cage_angle_id = p.addUserDebugParameter("Cage Angle", -180, 180, 0)

    # RUN SIM
    try:
        while True:
            t = time.time()
            mv = Movements(operator)

            if issue == False:
                # Call movement function
                issue = mv.simple_move('forearm_roll')

            motor_forces = []
            for tendon in exoforce.get_tendons():
                motor_forces.append(p.readUserDebugParameter(tendon.forceId))

            if automatic_cage_rotation == False:
            	cage_angle = p.readUserDebugParameter(cage_angle_id)
            else:
                cage_angle = 0
            exoforce.update(cage_angle, motor_forces, automatic_cage_rotation)

            p.stepSimulation()

    except KeyboardInterrupt:
        pass

