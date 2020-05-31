import os
import pybullet as p
import pybullet_data

from exoforce import CageConfiguration, Operator, ExoForce

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

    cage_angle_id = p.addUserDebugParameter("Cage Angle", -180, 180, 0)

    # RUN SIM
    try:
        while True:
            cage_angle = p.readUserDebugParameter(cage_angle_id)

            motor_forces = []
            for tendon in exoforce.get_tendons():
                motor_forces.append(p.readUserDebugParameter(tendon.forceId))
            
            exoforce.update(cage_angle, motor_forces)

            p.stepSimulation()

    except KeyboardInterrupt:
        pass
