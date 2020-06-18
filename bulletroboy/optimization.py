import numpy as np
import os
from scipy.optimize import minimize
import time
import matplotlib.pyplot as plt

from utils import CageConfiguration

MIN_FORCE = 0.1

def force_decomposition(force_vector, force_point, tendons):
    force_direction = force_vector / np.linalg.norm(force_vector)

    max_forces = [int(tendons[id]['max_force']) for id in tendons]
    attachment_points = [tendons[id]['point'] for id in tendons]
    tendon_vectors = attachment_points - force_point
    tendon_directions = tendon_vectors / np.linalg.norm(tendon_vectors, axis=1, keepdims=True)

    # print("\n\n\nmost important:\n", force_point, "\n\n\n")

    # projecting vector directions to force direction, only tendons with positive values, will be part of the decomposition
    projections = tendon_directions.dot(force_direction)
    active_tendons = tendon_directions[projections > 0]
    active_max_forces = np.asarray(max_forces)[projections > 0]

    forces = None
    if len(active_tendons) > 0:
        initial_forces = np.random.rand(len(active_tendons)) * np.mean(force_vector)
        constraints = [{'type':'eq', 'fun': lambda x: np.sum(active_tendons.T * x, axis=1) - force_vector},
                    {'type':'ineq', 'fun': lambda x: active_max_forces - x}]
        solution = minimize(lambda x: np.sqrt(np.sum(np.square(x))),
                                initial_forces,
                                constraints=constraints, options={'maxiter': 200}
                                )

        if solution.success:
            forces = {id: force for id, force in zip(tendons, solution.x) if force > MIN_FORCE}
    
    # if not forces:
    #     print(f"WARNING: Cannot decompose the forces: [ {solution.message} ]")
        
    return forces


if __name__ == "__main__":
    ######   GETTING TENDONS FROM CAGE CONFIGURATION FILE   ######

    cage_conf_file_path = os.path.dirname(os.path.realpath(__file__)) + "/../config/cageConfiguration.xml"
    cage_conf = CageConfiguration(cage_conf_file_path)

    # print(cage_conf)

    tendon_groups = {}
    for muscle in cage_conf.muscle_units:
        attachment_point = muscle['viaPoints'][-1]
        link_name = attachment_point['link']
        if link_name not in tendon_groups:
            tendon_groups[link_name] = {}
        tendon_groups[link_name][muscle['id']] = {}
        tendon_groups[link_name][muscle['id']]['point'] = muscle['viaPoints'][0]['point']
        tendon_groups[link_name][muscle['id']]['max_force'] = muscle['parameters']['max_force']

    print(f"\n{len(tendon_groups)} joints with tendons attached found in ({cage_conf_file_path}):")
    for joint in tendon_groups:
        print(f"\t- {joint}\t: {len(tendon_groups[joint])} tendons {[id for id in tendon_groups[joint]]}")
    print("\n")


    #############      APPLY FORCES TEST      #############

    # force beeing applied to all joints
    force = np.array([100, 0, 0])

    # points where the force is going to be applied
    human_attachment_points = [np.array([0, 0, 0]) for _ in tendon_groups]

    for attachment_point, joint in zip(human_attachment_points, tendon_groups):
        forces = force_decomposition(force, attachment_point, tendon_groups[joint])
        print(f"Decomposed force {force} for joint [{joint}]:")
        for tendon in forces:
            print(f"\t- tendon {tendon}: \t{np.around(forces[tendon],2):>6} Nm\n")

