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

def generate_workspace(x, y, z, end_efector, tendon_groups, force_vector=np.array([100, 0, 0])):
    print(f"Generating workspace for [{end_efector}] \twith force {force_vector}...")
    start = time.time()

    tendons = tendon_groups[end_efector]

    xx = x.reshape(-1)
    yy = y.reshape(-1)
    zz = z.reshape(-1)

    wx = np.zeros_like(xx)
    wy = np.zeros_like(yy)
    wz = np.zeros_like(zz)

    for i, (mx, my, mz) in enumerate(zip(xx, yy, zz)):
        point = np.array([mx, my, mz])
        forces = force_decomposition(force_vector, point, tendons)
        if forces:
            if force_vector[0] > 0: wx[i] = 1
            if force_vector[1] > 0: wy[i] = 1
            if force_vector[2] > 0: wz[i] = 1

    ws = wx + wy + wz
    ws_size = ws[ws>0].size
    ws_percentage = np.around(ws_size / x.size, 2)

    print(f"Done! [{np.around(time.time() - start, 3)} s] available workspace {ws_percentage * 100} %\n")
    return wx.reshape(x.shape), wy.reshape(y.shape), wz.reshape(z.shape)


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

    # # force beeing applied to all joints
    # force = np.array([100, 0, 0])

    # # points where the force is going to be applied
    # human_attachment_points = [np.array([0, 0, 0]) for _ in tendon_groups]

    # for attachment_point, joint in zip(human_attachment_points, tendon_groups):
    #     forces = force_decomposition(force, attachment_point, tendon_groups[joint])
    #     print(f"Decomposed force {force} for joint [{joint}]:")
    #     for tendon in forces:
    #         print(f"\t- tendon {tendon}: \t{np.around(forces[tendon],2):>6} Nm\n")


    #############     PLOT WORKSPACE     #############

    height = cage_conf.cage_structure['height']
    radius = cage_conf.cage_structure['radius']
    stepsize = 0.2

    x, y, z = np.meshgrid(np.arange(-radius, radius, stepsize),
                        np.arange(-radius, radius, stepsize),
                        np.arange(0, height, stepsize))


    forces = np.array([[100, 0, 0],
                        [0, 100, 0],
                        [0, 0, 100]])


    fig = plt.figure(figsize=plt.figaspect(0.3))
    fig.suptitle('ExoForce workspace', fontsize=16)
    for i, force in enumerate(forces):
        ax = fig.add_subplot(1, len(forces), i+1, projection='3d')
        ax.set(xlim=(-radius, radius),
               ylim=(-radius, radius),
               zlim=(0,height))
        ax.set_title(f"Force = {force}")
        ax.set_xlabel("X axis")
        ax.set_ylabel("Y axis")
        ax.set_zlabel("Height")

        lx, ly, lz = generate_workspace(x, y, z, "human/left_wrist", tendon_groups, force)
        rx, ry, rz = generate_workspace(x, y, z, "human/right_wrist", tendon_groups, force)

        ax.quiver(x, y, z, lx, ly, lz, length=0.1, normalize=True, color='b', label="Left Wrist")
        ax.quiver(x, y, z, rx, ry, rz, length=0.1, normalize=True, color='r', label="Right Wrist")
        ax.legend()

    plt.show()
