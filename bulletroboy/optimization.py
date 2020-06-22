import numpy as np
import os
from scipy.optimize import minimize
from utils import CageConfiguration
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

MIN_FORCE = 0.01

def equalize(forces):
    mean = np.mean(forces)
    distances = forces - mean
    loss = np.sqrt(np.sum(np.square(distances)))
    return loss

def force_decomposition(force_vector, force_point, tendons, attachment_points):
    #start = time.time()
    force_direction = force_vector / np.linalg.norm(force_vector)
    max_forces = [int(tendons[id]['max_force']) for id in tendons]
    tendon_vectors = (attachment_points - np.array([force_point]).T).T
    tendon_directions = tendon_vectors / np.linalg.norm(tendon_vectors, axis=1, keepdims=True)

    # projecting vector directions to force direction, only tendons with positive values, will be part of the decomposition
    projections = tendon_directions.dot(force_direction)
    active_tendons = tendon_directions[projections > 0]
    active_max_forces = np.asarray(max_forces)[projections > 0]

    forces = None
    if len(active_tendons) > 0:
        initial_forces = np.random.rand(len(active_tendons)) * np.mean(force_vector)
        constraints = [{'type':'eq', 'fun': lambda x: np.sum(active_tendons.T * np.array([x]), axis=1) - force_vector},
                       {'type':'ineq', 'fun': lambda x: active_max_forces - x},
                       {'type':'ineq', 'fun': lambda x: x}]
        solution = minimize(equalize, initial_forces, constraints=constraints, options={'maxiter': 200})

        if solution.success:
            forces = {id: force for id, force in zip(tendons, solution.x) if force > MIN_FORCE}
    # else:
    #     # print(f"WARNING: Cannot decompose the forces: [ {solution.message} ]")
    #     forces = None

    #print(f"Time: {time.time() - start} s")
    return forces

######   GETTING TENDONS FROM CAGE CONFIGURATION FILE   ######
cage_conf_file_path = os.path.dirname(os.path.realpath(__file__)) + "/cageConfiguration.xml"
cage_conf = CageConfiguration(cage_conf_file_path)

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

# print("tendongroups", tendon_groups['human/left_wrist'][1]['point'])
# attachment = np.array([])
# for id in tendon_groups['human/left_wrist']:
#     attachment = np.append(attachment, tendon_groups['human/left_wrist'][id]['point'])
#
# attachment = attachment.reshape(-1, 3)

def get_attachment_points(radius, hight, angles, joint):
    attachment_points = np.zeros((3, 8))
    attachment_points[0, 0] = radius
    attachment_points[0, 1] = radius * np.cos(angles[0])
    attachment_points[0, 2] = radius * np.cos(angles[1])
    attachment_points[0, 3] = 0
    attachment_points[0, 4] = radius
    attachment_points[0, 5] = radius * np.cos(angles[0])
    attachment_points[0, 6] = radius * np.cos(angles[1])
    attachment_points[0, 7] = 0
    attachment_points[2, 0:3] = hight
    attachment_points[2, 3] = 1.5
    attachment_points[2, 4:7] = 0
    attachment_points[2, 7] = 1

    if joint=="human/left_wrist":
        attachment_points[1, 0] = 0
        attachment_points[1, 1] = radius * np.sin(angles[0])
        attachment_points[1, 2] = radius * np.sin(angles[1])
        attachment_points[1, 3] = 0
        attachment_points[1, 4] = 0
        attachment_points[1, 5] = radius * np.sin(angles[0])
        attachment_points[1, 6] = radius * np.sin(angles[1])
        attachment_points[1, 7] = 0
    elif joint=="human/right_wrist":
        attachment_points[1, 0] = 0
        attachment_points[1, 1] = -radius * np.sin(angles[0])
        attachment_points[1, 2] = -radius * np.sin(angles[1])
        attachment_points[1, 3] = 0
        attachment_points[1, 4] = 0
        attachment_points[1, 5] = -radius * np.sin(angles[0])
        attachment_points[1, 6] = -radius * np.sin(angles[1])
        attachment_points[1, 7] = 0
    return attachment_points

### define workspace ###
def workspace_desired(stepsize=0.2, joint="human/left_wrist"):
    x, y, z = np.meshgrid(np.arange(-1, 1 + stepsize, stepsize),
                          np.arange(-1, 1 + stepsize, stepsize),
                          np.arange( 0, 2 + stepsize, stepsize))

    xx = x.reshape(-1)
    yy = y.reshape(-1)
    zz = z.reshape(-1)
    ws_array_desired = np.array([])
    ws = np.array([xx, yy, zz])
    for i, (mx, my, mz) in enumerate(zip(xx, yy, zz)):
        point = np.array([mx, my, mz])
        if joint == "human/left_wrist":
            if 0.2 <= np.linalg.norm(point - np.array([0, 0, 1])) <= 0.704 and mx >= 0 and my >= 0:
                ws_array_desired = np.append(ws_array_desired, 1)
            else:
                ws_array_desired = np.append(ws_array_desired, 0)
        elif joint == "human/right_wrist":
            if 0.2 <= np.linalg.norm(point - np.array([0, 0, 1])) <= 0.704 and mx >= 0 and my <= 0:
                ws_array_desired = np.append(ws_array_desired, 1)
            else:
                ws_array_desired = np.append(ws_array_desired, 0)

    ii = np.where(ws_array_desired == 1)[0]
    ws_desired = ws[:, ii]
    return ws_array_desired, ws_desired

def workspace_feasible(stepsize=0.2, joint="human/left_wrist", radius=1, height=2, angles=np.array([np.pi/3, 2*np.pi/3]), forces=np.array([[100, 0, 0]])):
    attachment_point = get_attachment_points(radius, height, angles, joint)
    x, y, z = np.meshgrid(np.arange(-1, 1 + stepsize, stepsize),
                          np.arange(-1, 1 + stepsize, stepsize),
                          np.arange( 0, 2 + stepsize, stepsize))

    xx = x.reshape(-1)
    yy = y.reshape(-1)
    zz = z.reshape(-1)
    ws = np.array([xx, yy, zz])
    ws_array_feasible = np.ones_like(xx)

    for n, force in enumerate(forces):
        print(force)
        for i, (mx, my, mz) in enumerate(zip(xx, yy, zz)):
            if ws_array_feasible[i] == 1:
                point = np.array([mx, my, mz])
                decomposed_forces = force_decomposition(force, point, tendon_groups[joint], attachment_point)
                if decomposed_forces:
                    ws_array_feasible[i] = 1
                else:
                    ws_array_feasible[i] = 0

    ii = np.where(ws_array_feasible == 1)[0]
    ws_feasible = ws[:, ii]
    return ws_array_feasible, ws_feasible

# def cost_function(radius, height, angles):
def cost_function(param):
    radius = param[0]
    height = param[1]
    angles = param[2::]
    forces = np.array([[100, 0, 0],
                       [0, 100, 0],
                       [0, 0, 100],
                       [-100, 0, 0],
                       [0, -100, 0],
                       [0, 0, -100]])
    ws_array_desired, _ = workspace_desired()
    ws_array_feasible, _ = workspace_feasible(radius=radius, height=height, angles=angles, forces=forces)
    loss = np.dot((ws_array_desired - ws_array_feasible), (ws_array_desired - ws_array_feasible))
    print(loss)
    print(param)
    return loss


initial_forces = np.array([1, 2, np.pi/3, 2*np.pi/3])
constraint = [{'type': 'ineq', 'fun': lambda x: x}]
solution = minimize(cost_function, initial_forces, method='Nelder-Mead', options={'maxiter': 10})
# solution = minimize(cost_function, initial_forces, constraints=constraint, options={'maxiter': 5})
print(solution)
radius = solution.x[0]

######################################################################
######################################################################
## display workspace under certain conditifullspace & wsdict[i]ons ##
forces = np.array([[100, 0, 0],
                   [0, 100, 0],
                   [0, 0, 100],
                   [-100, 0, 0],
                   [0, -100, 0],
                   [0, 0, -100]])

ws_array_feasible_L, ws_feasible_L = workspace_feasible(forces=forces)
ws_array_desired_R, ws_desired_R = workspace_desired(joint="human/right_wrist")

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(ws_feasible_L[0, :], ws_feasible_L[1, :], ws_feasible_L[2, :], c='c')
ax.scatter( ws_desired_R[0, :],  ws_desired_R[1, :],  ws_desired_R[2, :], c='r')
plt.show()
############################################
############################################
# print(ws_desired, ws_feasible, ws_array_desired, ws_array_feasible)
# print("delta workspace volum", np.sum(ws_array_desired - ws_array_feasible))

# def cost_function(attachment_points):
#     _, _, ws_array_desired, ws_array_feasible = workspace(side='left', attachment_points=attachment_points)
#     loss = np.dot((ws_array_desired - ws_array_feasible), (ws_array_desired - ws_array_feasible))
#     return loss


# print(cost_function(1000,2000))

# def cost_function(ws_desired, ws_feasible):
#     loss = np.dot((ws_desired - ws_feasible), (ws_desired - ws_feasible))
#     return loss

# constraints = [{'type':'eq', 'fun': lambda x: np.sum(active_tendons.T * np.array([x]), axis=1) - force_vector},
#                 {'type':'ineq', 'fun': lambda x: active_max_forces - x},
#                 {'type':'ineq', 'fun': lambda x: x}]
# solution = minimize(equalize, initial_forces, constraints=constraints, options={'maxiter': 500})
