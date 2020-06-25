import numpy as np
from os import path
from scipy.optimize import minimize
from utils import CageConfiguration
from time import time
import matplotlib.pyplot as plt
# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

def force_decomposition(force_vector, force_point, tendons, attachment_points):
    """ This function decomposes a force vector to different tendon vectors.

    Args:
        force_vector (numpy.array):         The force that shall be simulated.
        force_point (numpy.array):          The coordinates where this force vector is attached to.
        tendons (dict):                     Containing information about the tendons
        attachment_points (numpy.array):    The coordinates of the attachment points of the tendons on the cage.

    Example:
        The tendons dictionary contains information about the tendon Id and the max force for each tendon.

    Returns:
        forces (numpy.array):               The force each tendon has to pull in order to simulate the desired force.
                                            returns None if force decomposition fails.
    """
    # start = time()
    force_direction = force_vector / np.linalg.norm(force_vector)
    max_forces = [int(tendons[id]['max_force']) for id in tendons]
    tendon_vectors = (attachment_points - np.array([force_point]).T).T
    tendon_directions = tendon_vectors / np.linalg.norm(tendon_vectors, axis=1, keepdims=True)

    # Projecting the tendon vectors on to the force direction vector.
    # Only the tendons for which a positive value is returned will participate in the division of strength
    projections = tendon_directions.dot(force_direction)
    active_tendons = tendon_directions[projections > 0]
    active_max_forces = np.asarray(max_forces)[projections > 0]

    forces = None
    if len(active_tendons) > 0:
        # the idea is to distribute the forces onto every tendon equally while the following conditions hold."
        #   1. the sum over all tendon forces and the simulated force must become zero"
        #   2. the force in each tendon may not exceed a maximal force"
        #   3. the force must be non-negative since a tendon is not able to push but only pull"
        initial_forces = np.random.rand(len(active_tendons)) * np.mean(force_vector)
        constraints = [{'type': 'eq', 'fun': lambda x: np.sum(active_tendons.T * np.array([x]), axis=1) - force_vector},
                       {'type': 'ineq', 'fun': lambda x: active_max_forces - x},
                       {'type': 'ineq', 'fun': lambda x: x}]
        solution = minimize(lambda x: np.sqrt(np.sum(np.square(x))),
                            initial_forces,
                            constraints=constraints,
                            options={'maxiter': 200})

        if solution.success:
            forces = {id: force for id, force in zip(tendons, solution.x) if force > MIN_FORCE}
    # else:
    #     # print(f"WARNING: Cannot decompose the forces: [ {solution.message} ]")

    #print(f"Time: {time() - start} s")
    return forces

def get_attachment_points(radius, height, angles, joint):
    """ This function calculates the attachment points for the tendons.

    Args:
        radius (numpy.array):               The radius of the upper part of the cage and the lower part of the cage
        height (numpy.array):               The height of the cage
        angles (numpy.array):               Angles of the tendons with respect to the x-axis
        joint (str):                        Information about the left or right wrist

    Returns:
        attachment_points (numpy.array):    The force each tendon has to pull in order to simulate the desired force.
                                            returns None if force decomposition fails.
        """

    # The degree of freedom of some points is fixed, assuming that this can prevent the tendons from getting tangled.
    attachment_points = np.zeros((3, 8))
    attachment_points[0, 0] = radius[0]                     # attached in front of the person
    attachment_points[0, 1] = radius[0] * np.cos(angles[0])
    attachment_points[0, 2] = radius[0] * np.cos(angles[1])
    attachment_points[0, 3] = 0                             # attached on the person
    attachment_points[0, 4] = radius[1]
    attachment_points[0, 5] = radius[1] * np.cos(angles[2])
    attachment_points[0, 6] = radius[1] * np.cos(angles[3])
    attachment_points[0, 7] = 0                             # attached on the person
    attachment_points[2, 0:3] = height                      # upper tendons
    attachment_points[2, 3] = 1.5                           # shoulder tendon
    attachment_points[2, 4:7] = 0                           # lower tendons
    attachment_points[2, 7] = 0.75                          # hip tendon

    if joint == "human/left_wrist":
        attachment_points[1, 0] = 0
        attachment_points[1, 1] = radius[0] * np.sin(angles[0])
        attachment_points[1, 2] = radius[0] * np.sin(angles[1])
        attachment_points[1, 3] = 0
        attachment_points[1, 4] = 0
        attachment_points[1, 5] = radius[1] * np.sin(angles[2])
        attachment_points[1, 6] = radius[1] * np.sin(angles[3])
        attachment_points[1, 7] = 0
    elif joint == "human/right_wrist":
        attachment_points[1, 0] = 0
        attachment_points[1, 1] = -radius[0] * np.sin(angles[2])
        attachment_points[1, 2] = -radius[0] * np.sin(angles[3])
        attachment_points[1, 3] = 0
        attachment_points[1, 4] = 0
        attachment_points[1, 5] = -radius[1] * np.sin(angles[2])
        attachment_points[1, 6] = -radius[1] * np.sin(angles[3])
        attachment_points[1, 7] = 0
    return attachment_points

def workspace_desired(step_size=0.2, joint="human/left_wrist"):
    """ This function calculates the ergonomic workspace of the arm of a standing person

    Args:
        step_size (float):                  The size of the iteration steps
        joint (str):                        Information about the left or right wrist

    Returns:
        ws_array_desired (numpy.array):     A binary array where each one indicates that the vector
                                            corresponding to this index is inside the workspace.
        ws_desired (numpy.array):           A 3xm matrix where each row corresponds to the x, y or z coordinate
                                            respectively of all vectors inside the workspace.
   """
    # start = time()
    x, y, z = np.meshgrid(np.arange(-step_size, 1 + step_size, step_size),
                          np.arange(-1, 1 + step_size, step_size),
                          np.arange( 0, 2 + step_size, step_size))

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
    # print(f"Time: {time() - start} s")
    return ws_array_desired, ws_desired

def workspace_feasible(step_size=0.2, joint="human/left_wrist", radius=np.array([1., 1.]), height=2.,
                       angles=np.array([np.pi/3, 2*np.pi/3]), forces=np.array([[100., 0., 0.]])):
    """ This function calculates the feasible workspace of the arm of a standing person

    Args:
        step_size (float):                  The size of the iteration steps
        joint (str):                        Information about the left or right wrist
        radius (numpy.array):               The radius of the upper part of the cage and the lower part of the cage
        height (numpy.array):               The height of the cage
        angles (numpy.array):               Angles of the tendons with respect to the x-axis
        forces (numpy.array):               The forces applied to the wrist

    Returns:
        ws_array_feasible (numpy.array):    A binary array where each one indicates that the vector
                                            corresponding to this index is inside the workspace.
        ws_feasible (numpy.array):          A 3xm matrix where each row corresponds to the x, y or z coordinate
                                            respectively of all vectors inside the workspace.
    """

    # start = time()
    attachment_point = get_attachment_points(radius, height, angles, joint)
    x, y, z = np.meshgrid(np.arange(-step_size, 1 + step_size, step_size),
                          np.arange(-1, 1 + step_size, step_size),
                          np.arange( 0, 2 + step_size, step_size))

    xx = x.reshape(-1)
    yy = y.reshape(-1)
    zz = z.reshape(-1)
    ws = np.array([xx, yy, zz])
    ws_array_feasible = np.ones_like(xx)

    # If the workspace for multiple force vectors is calculated, the effort to calculate the ith workspace for the ith
    # vector can be reduced by only using coordinates for the i-1 th vector where a force decomposition was able.
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
    # print(f"Time: {time() - start} s")
    return ws_array_feasible, ws_feasible

def cost_function(param):
    """ This function calculates the L-2 distance loss between a desired and a feasible workspace

    Args:
        param (numpy.array):                Vector containing position parameters for the tendon attachment points

    Example:
        the first two entries in the param vector represent the radius
        the third value represents the height of the cage
        the rest of the parameters correspond to the angles of the tendon attachment with respect to the x-axis

    Returns:
        loss (float):                       scalar value describing the L-2 distance between a desired and
                                            a feasible workspace
    """

    # The loss is defined as the square distance between the desired coordinates and the accessible coordinates.
    # all matching coordinates are weighted positively.

    # start = time()
    radius = param[0:2]
    height = param[2]
    angles = param[3::]
    forces = np.array([[100, 0, 0],
                       [0, 100, 0],
                       [0, 0, 100],
                       [-100, 0, 0],
                       [0, -100, 0],
                       [0, 0, -100]])

    ws_array_feasible, _ = workspace_feasible(radius=radius, height=height, angles=angles, forces=forces)
    ws_array_feasible = (ws_array_feasible + (ws_array_feasible * WS_ARRAY_DESIRED * 9)) / 10
    loss = np.dot((WS_ARRAY_DESIRED - ws_array_feasible), (WS_ARRAY_DESIRED - ws_array_feasible))
    # print(f"Time: {time() - start} s")
    print(loss)
    print(param)
    return loss

#__main__

MIN_FORCE = 0.01

#   GETTING TENDONS FROM CAGE CONFIGURATION FILE
cage_conf_file_path = path.dirname(path.realpath(__file__)) + "/cageConfiguration.xml"
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

# print(f"\n{len(tendon_groups)} joints with tendons attached found in ({cage_conf_file_path}):")
# for joint in tendon_groups:
#    print(f"\t- {joint}\t: {len(tendon_groups[joint])} tendons {[id for id in tendon_groups[joint]]}")
# print("\n")

# Predefining the desired workspace saves allot of computing effort because is doesn't have
# to be calculated over and over again.
WS_ARRAY_DESIRED, _ = workspace_desired()

# we might need a constraint on the angles so that they do not enter a unreachable space.
# The theory says that the Nelder-Mead algorithm moves towards the cheapest solution of a symplex,
# and since the solution before reaching a colinearity is more likely to be cheaper than
# reaching colinearity, the occurrence of this solution is very low.
# for this theory to hold it is very important to initialize al parameters carefully.
# constraint = [{'type': 'ineq', 'fun': lambda x: x}]
initial_forces = np.array([1, 1, 2, np.pi/3, 2*np.pi/3, np.pi/3, 2*np.pi/3])

# Changing the radius and the height might change the most.
# During the calculations I noticed that the algorithm would not change the angles,
# so initializing the symplex with differing angles could cause the algorithm to start changing the angles.
one = np.pi/180
initial_simplex = np.array([[1.05, 1.05, 2.05, np.pi/3 + one, 2*np.pi/3 + one, np.pi/3 - one, 2*np.pi/3 - one],
                            [1.05, 1.05, 2.05, np.pi/3 - one, 2*np.pi/3 - one, np.pi/3 + one, 2*np.pi/3 + one],
                            [0.95, 0.95, 1.95, np.pi/3 + one, 2*np.pi/3 + one, np.pi/3 - one, 2*np.pi/3 - one],
                            [0.05, 0.95, 1.95, np.pi/3 - one, 2*np.pi/3 - one, np.pi/3 + one, 2*np.pi/3 + one],
                            [0.95, 1.05, 2.05, np.pi/3, 2*np.pi/3, np.pi/3, 2*np.pi/3],
                            [0.95, 0.95, 2.05, np.pi/3, 2*np.pi/3, np.pi/3, 2*np.pi/3],
                            [1.05, 1.05, 1.95, np.pi/3, 2*np.pi/3, np.pi/3, 2*np.pi/3],
                            [1.05, 0.95, 2.05, np.pi/3, 2*np.pi/3, np.pi/3, 2*np.pi/3]])

# I would use the Nelder-Mead algorithm since it is a very robust algorithm and it
# dose not relies on the derivative of the function. Since this algorithm is known to
# converge not so fast, I chose to set the number of maximal iterations to 100
# solver = 'BFGS'
solver = 'Nelder-Mead'
# solver = 'CG'
# solver = 'Newton-CG'
# solver = 'L-BFGS-B'
# solver = 'TNC'
# solver = 'COBYLA'
# solver = 'SLSQP'
# solver = 'trust-constr'
# xatol (the absolut error in xopt between iterations that is acceptable for convergence is set
# to 0.001 which is equivalent to 1mm
solution = minimize(cost_function, initial_forces, method='Nelder-Mead',
                    options={'maxiter': 100, 'initial_simplex': initial_simplex, 'xatol': 0.001})

# Get the position parameters that minimize the cost function.
print(solution)
radius = solution.x[0:2]
height = solution.x[2]
angles = solution.x[3::]

# Display workspace under certain conditions
forces = np.array([[100, 0, 0],
                   [0, 100, 0],
                   [0, 0, 100],
                   [-100, 0, 0],
                   [0, -100, 0],
                   [0, 0, -100]])

# Get the feasible part of the left workspace and the desired version of the right workspace
ws_array_feasible_L, ws_feasible_L = workspace_feasible(stepsize=0.1, forces=forces, radius=radius, height=height,
                                                        angles=angles)
ws_array_desired_R, ws_desired_R = workspace_desired(stepsize=0.1, joint="human/right_wrist")


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
data_L = np.array(ax.scatter(ws_feasible_L[0, :], ws_feasible_L[1, :], ws_feasible_L[2, :],
                             c='c', label='left ws feasible')._offsets3d)
data_R = np.array(ax.scatter(ws_desired_R[0, :], ws_desired_R[1, :], ws_desired_R[2, :],
                             c='r', label='right ws desired')._offsets3d)
ax.legend()
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
plt.title(f"Workspace {solver}")
n = 1
data = {n: [data_L, data_R]}

# Save test data to .txt file
text_file = open(f"../../Desktop/{solver}_02.txt", "wt")
n = text_file.write(f"solver:\n{solver}\n"
                    f"loss:\n{solution.fun}\n"
                    f"Number of iterations:\n{solution.nit}\n"
                    f"radius:\n{radius}\n"
                    f"height:\n{height}\n"
                    f"angles:\n{angles}\n"
                    f"Image Data:"
                    f"\ndata_L\n{data[n][0]}\ndata_R\n{data[n][0]}\n"
                    f"Initial simplex:\n{initial_simplex}")
text_file.close()
plt.show()
