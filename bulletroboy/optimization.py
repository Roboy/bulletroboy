import numpy as np
import os
import yaml
from scipy.optimize import minimize
from time import time


def force_decomposition(force_vector, force_point, attachment_points):
    """ This function decomposes a force vector to different tendon vectors.

    Args:
        force_vector (numpy.array):         The force that shall be simulated.
        force_point (numpy.array):          The coordinates where this force vector is attached to.
        attachment_points (numpy.array):    The coordinates of the attachment points of the tendons on the cage.

    Example:
        The tendons dictionary contains information about the tendon Id and the max force for each tendon.

    Returns:
        success :                           Returns True if it is possible to decompose the forces.
                                            Returns False if it is not possible to decompose the forces
                                            under the given conditions.
    """
    # start = time()
    min_force = OPTIMIZATION_CONFIG.get('MinForce')
    max_force = OPTIMIZATION_CONFIG.get('MaxForce')
    force_direction = force_vector / np.linalg.norm(force_vector)
    tendon_vectors = (attachment_points - np.array([force_point]).T).T
    tendon_directions = tendon_vectors / np.linalg.norm(tendon_vectors, axis=1, keepdims=True)

    # Projecting the tendon vectors on to the force direction vector.
    # Only the tendons for which a positive value is returned will participate in the division of strength
    projections = tendon_directions.dot(force_direction)
    active_tendons = tendon_directions[projections > 0]
    success = False
    if len(active_tendons) > 0:
        # the idea is to distribute the forces onto every tendon equally while the following conditions hold."
        #   1. the sum over all tendon forces and the simulated force must become zero"
        #   2. the force in each tendon may not exceed a maximal force"
        #   3. the force must be non-negative since a tendon is not able to push but only pull"
        #   4. the force in each tendon shall be bigger then a minimal force
        initial_forces = np.random.rand(len(active_tendons)) * np.mean(force_vector)
        constraints = [{'type': 'eq', 'fun': lambda x: np.sum(active_tendons.T * np.array([x]), axis=1) - force_vector},
                       {'type': 'ineq', 'fun': lambda x: max_force - x},
                       {'type': 'ineq', 'fun': lambda x: x},
                       {'type': 'ineq', 'fun': lambda x: x - min_force}]
        solution = minimize(lambda x: np.sqrt(np.sum(np.square(x))), initial_forces,
                            constraints=constraints, options={'maxiter': 100})

        success = solution.success

    # print(f"force decomposition Time:\t{time() - start} s")
    # force decomposition Time: 1e-05s - 1s
    return success


def get_attachment_points(cage, joint_right=True):

    """ This function calculates the attachment points for the tendons.

    Args:
        cage (tupel):                       Tupel containing the following parameters (floor_radius, ceiling_radius,
                                            height, floor_alpha, floor_beta, ceiling_alpha, ceiling_beta)
            floor_radius (float):           The radius of the lower part of the cage
            ceiling_radius (float):         The radius of the upper part of the cage
            height (float):                 The height of the cage.
            floor_alpha (float):            Angles of the tendons with respect to the x-axis.
            floor_beta (float):             Angles of the tendons with respect to the x-axis.
            ceiling_alpha (float):          Angles of the tendons with respect to the x-axis.
            ceiling_beta (float):           Angles of the tendons with respect to the x-axis.

        joint_right (bool):                 Information about the left or right wrist

    Returns:
        attachment_points (numpy.array):    The force each tendon has to pull in order to simulate the desired force.
                                            returns None if force decomposition fails.
        """

    # start = time()

    floor_radius = cage[0]
    ceiling_radius = cage[1]
    height = cage[2]
    floor_alpha = cage[3]
    floor_beta = cage[4]
    ceiling_alpha = cage[5]
    ceiling_beta = cage[6]

    shoulder_left = np.array(OPTIMIZATION_CONFIG.get('ShoulderLeft'))
    shoulder_right = np.array(OPTIMIZATION_CONFIG.get('ShoulderRight'))
    hip_left = np.array(OPTIMIZATION_CONFIG.get('HipLeft'))
    hip_right = np.array(OPTIMIZATION_CONFIG.get('HipRight'))

    # The degree of freedom of some points is fixed, assuming that this can prevent the tendons from getting tangled.
    attachment_points = np.zeros((3, 8))
    attachment_points[0, 0] = floor_radius          # attached in front of the person
    attachment_points[0, 1] = floor_radius * np.cos(floor_alpha)
    attachment_points[0, 2] = floor_radius * np.cos(floor_beta)
    attachment_points[0, 4] = ceiling_radius        # attached in front of the person
    attachment_points[0, 5] = ceiling_radius * np.cos(ceiling_alpha)
    attachment_points[0, 6] = ceiling_radius * np.cos(ceiling_beta)
    attachment_points[2, 0:3] = height              # ceiling tendons
    attachment_points[2, 4:7] = 0                   # floor tendons

    if joint_right:
        attachment_points[:, 3] = shoulder_right    # attached on the person
        attachment_points[:, 7] = hip_right         # attached on the person
        attachment_points[1, 1] = floor_radius * np.sin(floor_alpha)
        attachment_points[1, 2] = floor_radius * np.sin(floor_beta)
        attachment_points[1, 5] = ceiling_radius * np.sin(ceiling_alpha)
        attachment_points[1, 6] = ceiling_radius * np.sin(ceiling_beta)
    elif not joint_right:
        attachment_points[:, 3] = shoulder_left     # attached on the person
        attachment_points[:, 7] = hip_left          # attached on the person
        attachment_points[1, 1] = -floor_radius * np.sin(floor_alpha)
        attachment_points[1, 2] = -floor_radius * np.sin(floor_beta)
        attachment_points[1, 5] = -ceiling_radius * np.sin(ceiling_alpha)
        attachment_points[1, 6] = -ceiling_radius * np.sin(ceiling_beta)

    # print(f"get attachment points Time:\t{time() - start} s")
    # get attachment points Time:   3e-05 s

    return attachment_points


def workspace_desired(joint_right=True):

    """ This function calculates the ergonomic workspace of the arm of a standing person

    Args:
        joint_right (bool):                 Information about the left or right wrist

    Returns:
        ws_desired_array (numpy.array):     A binary array where each one indicates that the vector
                                            corresponding to this index is inside the workspace.
        ws_desired (numpy.array):           A 3xm matrix where each row corresponds to the x, y or z coordinate
                                            respectively of all vectors inside the workspace.
   """
    # start = time()

    total_height = OPTIMIZATION_CONFIG.get('TotalHeight')  # in m
    floor_height = OPTIMIZATION_CONFIG.get('FloorHeight')  # in m
    width = OPTIMIZATION_CONFIG.get('Width')  # in m
    step_size = OPTIMIZATION_CONFIG.get('StepSize')

    # build mesh grid of the sample space
    x, y, z = np.meshgrid(np.arange(-step_size, width + step_size, step_size),
                          np.arange(-width, width + step_size, step_size),
                          np.arange(floor_height, total_height + step_size, step_size))

    # reshape the mesh grid and rearrange it to a matrix with the xyz coordinates of every sample
    xx = x.reshape(-1)
    yy = y.reshape(-1)
    zz = z.reshape(-1)
    ws = np.array([xx, yy, zz])

    radius_small_grip_space = OPTIMIZATION_CONFIG.get('RadiusSmallGripSpace')
    radius_large_grip_space = OPTIMIZATION_CONFIG.get('RadiusLargeGripSpace')

    shoulder_left = np.array(OPTIMIZATION_CONFIG.get('ShoulderLeft'))
    shoulder_right = np.array(OPTIMIZATION_CONFIG.get('ShoulderRight'))

    ws_desired_array = np.array([])

    # Since the workspace is oriented around the shoulder of the Pilot, the shoulder determines the offset.
    if joint_right:
        offset = shoulder_right
    else:
        offset = shoulder_left

    for point in ws.T:
        x = point[0]
        y = point[1]
        if radius_small_grip_space <= np.linalg.norm(point - offset) <= radius_large_grip_space and offset[0] <= x:
            if joint_right and offset[1] <= y:
                ws_desired_array = np.append(ws_desired_array, 1)
            elif not joint_right and y <= offset[1]:
                ws_desired_array = np.append(ws_desired_array, 1)
            else:
                ws_desired_array = np.append(ws_desired_array, 0)
        else:
            ws_desired_array = np.append(ws_desired_array, 0)
    ii = np.where(ws_desired_array == 1)[0]
    ws_desired = ws[:, ii]

    # print(f"workspace desired Time:\t{time() - start} s")
    # workspace desired Time: 0.02 s

    return ws_desired_array, ws_desired


def loss_function(x, *params):
    """ This function calculates the L-2 distance loss between a desired and a feasible workspace

    Args:
        x (numpy.array):                    Vector containing position parameters for the tendon attachment points
        params (tupel):                     Tupel containing the following parameters (ws, ws_desired, forces)
            ws_desired_array (numpy.array): Binary vector, with ones where the coordinates for the workspace are within
                                            the desired workspace
            forces (numpy.array):           Matrix of force vectors, the optimizer is optimizing with

    Example:
        x[0] = FloorRadius
        x[1] = CeilingRadius
        x[2] = Height
        x[3] = FloorAlpha
        x[4] = FloorBeta
        x[5] = CeilingAlpha
        x[6] = CeilingBeta

        ws_desired_array = 1 for x,y and z within the desired workspace
        ws_desired_array = 0 for x,y or z outside the desired workspace

    Returns:
        loss (float):                       scalar value describing the L-2 distance between a desired and
                                            a feasible workspace
    """

    # The loss is defined as the square distance between the desired coordinates and the accessible coordinates.
    # all matching coordinates are weighted positively.

    # start = time()

    floor_radius = x[0]
    ceiling_radius = x[1]
    height = x[2]
    floor_alpha = x[3]
    floor_beta = x[4]
    ceiling_alpha = x[5]
    ceiling_beta = x[6]

    ws_desired_array = params[0]
    forces = params[1]
    direction_preference = params[2]

    total_height = OPTIMIZATION_CONFIG.get('TotalHeight')
    floor_height = OPTIMIZATION_CONFIG.get('FloorHeight')
    width = OPTIMIZATION_CONFIG.get('Width')
    step_size = OPTIMIZATION_CONFIG.get('StepSize')

    # build mesh for the sample space
    x, y, z = np.meshgrid(np.arange(-step_size, width + step_size, step_size),
                          np.arange(-width, width + step_size, step_size),
                          np.arange(floor_height, total_height + step_size, step_size))

    xx = x.reshape(-1)
    yy = y.reshape(-1)
    zz = z.reshape(-1)
    ws = np.array([xx, yy, zz]) # change mesh into coordinate vectors.

    cage = (floor_radius, ceiling_radius, height, floor_alpha, floor_beta, ceiling_alpha, ceiling_beta)
    attachment_point = get_attachment_points(cage)
    loss = 0
    ws_feasible = np.ones(ws.shape[1])
    for force in forces:

        force_norm = force / np.linalg.norm(force)                  # normalized force vector
        force_direction = np.dot(force_norm, direction_preference)  # projection on the direction preference vector
        force_direction_positive = np.sqrt(force_direction ** 2)    # eliminate the sign

        for i, point in enumerate(ws.T):
            if ws_feasible[i] == 1 and force_decomposition(force, point, attachment_point):
                ws_feasible[i] = 1
            else:
                ws_feasible[i] = 0
        ws_feasible_array = (ws_feasible + (ws_feasible * ws_desired_array * 14)) / 15
        loss = loss + np.dot((ws_desired_array - ws_feasible_array), (ws_desired_array - ws_feasible_array)) \
                    + force_direction_positive

    # print(f"loss function time: {time() - start} s")
    # loss function time: ~67s
    print(f"Loss:{loss}")
    # Loss: ~ 20
    return loss


__name__ = "__main__"

if __name__ == "__main__":
    start = time()

    # get initial values from optimization config file
    OptimizationConfigFilePath = os.path.dirname(os.path.realpath(__file__)) + "/../config/Optimization_Config.yaml"

    with open(OptimizationConfigFilePath) as File:
        OPTIMIZATION_CONFIG = yaml.load(File, Loader=yaml.FullLoader)
    StepSize = OPTIMIZATION_CONFIG.get('StepSize')              # in m

    FloorRadius = OPTIMIZATION_CONFIG.get('FloorRadius')        # in m
    CeilingRadius = OPTIMIZATION_CONFIG.get('CeilingRadius')    # in m
    Height = OPTIMIZATION_CONFIG.get('Height')                  # in m
    FloorAlpha = OPTIMIZATION_CONFIG.get('FloorAlpha')          # in rad
    FloorBeta = OPTIMIZATION_CONFIG.get('FloorBeta')            # in rad
    CeilingAlpha = OPTIMIZATION_CONFIG.get('CeilingAlpha')      # in rad
    CeilingBeta = OPTIMIZATION_CONFIG.get('CeilingBeta')        # in rad
    MaxForce = OPTIMIZATION_CONFIG.get('MaxForce')              # in N
    MinForce = OPTIMIZATION_CONFIG.get('MinForce')              # in N
    TotalHeight = OPTIMIZATION_CONFIG.get('TotalHeight')        # in m
    FloorHeight = OPTIMIZATION_CONFIG.get('FloorHeight')        # in m
    Width = OPTIMIZATION_CONFIG.get('Width')                    # in m

    # todo: set Force vectors in N. The optimizer uses these vectors to determine the work space in which these vectors
    #       can be decomposed into the tendons
    Forces = np.array([[100, 0, 0],     # in N
                       [0, 100, 0],
                       [0, 0, 100],
                       [-100, 0, 0],
                       [0, -100, 0],
                       [0, 0, -100]])

    # todo: set the direction preference vector to support a certain direction in the optimization
    # note: if the direction vector is set to all zero the optimizer will not value any force direction stronger.
    DirectionPreference = np.array([0, 0, 1])   # give mor value to a workspace that is able to simulate vertical forces

    WsDesiredArray, _ = workspace_desired()

    # Initial values
    X0 = np.array([FloorRadius, CeilingRadius, Height, FloorAlpha, FloorBeta, CeilingAlpha, CeilingBeta])
    Params = (WsDesiredArray, Forces, DirectionPreference)

    # initializing the simplex
    Low = -10
    High = 10
    Size = [8, 7]
    Seed = np.random.randint(0, 100)
    np.random.seed(Seed)
    Random = np.random.randint(Low, High, Size)
    InitialSimplex = (np.ones(Size) * X0 + Random * X0 / 100).round(4)

    Solver = 'Nelder-Mead'
    Solution = minimize(loss_function, X0, args=Params, method=Solver,
                        options={'maxiter': 100, 'initial_simplex': InitialSimplex, 'xatol': 0.001, 'fatol': 0.001})
    Radius = Solution.x[0:2]
    Height = Solution.x[2]
    Angles = Solution.x[3::]

    print(Solution)

    # save the solution to a text file
    # k = 0
    # saved = False
    # while not saved:
    #     if not any(f"{Solver}_test_{k}.txt" in s for s in os.listdir(os.getcwd())):
    #         with open(f"{Solver}_test_{k}.txt", "wt") as text_file:
    #             text_file.write(f"loss:\n{Solution.fun}\n"
    #                             f"Number of iterations:\n{Solution.nit}\n"
    #                             f"radius:\n{Radius}\n"
    #                             f"height:\n{Height}\n"
    #                             f"angles:\n{Angles}\n"
    #                             f"step size:{StepSize}\n"
    #                             f"seed:{Seed}\n"
    #                             f"Initial simplex:\n{InitialSimplex}")
    #         text_file.close()
    #         saved = True
    #     else:
    #         k += 1
    #         saved = False

    AttachmentPointsLeft = get_attachment_points(Solution.x, joint_right=False).tolist()
    AttachmentPointsRight = get_attachment_points(Solution.x, joint_right=True).tolist()
    k = 0
    saved = False
    while not saved:
        if not any(f"attachment_points_{Solver}_test_{k}.yaml" in s for s in os.listdir(os.getcwd())):
            OptimizationSolution = {'AttachmentPointsLeft': AttachmentPointsLeft, 'Seed': Seed, 'StepSize': StepSize,
                                    'AttachmentPointsRight': AttachmentPointsRight, 'Width': Width,
                                    'FloorHeight': FloorHeight, 'MaxForce': MaxForce, 'MinForce': MinForce,
                                    'TotalHeight': TotalHeight}
            with open(f"attachment_points_{Solver}_test_{k}.yaml", 'w') as File:
                documents = yaml.dump(OptimizationSolution, File)
            saved = True
        else:
            k += 1
            saved = False

    print(f"main time: {time() - start} s")
    # main time: 10 000s = 2,77h
