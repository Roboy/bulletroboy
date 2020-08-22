import numpy as np
from scipy.optimize import minimize

MIN_TENDON_FORCE = 0.01
MAX_COLLISION_FORCE = 100

def decompose_force_link_to_ef(link_id):
    """Gets link bounding box dimensions.
    
    Args:
        link_id (int): Index of the link to search.

    Returns:
        3darray[float]: x, y, and z dimensions of the bounding box.

    """
    end_effector = None
    if link_id in [7, 8]:
        end_effector = "left_wrist"
    elif link_id in [4, 5]:
        end_effector = "right_wrist"
    return end_effector

def decompose_force_ef_to_tendons(force_value, force_direction, muscle_units):
    """Decomposes force applied to end effector to the tendons connected to it.
    
    Args:
        force_value (float): Value of the applied force.
		force_direction array[3]: Force direction in world space coordinates.
        muscle_units (list[MuscleUnit]): List of the muscle units conected to the end effector.

    Returns:
		dict: Dictionary with the decomposed forces, the key is the tendon id.
        string: Decomposition error message, will be empty if the decomposition was performed succesfully.

    """
    force_value = np.clip(force_value, 0 , MAX_COLLISION_FORCE)

    max_forces = [muscle.max_force for muscle in muscle_units]
    motor_attachments = np.array([muscle.motor.via_point.world_point for muscle in muscle_units])
    tendon_attachments = np.array([muscle.end_effector.world_point for muscle in muscle_units])

    tendon_vectors = motor_attachments - tendon_attachments
    tendon_directions = tendon_vectors / np.linalg.norm(tendon_vectors, axis=1, keepdims=True)

    # projecting vector directions to force direction, only tendons with positive values, will be part of the decomposition
    projections = tendon_directions.dot(force_direction)
    active_mask = projections > 0
    active_tendons = tendon_directions[active_mask]
    active_max_forces = np.asarray(max_forces)[active_mask]

    forces = {}
    message = ""
    if len(active_tendons) > 0:

        initial_forces = np.random.rand(len(active_tendons)) * force_value
        constraints = [{'type':'eq', 'fun': lambda x: np.sum(active_tendons.T * x, axis=1) - force_value * force_direction},
                        {'type':'ineq', 'fun': lambda x: active_max_forces - x},
                        {'type':'ineq', 'fun': lambda x: x - MIN_TENDON_FORCE}]
        solution = minimize(lambda x: np.sqrt(np.sum(np.square(x))),
                            initial_forces,
                            constraints=constraints, options={'maxiter': 100}
                            )
        if solution.success:
            final_forces = np.zeros(len(tendon_vectors))
            final_forces[active_mask] = solution.x
            forces = {muscle.id: force for muscle, force in zip(muscle_units, final_forces)}
        else:
            message = solution.message
    
    return forces, message
