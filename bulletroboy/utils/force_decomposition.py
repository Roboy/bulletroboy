import numpy as np
from scipy.optimize import minimize
import rclpy

def decompose_force_ef_to_tendons(force_value, force_direction, end_effector, params):
    """Decomposes force applied to end effector to the tendons connected to it.
    
    Args:
        force_value (float): Value of the applied force.
		force_direction (3darray(float)): Force direction in world space coordinates.
        end_effector (EndEffector): End effector for which the force must be decomposed.
        params (dict): Decomposition params.

    Returns:
		dict{tendon_id: force_value}: Dictionary with the decomposed forces.
        string: Decomposition error message, will be empty if the decomposition was performed succesfully.

    """
    rclpy.logging._root_logger.info(f"force_value before : {force_value}")

    force_value = np.clip(force_value, 0 , params['max_collision_force'])

    rclpy.logging._root_logger.info(f"force_value after : {force_value}")

    max_forces = [muscle.max_force for muscle in end_effector.muscle_units]
    motor_attachments = np.array([muscle.motor.via_point.world_point for muscle in end_effector.muscle_units])
    tendon_attachments = np.array([end_effector.position for _ in end_effector.muscle_units])

    forces = {}
    message = ""

    if np.any([attach is None for attach in tendon_attachments]):
        message = "End effector positions are not initialized!"
    else:
        tendon_vectors = motor_attachments - tendon_attachments
        tendon_directions = tendon_vectors / np.linalg.norm(tendon_vectors, axis=1, keepdims=True)

        # projecting vector directions to force direction, only tendons with positive values, will be part of the decomposition
        projections = tendon_directions.dot(force_direction)
        active_mask = projections > 0
        active_tendons = tendon_directions[active_mask]
        active_max_forces = np.asarray(max_forces)[active_mask]

        if len(active_tendons) > 0:
            rclpy.logging._root_logger.info(f"active_tendons: {active_tendons}, len = {len(active_tendons)}")
            rclpy.logging._root_logger.info(f"np.random.rand(len(active_tendons): {np.random.rand(len(active_tendons))}")

            initial_forces = np.random.rand(len(active_tendons)) * force_value
            rclpy.logging._root_logger.info("Here")

            constraints = [{'type':'eq', 'fun': lambda x: np.sum(active_tendons.T * x, axis=1) - force_value},
                            {'type':'ineq', 'fun': lambda x: active_max_forces - x},
                            {'type':'ineq', 'fun': lambda x: x - params['min_tendon_force']}]
            solution = minimize(lambda x: np.sqrt(np.sum(np.square(x))),
                                initial_forces,
                                constraints=constraints, options={'maxiter': 100}
                                )
            if solution.success:
                final_forces = np.zeros(len(tendon_vectors))
                final_forces[active_mask] = solution.x
                forces = {muscle.id: force for muscle, force in zip(end_effector.muscle_units, final_forces)}
            else:
                message = solution.message
    
    return forces, message
