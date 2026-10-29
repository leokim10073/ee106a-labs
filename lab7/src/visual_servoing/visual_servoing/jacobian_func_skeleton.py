"""Functions to help with Jacobian calculations."""

import numpy as np
from scipy.linalg import logm

from .kin_func_skeleton import (
    R3_to_so3,
    forward_kinematics,
    inverse_SE3,
    se3_to_twist,
    twist_to_SE3,
)


def adjoint(g):
    """Calculate the adjoint matrix of a rigid transformation.

    Args:
        g: NumPy array with shape ``(4, 4)`` representing a
            transformation in SE(3).

    Returns:
        NumPy array with shape ``(6, 6)`` representing ``Ad_g``.
    """

    R = g[:3, :3]
    p = g[:3, 3]
    ad = np.zeros((6, 6))
    ad[:3, :3] = R
    ad[:3, 3:] = R3_to_so3(p) @ R
    ad[3:, 3:] = R
    return ad


def adjoint_inv(g):
    """Calculate the inverse adjoint matrix of a rigid transformation.

    Args:
        g: NumPy array with shape ``(4, 4)`` representing a
            transformation in SE(3).

    Returns:
        NumPy array with shape ``(6, 6)`` representing ``Ad_g^{-1}``.
    """

    return adjoint(inverse_SE3(g))


def spatial_jacobian(twists, theta):
    """Calculate the spatial manipulator Jacobian.

    Args:
        twists: NumPy array with shape ``(6, N)`` whose columns are the
            spatial twists of the joints at the home configuration.
        theta: NumPy array with shape ``(N,)`` containing the joint values.

    Returns:
        NumPy array with shape ``(6, N)`` containing the spatial Jacobian at
        ``theta``.
    """

    twists = np.asarray(twists, dtype=float)
    theta = np.asarray(theta, dtype=float).flatten()
    n = twists.shape[1]
    J = np.zeros((6, n))
    g = np.eye(4)
    for i in range(n):
        # Column i is xi_i transformed by the product of the preceding exponentials
        J[:, i] = adjoint(g) @ twists[:, i]
        g = g @ twist_to_SE3(twists[:, i], theta[i])
    return J


def body_jacobian(twists, theta, g_st0):
    """Calculate the body manipulator Jacobian.

    Args:
        twists: NumPy array with shape ``(6, N)`` whose columns are the
            spatial twists of the joints at the home configuration.
        theta: NumPy array with shape ``(N,)`` containing the joint values.
        g_st0: NumPy array with shape ``(4, 4)`` containing the end-effector
            pose at the home configuration.

    Returns:
        NumPy array with shape ``(6, N)`` containing the body Jacobian at
        ``theta``.
    """

    g_st = forward_kinematics(twists, theta) @ g_st0
    return adjoint_inv(g_st) @ spatial_jacobian(twists, theta)


def spatial_velocity(twists, theta, theta_dot):
    """Calculate the spatial end-effector velocity.

    Args:
        twists: NumPy array with shape ``(6, N)`` whose columns are the
            spatial twists of the joints at the home configuration.
        theta: NumPy array with shape ``(N,)`` containing the joint values.
        theta_dot: NumPy array with shape ``(N,)`` containing the joint
            velocities.

    Returns:
        NumPy array with shape ``(6,)`` containing the spatial velocity.
    """

    return spatial_jacobian(twists, theta) @ np.asarray(theta_dot, dtype=float).flatten()


def body_velocity(twists, theta, theta_dot, g_st0):
    """Calculate the body end-effector velocity.

    Args:
        twists: NumPy array with shape ``(6, N)`` whose columns are the
            spatial twists of the joints at the home configuration.
        theta: NumPy array with shape ``(N,)`` containing the joint values.
        theta_dot: NumPy array with shape ``(N,)`` containing the joint
            velocities.
        g_st0: NumPy array with shape ``(4, 4)`` containing the end-effector
            pose at the home configuration.

    Returns:
        NumPy array with shape ``(6,)`` containing the body velocity.
    """

    return body_jacobian(twists, theta, g_st0) @ np.asarray(theta_dot, dtype=float).flatten()


def spatial_configuration_error(pose, desired_pose):
    """Calculate the spatial-frame transformation error.

    Args:
        pose: NumPy array with shape ``(4, 4)`` for the current SE(3) pose.
        desired_pose: NumPy array with shape ``(4, 4)`` for the desired
            SE(3) pose, expressed in the same reference frame.

    Returns:
        NumPy array with shape ``(4, 4)`` containing our error.
    """

    return np.asarray(desired_pose, dtype=float) @ inverse_SE3(np.asarray(pose, dtype=float))


def spatial_error_twist(error):
    """Convert a transformation error into an exponential-coordinate vector.

    Args:
        error: NumPy array with shape ``(4, 4)`` in SE(3), returned by
            ``spatial_configuration_error``. Assume its rotation angle is less than pi.

    Returns:
        NumPy array with shape ``(6,)`` containing ``log(error)^vee``.

    Use ``scipy.linalg.logm`` for the matrix logarithm.
    """

    xi_hat = np.real(logm(np.asarray(error, dtype=float)))
    return se3_to_twist(xi_hat)


def jacobian_inverse_kinematics(
    twists, theta, g_st0, desired_pose, step_size=0.2,
    max_iterations=200, tolerance=1e-6,
):
    """Solve inverse kinematics using spatial errors and forward-Euler updates.

    Args:
        twists: NumPy array with shape ``(6, N)`` containing the joint twists
            at the home configuration.
        theta: NumPy array with shape ``(N,)`` containing the initial joint values.
        g_st0: NumPy array with shape ``(4, 4)`` containing the home tool pose.
        desired_pose: NumPy array with shape ``(4, 4)`` containing the target
            tool pose in the same reference frame.
        step_size: Nonnegative scalar multiplier for each joint update.
        max_iterations: Nonnegative integer limiting the number of updates.
        tolerance: Positive scalar threshold for the norm of the spatial
            error twist. Assume relative rotation angles are less than pi.

    Returns:
        ``(theta, history)``, where theta is a NumPy array with shape ``(N,)``
        and history has shape ``(M, N)``. Include the initial joint values
        and every update, so ``1 <= M <= max_iterations + 1`` and
        ``theta == history[-1]``. Return the latest values even if the target
        is not reached.
    """

    twists = np.asarray(twists, dtype=float)
    theta = np.asarray(theta, dtype=float).flatten().copy()
    history = [theta.copy()]
    for _ in range(max_iterations):
        pose = forward_kinematics(twists, theta) @ g_st0
        error_twist = spatial_error_twist(spatial_configuration_error(pose, desired_pose))
        if np.linalg.norm(error_twist) <= tolerance:
            break
        # Forward-Euler step along the joint velocity that produces the error twist
        theta = theta + step_size * np.linalg.pinv(spatial_jacobian(twists, theta)) @ error_twist
        history.append(theta.copy())
    return theta, np.array(history)
