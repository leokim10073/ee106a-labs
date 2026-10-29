"""Functions to help with Jacobian calculations."""

import numpy as np

import kin_func_skeleton #keep this because the lab repo doesn't know about hw2


def adjoint(g):
    """Calculate the adjoint matrix of a rigid transformation.

    Args:
        g: NumPy array with shape ``(4, 4)`` representing a
            transformation in SE(3).

    Returns:
        NumPy array with shape ``(6, 6)`` representing ``Ad_g``.
    """

    # TODO
    return ...


def adjoint_inv(g):
    """Calculate the inverse adjoint matrix of a rigid transformation.

    Args:
        g: NumPy array with shape ``(4, 4)`` representing a
            transformation in SE(3).

    Returns:
        NumPy array with shape ``(6, 6)`` representing ``Ad_g^{-1}``.
    """

    # TODO
    return ...


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

    # TODO
    return ...


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

    # TODO
    return ...


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

    # TODO
    return ...


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

    # TODO
    return ...





def jacobian_ik_step(

    theta, velocity, step_size=0.2, twists=PANDA_TWISTS, g_st0=PANDA_G_ST0,

):

    """Apply one forward-Euler Jacobian IK update for the supplied robot."""

    jacobian = body_jacobian(twists, theta, g_st0)

    return theta + step_size * np.linalg.pinv(jacobian) @ velocity





def null_space_ik_step(

    theta,

    velocity,

    secondary_gradient,

    step_size=0.2,

    secondary_gain=0.05,

):

    """Apply one Panda IK update with a projected secondary objective."""

    jacobian = body_jacobian(PANDA_TWISTS, theta, PANDA_G_ST0)

    pseudoinverse = np.linalg.pinv(jacobian)

    primary = pseudoinverse @ velocity

    null_motion = (

        np.eye(7) - pseudoinverse @ jacobian

    ) @ secondary_gradient

    return theta + step_size * (

        primary + secondary_gain * null_motion

    )





def joint_limit_gradient(theta):

    """Compute the Panda joint-limit avoidance direction."""

    midpoint = (PANDA_THETA_MIN + PANDA_THETA_MAX) / 2

    half_range = (PANDA_THETA_MAX - PANDA_THETA_MIN) / 2

    return -(theta - midpoint) / half_range**2