#!/usr/bin/env python
"""
Kinematic function skeleton code for HW2.

Course: EECS C106A, Fall 2026
Originally written by: Aaron Bestick, 9/10/14
Adapted for Fall 2020 by: Amay Saxena, 9/10/20
Adapted for Fall 2026 by: Sebastian Vargas and Valmik Prabhu

This Python file is a code skeleton for HW2. You should fill in
the body of the eight empty methods below so that they implement the kinematic
functions described in the assignment.

When you think you have the methods implemented correctly, you can test your
code by running "python kin_func_skeleton.py" at the command line.
"""

import numpy as np
import scipy.linalg as spl
from scipy.spatial.transform import Rotation

np.set_printoptions(precision=4, suppress=True)

# ----------------------------- 2D Examples ------------------------------------
# -- (You don't need to modify anything here, but you should look at them.) ----


def rotation_2d(theta):
    """
    Computes a 2D rotation matrix given the angle of rotation.

    Args:
    theta: the angle of rotation

    Returns:
    rot - (2,2) ndarray: the resulting rotation matrix
    """

    rot = np.zeros((2, 2))
    rot[0, 0] = np.cos(theta)
    rot[1, 1] = np.cos(theta)
    rot[0, 1] = -np.sin(theta)
    rot[1, 0] = np.sin(theta)

    return rot


def twist_to_se2(xi):
    """
    Converts a 2D twist to its corresponding 3x3 matrix representation

    Args:
    xi - (3,) ndarray: the 2D twist

    Returns:
    xi_hat - (3,3) ndarray: the resulting 3x3 matrix
    """
    if not xi.shape == (3,):
        raise TypeError('omega must be a 3-vector')

    xi_hat = np.zeros((3, 3))
    xi_hat[0, 1] = -xi[2]
    xi_hat[1, 0] = xi[2]
    xi_hat[0:2, 2] = xi[0:2]

    return xi_hat


def twist_to_SE2(xi, theta=1.0):
    """
    Computes a 3x3 homogeneous transformation matrix given a 2D twist and a
    joint displacement

    Args:
    xi - (3,) ndarray: the 2D twist
    theta: the joint displacement

    Returns:
    g - (3,3) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (3,):
        raise TypeError('xi must be a 3-vector')

    g = np.zeros((3, 3))
    wtheta = xi[2] * theta
    R = rotation_2d(wtheta)
    p = np.dot(
        np.dot(
            [
                [1 - np.cos(wtheta), np.sin(wtheta)],
                [-np.sin(wtheta), 1 - np.cos(wtheta)],
            ],
            [[0, -1], [1, 0]],
        ),
        [[xi[0] / xi[2]], [xi[1] / xi[2]]],
    )

    g[0:2, 0:2] = R
    g[0:2, 2:3] = p[0:2]
    g[2, 2] = 1

    return g

# ----------------------------- 3D Functions -----------------------------------
# --------------- (These are the functions you need to complete.) --------------


def R3_to_so3(omega):
    """
    Converts a rotation vector in 3D to its corresponding skew-symmetric matrix.

    Args:
    omega - (3,) ndarray: the rotation vector

    Returns:
    omega_hat - (3,3) ndarray: the corresponding skew symmetric matrix
    """

    # YOUR CODE HERE


def so3_to_R3(omega_hat):
    """
    Converts a skew-symmetric matrix to a rotation vector in R3

    Args:
    omega_hat - (3,3) ndarray: the corresponding skew symmetric matrix

    Returns:
    omega - (3,) ndarray: the rotation vector

    """
    # Check that the input is skew-symmetric.
    assert np.allclose(omega_hat, -omega_hat.T)

    # YOUR CODE HERE


def axis_angle_to_SO3(omega, theta):
    """
    Computes a 3D rotation matrix given a rotation axis and angle of rotation.

    Args:
    omega - (3,) ndarray: the axis of rotation
    theta: the angle of rotation

    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix

    Note! Axes are always unit vectors (though you may need to unitify the input omega)

    """

    # YOUR CODE HERE


def so3_to_SO3(omega_hat, theta=1):
    """
    Converts an element in so(3) to its corresponding rotation matrix in SO(3)

    Args:
    omega_hat - (3,3) ndarray: the skew symmetric matrix
    theta - optional angle

    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix

    Note! omega_hat may not correspond to a unit vector (ie it might have some angle information embedded into it)

    """

    # YOUR CODE HERE


def twist_to_se3(xi, theta=1):
    """
    Converts a 3D twist to its corresponding 4x4 matrix representation

    Args:
    xi - (6,) ndarray: the 3D twist
    theta - optional joint displacement

    Returns:
    xi_hat - (4,4) ndarray: the corresponding 4x4 matrix

    Note: xi need not be a unit twist! (ie it may have some displacement information embedded into it)
    """

    # YOUR CODE HERE



def se3_to_twist(xi_hat):
    """
    Converts a 4x4 matrix in se(3) to the corresponding twist

    Args:
    xi_hat - (4,4) ndarray: the corresponding 4x4 matrix

    Returns:
    xi - (6,) ndarray: the 3D twist
    """

    # YOUR CODE HERE


def twist_to_SE3(xi, theta = 1)
    """
    Converts a 3D twist and optional angle to a 4x4 rigid body transformation in SE(3)

    Args:
    xi - (6,) ndarray: the 3D twist
    theta - optional joint displacement

    Returns:
    g - (4,4) ndarray: the resulting homogenous transformation matrix

    Note: xi need not be a unit twist! (ie it may have some displacement information embedded into it)

    """


def se3_to_SE3(xi_hat, theta=1):
    """
    Converts a 4x4 matrix in se(3) to a 4x4 rigid body transformation in SE(3)

    Args:
    xi_hat - (4,4) ndarray: matrix in se(3)
    theta - optional joint displacement

    Returns:
    g - (4,4) ndarray: the resulting homogenous transformation matrix

    Note: xi_hat need not correspond to a unit twist! (ie it may have some displacement information embedded into it)

    """

    # YOUR CODE HERE


def forward_kinematics(xi, theta):
    """
    Computes the product of exponentials for a kinematic chain, given
    the twists and displacements for each joint.

    Args:
    xi - (6, N) ndarray: the twists for each joint
    theta - (N,) ndarray: the displacement of each joint

    Returns:
    g - (4,4) ndarray: the resulting homogeneous transformation matrix
    """

    # YOUR CODE HERE


# ------------------------- Other Helper Functions -----------------------------
# ---- (These are completed for you and can be used in this assignment or for future problems) ------

def unitify(x):
    """
    Turns an input vector into a unit vector

    Args:
    """
    norm = spl.norm(x)
    return x / norm if norm > 1e-10 else np.zeros_like(x)

def inverse_SO3(R):
    """
    Computes the inverse of a rotation matrix
    """
    return R.T


def SO3_to_axis_angle(R):
    """
    Converts a rotation matrix to axis angle form
    """

    theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))
    if np.isclose(theta, 0.0):
        omega = np.zeros((3,))
    elif np.isclose(theta, np.pi):
        eigenvalues, eigenvectors = np.linalg.eig(R)
        index = np.argmin(np.abs(eigenvalues - 1.0))
        omega = np.real(eigenvectors[:, index])
        omega = unitify(omega)
    else:
        omega = np.array(
            [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]
        )
        omega = (1 / (2 * np.sin(theta))) * omega

    return omega, theta


def SO3_to_so3(R):
    """
    Converts an SO3 matrix back into so3 form
    """

    omega, theta = SO3_to_axis_angle(R)
    return R3_to_so3(omega * theta)


def pR_to_g(p, R):
    """
    converts a position and orientation into a rigid body transform
    """
    g = np.eye(4)
    g[:3, :3] = R
    g[:3, 3] = p
    return g


def g_to_pR(g):
    """
    decomposes a rigid body transform into a position and orientation
    """
    p = g[:3, 3]
    R = g[:3, :3]
    return p, R


def inverse_SE3(g):
    """
    Returns the inverse of a rigid body transform
    """
    return np.linalg.inv(g)


def SE3_to_se3(g):
    """
    Converts an SE3 matrix back into se3 form
    """
    return np.real_if_close(spl.logm(g))


def renormalize_SO3(R):
    """
    Sometimes when doing numerical integration on SO3, numerical errors accumulate
    and your rotation matrix is no longer in SO(3). Ie it's no longer orthogonal
    This code projects the input matrix to the closest matrix in SO3

    Use the scipy Rotation package:
    https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_matrix.html#scipy.spatial.transform.Rotation.from_matrix

    Args:
    R: (3, 3) array

    Returns:
    (3, 3) array - Nearest SO3 approximation of R

    """

    return Rotation.from_matrix(R).as_matrix()


def renormalize_SE3(g):
    """
    Renormalizes the rotation in a RBT
    """

    p, R = g_to_pR(g)
    R = renormalize_SO3(R)
    return pR_to_g(p, R)




# ------------------------------- Testing Code ---------------------------------
# ---------------------- Do not modify anything below here. --------------------


def array_func_test(func_name, args, ret_desired):
    ret_value = func_name(*args)
    if not isinstance(ret_value, np.ndarray):
        print(
            '[FAIL] '
            + func_name.__name__
            + '() returned something other than a NumPy ndarray'
        )
    elif ret_value.shape != ret_desired.shape:
        print(
            '[FAIL] '
            + func_name.__name__
            + '() returned an ndarray with incorrect dimensions'
        )
    elif not np.allclose(ret_value, ret_desired, rtol=1e-3):
        print('[FAIL] ' + func_name.__name__ + '() returned an incorrect value')
    else:
        print('[PASS] ' + func_name.__name__ + '() returned the correct value!')


if __name__ == "__main__":
    print('Testing...')

    # Test R3_to_so3()
    arg1 = np.array([1.0, 2, 3])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -3.,  2.],
                            [ 3., -0., -1.],
                            [-2.,  1.,  0.]])
    array_func_test(R3_to_so3, func_args, ret_desired)

    # Test so3_to_R3()
    arg1 = np.array([[ 0., -3.,  2.],
                     [ 3., -0., -1.],
                     [-2.,  1.,  0.]])
    func_args = (arg1,)
    ret_desired = np.array([1., 2., 3.])
    array_func_test(so3_to_R3, func_args, ret_desired)

    # Test axis_angle_to_SO3()
    arg1 = np.array([2.0, 1, 3])
    arg2 = 0.587
    func_args = (arg1, arg2)
    ret_desired = np.array([[-0.1325, -0.4234,  0.8962],
                            [ 0.8765, -0.4723, -0.0935],
                            [ 0.4629,  0.7731,  0.4337]])
    array_func_test(axis_angle_to_SO3, func_args, ret_desired)

    # Test so3_to_SO3()
    arg1 = np.array([[ 0.0000, -1.7610,  0.5870],
                     [ 1.7610,  0.0000, -1.1740],
                     [-0.5870,  1.1740,  0.0000]])
    func_args = (arg1,)
    ret_desired = np.array([[-0.1325, -0.4234,  0.8962],
                            [ 0.8765, -0.4723, -0.0935],
                            [ 0.4629,  0.7731,  0.4337]])
    array_func_test(so3_to_SO3, func_args, ret_desired)

    # Test twist_to_se3()
    arg1 = np.array([2.0, 1, 3, 5, 4, 2])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -2.,  4.,  2.],
                            [ 2., -0., -5.,  1.],
                            [-4.,  5.,  0.,  3.],
                            [ 0.,  0.,  0.,  0.]])
    array_func_test(twist_to_se3, func_args, ret_desired)

    # Test se3_to_SE3()
    arg1 = np.array([[ 0., -2.,  4.,  2.],
                     [ 2., -0., -5.,  1.],
                     [-4.,  5.,  0.,  3.],
                     [ 0.,  0.,  0.,  0.]])
    arg2 = 0.658
    func_args = (arg1, arg2)
    ret_desired = np.array([[ 0.4249,  0.8601, -0.2824,  1.7814],
                            [ 0.2901,  0.1661,  0.9425,  0.9643],
                            [ 0.8575, -0.4824, -0.179 ,  0.1978],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(se3_to_SE3, func_args, ret_desired)

    # Test se3_to_twist()
    arg1 = np.array([[ 0., -2.,  4.,  2.],
                     [ 2., -0., -5.,  1.],
                     [-4.,  5.,  0.,  3.],
                     [ 0.,  0.,  0.,  0.]])
    func_args = (arg1,)
    ret_desired = np.array([2., 1., 3., 5., 4., 2.])
    array_func_test(se3_to_twist, func_args, ret_desired)

    # Test forward_kinematics()
    arg1 = np.array([[2.0, 1, 3, 5, 4, 6], [5, 3, 1, 1, 3, 2], [1, 3, 4, 5, 2, 4]]).T
    arg2 = np.array([0.658, 0.234, 1.345])
    func_args = (arg1, arg2)
    ret_desired = np.array([[ 0.4392,  0.4998,  0.7466,  7.6936],
                            [ 0.6599, -0.7434,  0.1095,  2.8849],
                            [ 0.6097,  0.4446, -0.6562,  3.3598],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(forward_kinematics, func_args, ret_desired)

    print('Done!')
