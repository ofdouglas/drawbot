import numpy as np
import math

# Arm lengths (mm)
l0 = 82         # z
l1 = 75         # x
l2 = 15         # z
l3 = 67.5       # x
l4 = 45         # x
l5 = -53        # z


def get_translation_matrix(x,y,z):
    return np.matrix([[x],[y],[z]])


def get_rotation_matrix(axis_str,theta):
    if axis_str == 'x':
        return np.matrix([[1, 0, 0],
                         [0, math.cos(theta), -math.sin(theta)],
                         [0, math.sin(theta), math.cos(theta)]])
    elif axis_str == 'y':
        return np.matrix([[math.cos(theta), 0, math.sin(theta)],
                         [0,                1,               0],
                         [-math.sin(theta), 0, math.cos(theta)]])
    elif axis_str == 'z':
        return np.matrix([[math.cos(theta), -math.sin(theta), 0],
                         [math.sin(theta),  math.cos(theta),  0],
                         [0,                0,                1]])
    else:
        return None


def get_transformation_matrix(x,y,z,theta,axis_str):
    tran = get_translation_matrix(x,y,z)
    rot  = get_rotation_matrix(axis_str,theta)

    matA = np.concatenate((rot, np.matrix([[0],[0],[0]])), axis=1)
    matA = np.concatenate((matA, np.matrix([0, 0, 0, 1])), axis=0)

    matB = np.concatenate((np.eye(3), tran), axis=1)
    matB = np.concatenate((matB, np.matrix([0, 0, 0, 1])), axis=0)

    return np.dot(matA,matB)


# Forward kinematics: find end effector position [X, Y, Z] from joint angles
def get_fk(theta):

    T0 = get_transformation_matrix(0,0,l0,theta[0],'z')
    T1 = get_transformation_matrix(0,l1,l2,theta[1],'x')
    T2 = get_transformation_matrix(0,l3,0,theta[2],'x')
    T3 = get_transformation_matrix(0,l4,l5,theta[3],'x')
    TE = np.dot(np.dot(np.dot(T0,T1),T2),T3)
        
    x = TE[0,3]
    y = TE[1,3]
    z = TE[2,3]

    # print
    # print T0, "\n"
    # print T1, "\n"
    # print T2, "\n"
    # print T3, "\n"
    # print TE,
    # exit()

    return [x, y, z]
