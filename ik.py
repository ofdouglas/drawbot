import numpy as np
import math
import fk
import arm

# Constants
err_des = 0.5
e_step = err_des * 0.5
d_theta = 0.01
max_loops = 5000

# Get Jacobian corresponding to current angles
def get_J(theta):
    
    # Incremental change in joint angles...
    theta0_new = np.array([d_theta, 0, 0, 0]) + theta
    theta1_new = np.array([0, d_theta, 0, 0]) + theta
    theta2_new = np.array([0, 0, d_theta, 0]) + theta
    theta3_new = np.array([0, 0, 0, d_theta]) + theta
    
    # ...and the resulting change in end effector positions
    cur_pos = fk.get_fk(theta)
    de0 = np.subtract(fk.get_fk(theta0_new), cur_pos)
    de1 = np.subtract(fk.get_fk(theta1_new), cur_pos)
    de2 = np.subtract(fk.get_fk(theta2_new), cur_pos)
    de3 = np.subtract(fk.get_fk(theta3_new), cur_pos)

    # Jacobian matrix: [dx/dtheta_0, dx/dtheta_1, ... ]
    J = np.array(
        [ [de0[0]/d_theta, de1[0]/d_theta, de2[0]/d_theta, de3[0]/d_theta],
          [de0[1]/d_theta, de1[1]/d_theta, de2[1]/d_theta, de3[0]/d_theta],
          [de0[2]/d_theta, de1[2]/d_theta, de2[2]/d_theta, de3[0]/d_theta] ])

    return J


# Pick incremental change in end effector position
def get_d_pos(e_goal, e_current):
    
    theta_xy = math.atan2((e_goal[1] - e_current[1]),
                      (e_goal[0] - e_current[0]))
    dx = e_step * math.cos(theta_xy)
    dy = e_step * math.sin(theta_xy)

    rho_goal = math.sqrt(e_goal[0]**2 + e_goal[1]**2)
    rho_current = math.sqrt(e_current[0]**2 + e_current[1]**2)
    theta_z = math.atan2((e_goal[2] - e_current[2]),
                         (rho_goal - rho_current))
    dz = e_step * math.sin(theta_z)
        
    return np.array([dx, dy, dz])


# Return vector of joint angles needed to reach goal position
#  cur_theta - current joint angles
#  goal_pos  - goal position
#
def plan_move(cur_theta, goal_pos):

    count = 0
    err_cur = 2 * err_des
    theta = cur_theta
    
    while err_cur > err_des:
        # Find incremental angle adjustment using Jacobian
        J = get_J(theta)
        J_inv = np.linalg.pinv(J)
        cur_pos = fk.get_fk(theta)
        d_pos = get_d_pos(goal_pos, cur_pos)
        d_theta = np.dot(J_inv, d_pos)

        # Limit joint angles (based on mechanical limits of the arm)
        for i in range(len(theta)):
            enc = arm.rads_to_encoder(theta[i], arm.encoder_offsets[i])
            if enc < joint_limits[i][0]:
                print "Joint " + str(i) + " hit lower limit"
                theta[i] = 0.75 * arm.encoder_to_rads(joint_limits[i][0],
                                                  arm.encoder_offsets[i])
            elif enc > joint_limits[i][1]:
                print "Joint " + str(i) + " hit upper limit"
                theta[i] = 0.75 * arm.encoder_to_rads(joint_limits[i][1],
                                                  arm.encoder_offsets[i])
            else:
                theta[i] = theta[i] + d_theta[i]

        # Calculate how far off we are from the desired position
        cur_pos = fk.get_fk(theta)        
        err_cur = math.sqrt((goal_pos[0] - cur_pos[0])**2 +
                            (goal_pos[1] - cur_pos[1])**2 +
                            (goal_pos[2] - cur_pos[2])**2)

        # Show the current error amount
        # print str(count) + ': ', err_cur

        # If an excessive number of iterations occur, the position may be
        # unreachable 
        count = count + 1
        if i > max_loops:
            break

    if count > max_loops:
        print "\nIK: failed after", count, "iterations\n"
    else:
        print "\nIK: succeeded after", count, "iterations\n"
        
 #   print "\nFinal position:\n" + "Theta = " + str(theta * 180/math.pi)
 #   print "[X, Y, Z] = " + str(cur_pos)

    
    return theta
