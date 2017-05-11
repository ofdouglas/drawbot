from dynamixel_hr.dxl.dxlchain import DxlChain
import math
import numpy as np

# Dynamixel encoder precision & range
rads_per_tick = 300.0/1023.0 * math.pi / 180.0
max_tick = 1023
min_tick = 0

# The encoder angxle is 150 degrees ahead of our angle, so we need to
# add or subtract this many ticks when converting rads <-> ticks
# Encoder offset for J1 differs because of how arm supports attach
encoder_offsets = [511, 808, 511, 511]

# Joint limits (in encoder ticks)
# These joint limits are purely based on the amount each servo can rotate
# before hitting another part of the arm.
joint_limits = [(0, 1023), (540, 940), (70, 1010), (170, 840)]

# Convert an angle from encoder ticks to radians, using the given offset
# (the encoder position corresponding to an angle of 0)
def encoder_to_rads(enc, offs):
    return (offs - enc) * rads_per_tick

# Convert an angle from radians to encoder ticks
def rads_to_encoder(rad, offs):
    return offs - (rad / rads_per_tick)


# Print the current joint angles from the arm servos, in degrees
def print_arm_pos(chain):
    theta = get_arm_angles(chain)
    pos = fk.get_fk(theta)
    rad2deg = 180 / math.pi
    
    print "Joint angles: ", [theta[0] * rad2deg, theta[1] * rad2deg, \
                             theta[2] * rad2deg, theta[3] * rad2deg]

    print "Position:     ", pos


# Return the current joint angles from the arm servos, in radians
def get_arm_angles(chain):
    pos = None
    
    while True:
        try:
            pos = chain.get_position()
            break
        except:
            print "Get position failed"

    theta = np.array([0.0, 0, 0, 0])
    theta[0] = encoder_to_rads(pos[1], encoder_offsets[0])
    theta[1] = encoder_to_rads(pos[2], encoder_offsets[1])
    theta[2] = encoder_to_rads(pos[3], encoder_offsets[2])
    theta[3] = encoder_to_rads(pos[4], encoder_offsets[3])
    return theta


# Set joint angles in the arm to the given angle vector
def move_arm(chain, theta):
    t0 = rads_to_encoder(theta[0], encoder_offsets[0])
    t1 = rads_to_encoder(theta[1], encoder_offsets[1])
    t2 = rads_to_encoder(theta[2], encoder_offsets[2])
    t3 = rads_to_encoder(theta[3], encoder_offsets[3])

    chain.sync_write_pos_speed((1,2,3,4), (t0, t1, t2, t3),
                                       (180,180,180,180))

    # Wait for move to finish. Because some joints may move farther than
    # others, the next move could overlap with this one if we don't wait
    while chain.is_moving():
        pass
    time.sleep(0.05)


            
# Modify the configuration of the arm servos
def config_chain(chain):
    config = chain.get_configuration();
    config[1]['cw_compliance_slope'] = 64
    config[1]['ccw_compliance_slope'] = 64

    config[2]['cw_compliance_slope'] = 64
    config[2]['ccw_compliance_slope'] = 64

    config[3]['cw_compliance_slope'] = 64
    config[3]['ccw_compliance_slope'] = 64

    config[4]['torque_limit'] = 250
    config[4]['cw_compliance_slope'] = 128
    config[4]['ccw_compliance_slope'] = 128
    chain.set_configuration(config)


# Connect to the dynamixel servos
def connect_to_dynamixels():
    for i in range(10):
        try:
            chain = DxlChain("/dev/ttyUSB0", rate=200000)
            motors = chain.get_motor_list()
            # Return if connection succeeded
            if (len(motors)) == 4:
                return chain
        except:
            # Connection failed, but we'll try 10 times before giving up
            pass
    else:
        print "connect_to_dynamixels failed.\n"
        exit()
