import numpy as np
import math
import sys
import time
import cv2

import fk
import ik
import arm


# Draw a dot at the given (X, Y) coordinate
def draw_point(chain, (px, py)):
    theta = arm.get_arm_angles(chain)

    # Move arm close to correct X,Y position, with Z offset to avoid hitting paper
    theta = ik.plan_move(theta, np.array([px, py, z_off + 5]))
    arm.move_arm(chain, theta)

    # Depress pen onto paper
    theta = ik.plan_move(theta, np.array([px, py, z_off]))
    arm.move_arm(chain, theta)
    
    # Lift pen back up
    theta = ik.plan_move(theta, np.array([px, py, z_off + 5]))
    arm.move_arm(chain, theta)

                        
# Draw a circle 
def circle_test(chain):
    center = 120
    for i in range(0, 360, 2):
        x = 25 * math.cos(i*math.pi/180.0)
        y = center + 25 * math.sin(i*math.pi/180.0)
        draw_point(chain, (x, y))

        
# Draw a square
def square_test(chain):
    corner_y = 50
    corner_x = 50
    length = 20
    
    for i in range(0, length):
        x = corner_x + i
        y = corner_y
        draw_point(chain, (x, y))

    for i in range(0, length):
        x = corner_x + length
        y = corner_y + i
        draw_point(chain, (x, y))

    for i in range(0, length):
        x = corner_x + length - i
        y = corner_y + length
        draw_point(chain, (x, y))

    for i in range(0, length):
        x = corner_x 
        y = corner_y + length - i
        draw_point(chain, (x, y))


# Draw a grid of dots
def grid_test(chain):
    x1 = -60
    x2 = 60
    y1 = 85
    y2 = 175

    for j in range(y1, y2, 5):
        for i in range(x1, x2, 5):
                draw_point(chain, (i, j))

                
# Read an image from a file and draw it's edges
def draw_image(chain):
    image_width = 120
    image_height = 90
    ratio = 1
    x_off = -image_width / (2 * ratio)
    y_off = 85
    
    img = cv2.imread('image.jpg')
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.resize(img, (image_width, image_height))
    img = cv2.Canny(img, 100, 200)

#   cv2.imshow('edges', edges)
#   cv2.waitKey(0)
    print img

    for y in range(0, image_height):
        for x in range(0, image_width):
            if img[y][x] > 128:
                draw_point(chain, (x/ratio + x_off, y/ratio + y_off))
                
    

def main():
    chain = arm.connect_to_dynamixels()
  # config_motors(chain)
    
  # grid_test(chain)
  # draw_point(chain, (0, 80))
  # draw_image(chain)
  # square_test(chain)
  # circle_test(chain)
    
    # while True:
    #     time.sleep(1)
    #     theta =  get_arm_angles(chain)
    #     print theta * 180/math.pi



    
if __name__ == "__main__":
    main()

