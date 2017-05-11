import cv2

image_width = 640
image_height = 480
ratio = 4
x_off = -image_width / (2 * ratio)
y_off = 80

img = cv2.imread('image.jpg')
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img = cv2.resize(img, (image_width, image_height))
edges = cv2.Canny(img, 100, 200)

#print edges
cv2.imshow('edges', edges)
cv2.waitKey(0)
