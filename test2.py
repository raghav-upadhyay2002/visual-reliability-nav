from controller import Robot
import numpy as np
import cv2
import csv
from controllers.classic_cv import detect_walls_countours, detect_walls_lines

robot = Robot()
timestep= int(robot.getBasicTimeStep())



#Cameras

#camera_front
front_camera= robot.getDevice('camera')
front_camera.enable(timestep)
print("Front Camera resolution: ", front_camera.getWidth(), "x", front_camera.getHeight())




#motors
left_motor= robot.getDevice('left wheel motor')
right_motor= robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)




#main loop
while robot.step(timestep) != -1:

    #raw image from front camera
    front_image= front_camera.getImage()
    width= front_camera.getWidth()
    height= front_camera.getHeight()

    #reshape the flat buffer to a 3D array (height, width, channels)
    front_image= np.frombuffer(front_image, np.uint8).reshape((height, width, 4))


    #drop the alpha channel
    front_image_bgr= cv2.cvtColor(front_image, cv2.COLOR_BGRA2BGR)


    #show the image in a window
    cv2.imshow("Front Camera", cv2.resize(front_image_bgr, (300, 300)))


    #required for Open cv to refresh the window
    cv2.waitKey(1)


#close the window when the simulation ends
cv2.destroyAllWindows()




