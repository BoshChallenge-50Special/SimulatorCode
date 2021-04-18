import math
import numpy as np

def car_to_world(x,y,alpha, point):
    if(len(point)<3):
        point=point+[1]
    matrix=np.array([[math.cos(alpha), -math.sin(alpha), x],
                     [math.sin(alpha), math.cos(alpha), y],
                     [0, 0, 1]])

    return matrix.dot(np.array(point)).tolist()

def world_to_car(x,y,alpha, point):
    if(len(point)<3):
        point=point+[1]
    rotation=np.array([[ math.cos(alpha), math.sin(alpha)],
                     [ -math.sin(alpha),  math.cos(alpha)]])
    translation = np.array([x,y])
    translation= -rotation.dot(translation.transpose())

    matrix=np.array([[ math.cos(alpha), math.sin(alpha),  translation[0]],
                     [ -math.sin(alpha),  math.cos(alpha), translation[1]],
                     [0, 0, 1]])
    return matrix.dot(np.array(point)).tolist()

def camera_to_car_ref(x, y):
    width = 640
    height = 480

    hor_fov = 1.085594795
    ver_fov= 1.085594795*height/width

    x_RF_center = x - width/2
    y_RF_center = y - height/2

    x_RF_center_degree = x_RF_center/(width/2)*(hor_fov/2)
    y_RF_center_degree = y_RF_center/(height/2)*(ver_fov/2)

    hypotenuse_air = 0.2/math.cos(math.pi/2 - (0.2617 + y_RF_center_degree)) #ipotenusa, 0.2 is the high of the camera on the street
    y_car =  math.sin(math.pi/2 - (0.2617 + y_RF_center_degree))*hypotenuse_air
    print(x_RF_center_degree)
    hypotenuse = y_car/math.cos(x_RF_center_degree) #ipotenusa, 0.2 is the high of the camera on the street
    x_car =  math.sin(x_RF_center_degree)*hypotenuse


    return x_car, y_car
