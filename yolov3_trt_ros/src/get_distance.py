#!/usr/bin/env python2

import rospy, serial, time, math
import numpy as np
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image as Imageros
from yolov3_trt_ros.msg import BoundingBoxes, BoundingBox


class_dict = {
    0:'left',
    1:'right',
    2:'stop', 
    3:'crosswalk',
    4:'uturn', 
    5:'traffic_light', 
    6:'xycar',
    7:'ignore'
}



motor_msg = xycar_motor()
motor_msg.speed = 0
trt_msg = BoundingBoxes()
obj_id = -1
is_obj_close = False
count = 0

homography = np.load('/home/nvidia/xycar_ws/src/yolov3_trt_ros/src/homography_xycar168.npy')
# homography = np.load('/home/nvidia/xycar_ws/src/yolov3_trt_ros/src/test.npy')

def callback(data) :
    global obj_id, is_obj_close
    for bbox in data.bounding_boxes:
        
        xmin = np.float32(bbox.xmin)
        xmax = np.float32(bbox.xmax)
        ymin = np.float32(bbox.ymin)
        ymax = np.float32(bbox.ymax)

        cx = xmin + (xmax - xmin) / 2
        
        center_point = (cx / 416.0 * 609.0 , ymax / 416.0 * 394.0)

        
        object_centor_point = np.array([center_point[0], center_point[1], 1], dtype=np.float32)

        estimation = np.dot(homography, object_centor_point)
        x, y, z = estimation[0], estimation[1], estimation[2]

        x = x / z
        y = y / z
        z = z / z

        distance = math.sqrt(x**2 + y**2 + z**2)

        # print("object : {} bbox ymax, ymin {}, {} \n bbox xmin, xmax {}, {} \n cx {}, cy {}".format(
        #     class_dict[bbox.id], 
        #     bbox.ymax, 
        #     bbox.ymin, 
        #     bbox.xmin, 
        #     bbox.xmax,
        #     center_point[0],
        #     center_point[1],
        #     )
        # )
        # print("center {} {}".format(center_point[0], center_point[1]))
        print("x : {}, y: {}, z : {}".format(x, y, z))
        print("distance : ", round(distance, 3))
        # if bbox.probability > 0.5 and bbox.ymax > 300:
        #     print("{} : ymax :{}".format(bbox.id, bbox.ymax))
        #     print(bbox.probability)
        
        if distance <= 1.5:
            is_obj_close = True
        else:
            is_obj_close = False
        obj_id = bbox.id
        print(bbox.id)


rospy.init_node('trt_driver')
rospy.Subscriber('/yolov3_trt_ros/detections', BoundingBoxes, callback, queue_size=1)
pub = rospy.Publisher('xycar_motor',xycar_motor,queue_size=1)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    rate.sleep()

