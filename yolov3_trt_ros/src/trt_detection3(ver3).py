#!/usr/bin/env python3
#
# Copyright 1993-2019 NVIDIA Corporation.  All rights reserved.
#
# NOTICE TO LICENSEE:
#
# This source code and/or documentation ("Licensed Deliverables") are
# subject to NVIDIA intellectual property rights under U.S. and
# international Copyright laws.
#
# These Licensed Deliverables contained herein is PROPRIETARY and
# CONFIDENTIAL to NVIDIA and is being provided under the terms and
# conditions of a form of NVIDIA software license agreement by and
# between NVIDIA and Licensee ("License Agreement") or electronically
# accepted by Licensee.  Notwithstanding any terms or conditions to
# the contrary in the License Agreement, reproduction or disclosure
# of the Licensed Deliverables to any third party without the express
# written consent of NVIDIA is prohibited.
#
# NOTWITHSTANDING ANY TERMS OR CONDITIONS TO THE CONTRARY IN THE
# LICENSE AGREEMENT, NVIDIA MAKES NO REPRESENTATION ABOUT THE
# SUITABILITY OF THESE LICENSED DELIVERABLES FOR ANY PURPOSE.  IT IS
# PROVIDED "AS IS" WITHOUT EXPRESS OR IMPLIED WARRANTY OF ANY KIND.
# NVIDIA DISCLAIMS ALL WARRANTIES WITH REGARD TO THESE LICENSED
# DELIVERABLES, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY,
# NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE.
# NOTWITHSTANDING ANY TERMS OR CONDITIONS TO THE CONTRARY IN THE
# LICENSE AGREEMENT, IN NO EVENT SHALL NVIDIA BE LIABLE FOR ANY
# SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
# DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
# WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
# ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
# OF THESE LICENSED DELIVERABLES.
#
# U.S. Government End Users.  These Licensed Deliverables are a
# "commercial item" as that term is defined at 48 C.F.R. 2.101 (OCT
# 1995), consisting of "commercial computer software" and "commercial
# computer software documentation" as such terms are used in 48
# C.F.R. 12.212 (SEPT 1995) and is provided to the U.S. Government
# only as a commercial end item.  Consistent with 48 C.F.R.12.212 and
# 48 C.F.R. 227.7202-1 through 227.7202-4 (JUNE 1995), all
# U.S. Government End Users acquire the Licensed Deliverables with
# only those rights set forth herein.
#
# Any use of the Licensed Deliverables in individual and commercial
# software must include, in the user documentation and internal
# comments to the code, the above Disclaimer and U.S. Government End
# Users Notice.

# Programmers Dev Course(Self Driving)
# Object Detection Project
# 2023 06 19 ~ 2023 07 06
# Written By Team1 (Justin)
# @ author Jeong Min Kim
# @ author Jung Eun Heo
# @ author Seung Bin Sim
# @ authro Dae Won Kim
# ADD func
# 1. traffic right detection
# 2. get distance and location from object


import sys, os
import time
import numpy as np
import cv2
import math
import tensorrt as trt
from PIL import Image, ImageDraw
import rospy

from std_msgs.msg import String
from yolov3_trt_ros.msg import BoundingBox, BoundingBoxes

from cv_bridge import CvBridge
from sensor_msgs.msg import Image as Imageros

from data_processing import PreprocessYOLO, PostprocessYOLO, ALL_CATEGORIES
import common

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

CFG = "/home/nvidia/xycar_ws/src/lane_keeping_system/yolov3_trt_ros/src/yolov3-tiny_tstl_416_my.cfg"
TRT = '/home/nvidia/xycar_ws/src/lane_keeping_system/yolov3_trt_ros/src/model_epoch2500v2.trt'
NUM_CLASS = 8
INPUT_IMG = '/home/nvidia/xycar_ws/src/yolov3_trt_ros/src/video1_2.png'

xycar_image = np.uint8([0])
bridge = CvBridge()

class yolov3_trt(object):
    def __init__(self):
        self.cfg_file_path = CFG
        self.num_class = NUM_CLASS
        width, height, masks, anchors = parse_cfg_wh(self.cfg_file_path)
        self.engine_file_path = TRT
        self.show_img = False

        # Two-dimensional tuple with the target network's (spatial) input resolution in HW ordered
        input_resolution_yolov3_WH = (width, height)
        # Create a pre-processor object by specifying the required input resolution for YOLOv3
        self.preprocessor = PreprocessYOLO(input_resolution_yolov3_WH)

        # Output shapes expected by the post-processor
        output_channels = (self.num_class + 5) * 3
        if len(masks) == 2:
            self.output_shapes = [(1, output_channels, height // 32, width // 32),
                                  (1, output_channels, height // 16, width // 16)]
        else:
            self.output_shapes = [(1, output_channels, height // 32, width // 32),
                                  (1, output_channels, height // 16, width // 16),
                                  (1, output_channels, height // 8, width // 8)]

        postprocessor_args = {"yolo_masks": masks,  # A list of 3 three-dimensional tuples for the YOLO masks
                              "yolo_anchors": anchors,
                              "obj_threshold": 0.5,  # Threshold for object coverage, float value between 0 and 1
                              "nms_threshold": 0.3,
                              # Threshold for non-max suppression algorithm, float value between 0 and 1
                              "yolo_input_resolution": input_resolution_yolov3_WH,
                              "num_class": self.num_class}

        self.postprocessor = PostprocessYOLO(**postprocessor_args)

        self.engine = get_engine(self.engine_file_path)

        self.context = self.engine.create_execution_context()

        self.detection_pub = rospy.Publisher('/yolov3_trt_ros/detections', BoundingBoxes, queue_size=1)

    # traffic light r, b detection
    def rgb_detection(self, bboxes_info):
        global xycar_image
        # if no image -> return image_raw
        if bboxes_info is None:
            return 0

        # box score, xmin, xmax, ymin, ymax => bboxes infos
        traffic_roi = []

        for bbox in bboxes_info:
            # if the bbox is a traffic light
            if bbox.id == 5:
                # add bbox -> traffic_roi
                traffic_roi.append(bbox)
            else:
                pass
        
        # roi area
        for roi in traffic_roi:
            # crop image
            cropped_image = xycar_image[roi.ymin:roi.ymax, roi.xmin:roi.xmax]

            # blur processing for accurate circle detection, use median blur
            blur_image = cv2.medianBlur(cropped_image, 5)

            # BGR -> HSV
            hsv = cv2.cvtColor(blur_image, cv2.COLOR_BGR2HSV)
            # split h, s, v -> to use v
            h, s, v = cv2.split(hsv)

            # detect circle
            circles = cv2.HoughCircles(image=v,
                                        method=cv2.HOUGH_GRADIENT,
                                        dp=1,
                                        minDist=20,
                                        param1=25, param2=25,
                                        minRadius=1, maxRadius=100)
            if circles is None:
                return

            circles = np.uint8(np.around(circles))

            # define the lower, upper boundaries for red, green color
            red_lower = np.array([0, 100, 100])
            red_upper = np.array([10, 255, 255])

            green_lower = np.array([45, 90, 90])
            green_upper = np.array([100, 255, 255])

            yellow_lower = np.array([20, 100, 120])
            yellow_upper = np.array([35, 255, 255])

            for i in circles[0, :]:
                # circle coordinate value, average value of pixels in a circle
                cr_image_h = h[i[1] - 10: i[1] + 10, i[0] - 10: i[0] + 10]
                cr_image_s = s[i[1] - 10: i[1] + 10, i[0] - 10: i[0] + 10]
                cr_image_v = v[i[1] - 10: i[1] + 10, i[0] - 10: i[0] + 10]

                cr_image = cv2.merge((cr_image_h, cr_image_s, cr_image_v))
                # image_str = 'x : {0}, y : {1}, mean : {2}'.format(i[0], i[1], cr_image.mean())

                if cr_image is None:
                    continue

                # draw a circle
                cv2.circle(cropped_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
                # draw the center point of a circle
                cv2.circle(cropped_image, (i[0], i[1]), 2, (0, 0, 255), 3)

                # resize images to the same size
                resized_image = cv2.resize(cropped_image, (cr_image.shape[1], cr_image.shape[0]))

                # uses mask images to detect only the specified range of colors within the hsv image
                # red version
                red_mask = cv2.inRange(cr_image, red_lower, red_upper)
                red_result = cv2.bitwise_and(resized_image, resized_image, mask=red_mask)

                # green version
                green_mask = cv2.inRange(cr_image, green_lower, green_upper)
                green_result = cv2.bitwise_and(resized_image, resized_image, mask=green_mask)

                # yellow version
                yellow_mask = cv2.inRange(cr_image, yellow_lower, yellow_upper)
                yellow_result = cv2.bitwise_and(resized_image, resized_image, mask=yellow_mask)

                # off = 0, red = 1, yellow = 2, green = 2
                result = 0

                # pixel mean value is nonzero -> determined that a color has been detected
                if cr_image.mean() >= 90:
                    if red_result.mean() >= 2: result = 1
                    if yellow_result.mean() >= 2: result = 2
                    if green_result.mean() >= 2: result = 3

                # traffic light is off, result = 0
                if cropped_image.mean() <= 60:
                    result = 0 
                    
        return result 
        

    def detect(self):
        rate = rospy.Rate(10)
        image_sub = rospy.Subscriber("/usb_cam/image_raw", Imageros, img_callback)
        while not rospy.is_shutdown():
            rate.sleep()

            # Do inference with TensorRT

            inputs, outputs, bindings, stream = common.allocate_buffers(self.engine)

            # if xycar_image is empty, skip inference
            if xycar_image.shape[0] == 0:
                continue

            if self.show_img:
                cv2.imshow("show_trt", xycar_image)
                cv2.waitKey(1)

            image = self.preprocessor.process(xycar_image)
            # Store the shape of the original input image in WH format, we will need it for later
            shape_orig_WH = (image.shape[3], image.shape[2])
            # Set host input to the image. The common.do_inference function will copy the input to the GPU before executing.
            start_time = time.time()
            inputs[0].host = image
            trt_outputs = common.do_inference(self.context, bindings=bindings, inputs=inputs, outputs=outputs,
                                              stream=stream)

            # Before doing post-processing, we need to reshape the outputs as the common.do_inference will give us flat arrays.
            trt_outputs = [output.reshape(shape) for output, shape in zip(trt_outputs, self.output_shapes)]

            # Run the post-processing algorithms on the TensorRT outputs and get the bounding box details of detected objects
            boxes, classes, scores = self.postprocessor.process(trt_outputs, shape_orig_WH)

            latency = time.time() - start_time
            fps = 1 / latency

            # publish detected objects boxes and classes
            self.publisher(boxes, scores, classes)

            # Draw the bounding boxes onto the original input image and save it as a PNG file
            # print(boxes, classes, scores)
            if self.show_img:
                img_show = np.array(np.transpose(image[0], (1, 2, 0)) * 255, dtype=np.uint8)
                obj_detected_img = draw_bboxes(Image.fromarray(img_show), boxes, scores, classes, ALL_CATEGORIES)
                obj_detected_img_np = np.array(obj_detected_img)
                show_img = cv2.cvtColor(obj_detected_img_np, cv2.COLOR_RGB2BGR)
                cv2.putText(show_img, "FPS:" + str(int(fps)), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, 1)
                cv2.imshow("result", show_img)
                cv2.waitKey(1)

    def get_location_and_distance(self, bbox):

        homography = np.load('/home/nvidia/xycar_ws/src/lane_keeping_system/yolov3_trt_ros/src/homography_xycar168.npy')

        xmin = np.float32(bbox.xmin)
        xmax = np.float32(bbox.xmax)
        ymin = np.float32(bbox.ymin)
        ymax = np.float32(bbox.ymax)

        cx = xmin + (xmax - xmin) / 2

        center_point = (cx / 416.0 * 609.0, ymax / 416.0 * 394.0)

        object_centor_point = np.array([center_point[0], center_point[1], 1], dtype=np.float32)

        estimation = np.dot(homography, object_centor_point)
        x, y, z = estimation[0], estimation[1], estimation[2]

        x = x / z
        y = y / z
        z = z / z

        distance = math.sqrt(x ** 2 + y ** 2 + z ** 2)

        return x, y, z, distance

    def _write_message(self, detection_results, boxes, scores, classes):
        """ populate output message with input header and bounding boxes information """

        if boxes is None:
            return None
        for box, score, category in zip(boxes, scores, classes):
            # Populate darknet message
            minx, miny, width, height = box
            detection_msg = BoundingBox()
            # add Bounding Box info
            detection_msg.xmin = int(minx)
            detection_msg.xmax = int(minx + width)
            detection_msg.ymin = int(miny)
            detection_msg.ymax = int(miny + height)
            detection_msg.probability = score
            detection_msg.id = int(category)
            # add real distance and real location of object
            detection_msg.x, detection_msg.y, detection_msg.z, detection_msg.distance = self.get_location_and_distance(detection_msg)
            # add traffic signal color msg
            # detection_msg.color = self.rgb_detection(detection_msg)
            detection_results.bounding_boxes.append(detection_msg)
        return detection_results

    def publisher(self, boxes, confs, classes):
        """ Publishes to detector_msgs
        Parameters:
        boxes (List(List(int))) : Bounding boxes of all objects
        confs (List(double))	: Probability scores of all objects
        classes  (List(int))	: Class ID of all classes
        """
        detection_results = BoundingBoxes()
        self._write_message(detection_results, boxes, confs, classes)
        self.detection_pub.publish(detection_results)


# parse width, height, masks and anchors from cfg file
def parse_cfg_wh(cfg):
    masks = []
    with open(cfg, 'r') as f:
        lines = f.readlines()
        for line in lines:
            if 'width' in line:
                w = int(line[line.find('=') + 1:].replace('\n', ''))
            elif 'height' in line:
                h = int(line[line.find('=') + 1:].replace('\n', ''))
            elif 'anchors' in line:
                anchor = line.split('=')[1].replace('\n', '')
                anc = [int(a) for a in anchor.split(',')]
                anchors = [(anc[i * 2], anc[i * 2 + 1]) for i in range(len(anc) // 2)]
            elif 'mask' in line:
                mask = line.split('=')[1].replace('\n', '')
                m = tuple(int(a) for a in mask.split(','))
                masks.append(m)
    return w, h, masks, anchors


def img_callback(data):
    global xycar_image
    global bridge

    img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
    cv_image = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    # cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    calibrate_mtx = np.array([
        [361.447024, 0.0, 331.957667],
        [0.0, 360.079036, 244.636355],
        [0.0, 0.0, 1.0]
    ])

    dist = np.array([-0.322153, 0.082121, -0.001633, 0.003670, 0.0])
    image_height, image_width, _ = cv_image.shape

    cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(calibrate_mtx, dist,
                                                     (image_width, image_height), 1, (image_width, image_height))

    dst = cv2.undistort(cv_image, calibrate_mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    calibrated_image = dst[y: y + h, x: x + w]
    xycar_image = calibrated_image

    xycar_msg = data


def draw_bboxes(image_raw, bboxes, confidences, categories, all_categories, bbox_color='blue'):
    """Draw the bounding boxes on the original input image and return it.
    Keyword arguments:
    image_raw -- a raw PIL Image
    bboxes -- NumPy array containing the bounding box coordinates of N objects, with shape (N,4).
    categories -- NumPy array containing the corresponding category for each object,
    with shape (N,)
    confidences -- NumPy array containing the corresponding confidence for each object,
    with shape (N,)
    all_categories -- a list of all categories in the correct ordered (required for looking up
    the category name)
    bbox_color -- an optional string specifying the color of the bounding boxes (default: 'blue')
    """
    draw = ImageDraw.Draw(image_raw)
    if bboxes is None and confidences is None and categories is None:
        return image_raw
    for box, score, category in zip(bboxes, confidences, categories):
        x_coord, y_coord, width, height = box
        left = max(0, np.floor(x_coord + 0.5).astype(int))
        top = max(0, np.floor(y_coord + 0.5).astype(int))
        right = min(image_raw.width, np.floor(x_coord + width + 0.5).astype(int))
        bottom = min(image_raw.height, np.floor(y_coord + height + 0.5).astype(int))

        draw.rectangle(((left, top), (right, bottom)), outline=bbox_color)
        draw.text((left, top - 12), '{0} {1:.2f}'.format(all_categories[category], score), fill=bbox_color)

    return image_raw


def get_engine(engine_file_path=""):
    """Attempts to load a serialized engine if available, otherwise builds a new TensorRT engine and saves it."""
    if os.path.exists(engine_file_path):
        # If a serialized engine exists, use it instead of building an engine.
        print("Reading engine from file {}".format(engine_file_path))
        with open(engine_file_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
            return runtime.deserialize_cuda_engine(f.read())
    else:
        print("no trt model")
        sys.exit(1)


if __name__ == '__main__':
    yolo = yolov3_trt()
    rospy.init_node('yolov3_trt_ros', anonymous=True)
    yolo.detect()
