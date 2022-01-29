#! /usr/bin/env python3


import roslib
import rospy
from std_msgs.msg import Header
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
IMAGE_WIDTH=640
IMAGE_HEIGHT=480

import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import argparse
import glob
import time
from pathlib import Path

import os
import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
import numpy as np
from models.experimental import attempt_load
from utils.datasets import letterbox
from utils.general import check_img_size, check_requirements, non_max_suppression_face, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized
from tqdm import tqdm

ros_image=0

def dynamic_resize(shape, stride=64):
    max_size = max(shape[0], shape[1])
    if max_size % stride != 0:
        max_size = (int(max_size / stride) + 1) * stride 
    return max_size

def scale_coords_landmarks(img1_shape, coords, img0_shape, ratio_pad=None):
    # Rescale coords (xyxy) from img1_shape to img0_shape
    if ratio_pad is None:  # calculate from img0_shape
        gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])  # gain  = old / new
        pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2  # wh padding
    else:
        gain = ratio_pad[0][0]
        pad = ratio_pad[1]

    coords[:, [0, 2, 4, 6, 8]] -= pad[0]  # x padding
    coords[:, [1, 3, 5, 7, 9]] -= pad[1]  # y padding
    coords[:, :10] /= gain
    #clip_coords(coords, img0_shape)
    coords[:, 0].clamp_(0, img0_shape[1])  # x1
    coords[:, 1].clamp_(0, img0_shape[0])  # y1
    coords[:, 2].clamp_(0, img0_shape[1])  # x2
    coords[:, 3].clamp_(0, img0_shape[0])  # y2
    coords[:, 4].clamp_(0, img0_shape[1])  # x3
    coords[:, 5].clamp_(0, img0_shape[0])  # y3
    coords[:, 6].clamp_(0, img0_shape[1])  # x4
    coords[:, 7].clamp_(0, img0_shape[0])  # y4
    coords[:, 8].clamp_(0, img0_shape[1])  # x5
    coords[:, 9].clamp_(0, img0_shape[0])  # y5
    return coords

def show_results(img, xywh, conf, landmarks, class_num):
    h,w,c = img.shape
    tl = 1 or round(0.002 * (h + w) / 2) + 1  # line/font thickness
    
    x1 = int(xywh[0] * w - 0.5 * xywh[2] * w)
    y1 = int(xywh[1] * h - 0.5 * xywh[3] * h)
    x2 = int(xywh[0] * w + 0.5 * xywh[2] * w)
    y2 = int(xywh[1] * h + 0.5 * xywh[3] * h)
    cv2.rectangle(img, (x1,y1), (x2, y2), (0,255,0), thickness=tl, lineType=cv2.LINE_AA)

    clors = [(255,0,0),(0,255,0),(0,0,255),(255,255,0),(0,255,255)]

    for i in range(5):
        point_x = int(landmarks[2 * i] * w)
        point_y = int(landmarks[2 * i + 1] * h)
        cv2.circle(img, (point_x, point_y), tl+1, clors[i], -1)

    tf = max(tl - 1, 1)  # font thickness
    label = str(int(class_num)) + ': ' + str(conf)[:5]
    cv2.putText(img, label, (x1, y1 - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)
    return img

# def detect(model, img0):
def detect(img0, imgsz):

    time1 = time.time()

    global ros_image

    stride = int(model.stride.max())  # model stride
    # imgsz = opt.img_size
    if imgsz <= 0:                    # original size    
        imgsz = dynamic_resize(img0.shape)
    imgsz = check_img_size(imgsz, s=64)  # check img_size
    img = letterbox(img0, imgsz)[0]
    # Convert
    img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
    img = np.ascontiguousarray(img)
    img = torch.from_numpy(img).to(device)
    img = img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    # Inference
    pred = model(img)[0]
    # Apply NMS
    pred = non_max_suppression_face(pred, 0.02, 0.5)[0]
    gn = torch.tensor(img0.shape)[[1, 0, 1, 0]].to(device)  # normalization gain whwh
    gn_lks = torch.tensor(img0.shape)[[1, 0, 1, 0, 1, 0, 1, 0, 1, 0]].to(device)  # normalization gain landmarks
    boxes = []
    landmarks5 = []
    faces = [] 
    """
    if pred[0] is None:
        boxes = [0] * 5
        landmarks5 = [0] * 10
    else:
        boxes = [len(pred[0])]
        landmarks5 = [len(pred[0])]
    """
    h, w, c = img0.shape
    if pred is not None:
        pred[:, :4] = scale_coords(img.shape[2:], pred[:, :4], img0.shape).round()
        pred[:, 5:15] = scale_coords_landmarks(img.shape[2:], pred[:, 5:15], img0.shape).round()
        for j in range(pred.size()[0]):
            xywh = (xyxy2xywh(pred[j, :4].view(1, 4)) / gn).view(-1)
            xywh = xywh.data.cpu().numpy()
            conf = pred[j, 4].cpu().numpy()
            landmarks = (pred[j, 5:15].view(1, 10) / gn_lks).view(-1).tolist()
            landmarks_org = (pred[j, 5:15].view(1, 10)).view(-1).tolist()
            class_num = pred[j, 15].cpu().numpy()
            x1 = int(xywh[0] * w - 0.5 * xywh[2] * w)
            y1 = int(xywh[1] * h - 0.5 * xywh[3] * h)
            x2 = int(xywh[0] * w + 0.5 * xywh[2] * w)
            y2 = int(xywh[1] * h + 0.5 * xywh[3] * h)
            # boxes.append([x1, y1, x2-x1, y2-y1, conf])
            boxes += [x1, y1, x2-x1, y2-y1, conf]
            # landmarks5.append(landmarks_org)    
            landmarks5 += landmarks_org
            face_list = [x1, y1, x2-x1, y2-y1, conf]
            for k in range(len(landmarks_org)):
               face_list.append(landmarks_org[k])
            faces += face_list 
            print(f'boxe:{[x1, y1, x2-x1, y2-y1, conf]}') 
            print(f'landmark:{landmarks_org}')
            print(f'face:{faces}')
            if (conf >= 0.2): 
               show_results(img0, xywh, conf, landmarks, 0)

    time4 = time.time()
    
    #print('2-1', time2 - time1)
    #print('3-2', time3 - time2)
    #print('4-3', time4 - time3)
    print('total processing time',time4-time1)
    print('************')
    ros_image = img0[:, :, [2, 1, 0]]
    #cv2.imshow('YOLOV5', im0)
    #a = cv2.waitKey(1)
    #### Create CompressedIamge ####
    publish_image(ros_image)
    """
    pub_array = Float32MultiArray(data=boxes)
    box_pub.publish(pub_array)
    pub_array = Float32MultiArray(data=landmarks5)
    land_pub.publish(pub_array)
    """
    pub_array = Float32MultiArray(data=faces)
    face_pub.publish(pub_array)

def image_callback_1(image):
    global ros_image
    #ros_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
    ros_image = np.fromstring(image.data, np.uint8)
    ros_image = cv2.imdecode(ros_image, cv2.IMREAD_COLOR)
    with torch.no_grad():
        detect(ros_image,imgsz)

def publish_image(imgdata):
    image_temp=Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = 'map'
    image_temp.height=IMAGE_HEIGHT
    image_temp.width=IMAGE_WIDTH
    image_temp.encoding='rgb8'
    image_temp.data=np.array(imgdata).tostring()
    #print(imgdata)
    #image_temp.is_bigendian=True
    image_temp.header=header
    image_temp.step=IMAGE_WIDTH*3
    image_pub.publish(image_temp)

if __name__ == '__main__':
    imgsz = 640
    # Load model
    device = select_device('0')
    # half = device.type != 'cpu'  # half precision only supported on CUDA
    model = attempt_load('yolov5s-face.pt', map_location=device)  # load FP32 model
    imgsz = check_img_size(imgsz, s=model.stride.max())  # check img_size
    # if half:
        # model.half()  # to FP16

    rospy.init_node('ros_face')

    image_topic_1 = 'camera0/compressed' #"/usb_cam/image_raw"
    rospy.Subscriber(image_topic_1, CompressedImage, image_callback_1, queue_size=1, buff_size=52428800)
    image_pub = rospy.Publisher('face_image', Image, queue_size=1)
    # box_pub = rospy.Publisher('face_box', Float32MultiArray, queue_size=1)
    # land_pub = rospy.Publisher('face_land', Float32MultiArray, queue_size=1)
    face_pub = rospy.Publisher('face_result', Float32MultiArray, queue_size=1)
    #rospy.init_node("yolo_result_out_node", anonymous=True)
    

    rospy.spin()
