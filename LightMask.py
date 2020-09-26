#!/usr/bin/env python
# -*- coding:utf-8 -*-
import sys, os
sys.path.insert(1, os.path.realpath(os.path.pardir))

import cv2
import time

import argparse
import numpy as np
from PIL import Image
# Ros libraries
import roslib
import rospy
# Ros Messages
from sensor_msgs.msg import CompressedImage
from face_mask.msg import BoundingBox, MaskFrame
from entity_tracker.msg import AddEntityRequestMsg, EntitiesFrameMsg, TrackEntityMsg
#


class LightMaskDetector:
    def __init__(self):
        kQueueSize = 15
        self.image_sub = rospy.Subscriber("/", CompressedImage, self._callback_image, queue_size=kQueueSize)
        self.mask_sub = rospy.Subscriber("/", MaskFrame, self._callback_mask, queue_size=kQueueSize)
        self.track_sub = rospy.Subscriber("/", EntitiesFrameMsg, self._callback_tracker, queue_size=kQueueSize) 

        self.add_entry_pub = rospy.Publisher("/", AddEntityRequestMsg)
        self.detect_pub = rospy.Publisher("/", CompressedImage)
        self.track_pub = rospy.Publisher("/", CompressedImage)
        self.out_pub = rospy.Publisher("/", CompressedImage)

        self.shouldInitialize = True 
        self.key_frame_msg = None
        self.counter = 1

    def _callback_image(self, ros_data):
        if self.shouldInitialize:
            self.key_frame_msg = ros_data
            self.detect_pub.publish(ros_data)
        else:
            self.track_pub.publish(ros_data)

    def _callback_mask(self, ros_data):
        if self.shouldInitialize:
            for bbox in ros_data.bboxes:
                request_msg = AddEntityRequestMsg()
                request_msg.img = self.key_frame_msg 
                request_msg.label = "Person " + str(self.counter)
                self.counter += 1
                request_msg.roi = bbox.bbox
                self.add_entry_pub.publish("request_msg")
            self.shouldInitialize = False

    def _callback_tracker(self, ros_data):
        color = (125,125,0)
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        for entity in data.entities:
            xmin, ymin, xmax, ymax = entity.roi
            cv2.rectangle(image, (xmin, ymin), (xmax, ymax), color, 2)
            cv2.putText(image, "%s" % (entity.label), (xmin + 2, ymin - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, color)
        out_msg = CompressedImage()
        out_msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        self.out_pub.publish(out_msg)
 
        
if __name__ == "__main__":
    detector = MaskDetector()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS  ght mask detector module")




