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
        topics = {}
        try:
            node_name = rospy.get_name()
            topics['image_sub'] = rospy.get_param(rospy.search_param("camera_image_topic"))
            topics['mask_sub'] = rospy.get_param(rospy.search_param("detector_out_topic"))
            topics['track_sub'] = rospy.get_param(rospy.search_param("tracker_out_topic"))
            topics['add_entry_pub'] = rospy.get_param(rospy.search_param("add_entity_topic"))
            topics['detect_pub'] = rospy.get_param(rospy.search_param("detector_input_topic"))
            topics['track_pub'] = rospy.get_param(rospy.search_param("tracker_input_topic"))
            topics['out_pub'] = rospy.get_param(rospy.search_param("image_out_topic"))
        except rospy.ROSException:
            print("could not get param name")

        self.image_sub = rospy.Subscriber(topics['image_sub'], CompressedImage, self._callback_image, queue_size=kQueueSize)
        self.mask_sub = rospy.Subscriber(topics['mask_sub'], MaskFrame, self._callback_mask, queue_size=kQueueSize)
        self.track_sub = rospy.Subscriber(topics['track_sub'], EntitiesFrameMsg, self._callback_tracker, queue_size=kQueueSize) 

        self.add_entry_pub = rospy.Publisher(topics['add_entry_pub'], AddEntityRequestMsg, queue_size=kQueueSize)
        self.detect_pub = rospy.Publisher(topics['detect_pub'], CompressedImage, queue_size=kQueueSize)
        self.track_pub = rospy.Publisher(topics['track_pub'], CompressedImage, queue_size=kQueueSize)
        self.out_pub = rospy.Publisher(topics['out_pub'], CompressedImage, queue_size=kQueueSize)

        self.shouldInitialize = True 
        self.isInitialized = False
        self.key_frame_msg = None
        self.counter = 1

    def _callback_image(self, ros_data):
        if self.shouldInitialize:
            rospy.loginfo("Initializing Tracker!")
            self.key_frame_msg = ros_data
            self.detect_pub.publish(ros_data)
            self.shouldInitialize = False
        elif self.isInitialized:
            rospy.loginfo("Updating track bboxes")
            self.track_pub.publish(ros_data)

    def _callback_mask(self, ros_data):
        rospy.loginfo("Received Detected Masks...")
        for bbox in ros_data.bboxes:
            request_msg = AddEntityRequestMsg()
            request_msg.img = self.key_frame_msg 
            request_msg.label = "Person " + str(self.counter)
            self.counter += 1
            xmin, ymin, xmax, ymax = bbox.bbox
            x = xmin
            y = ymin
            w = xmax - xmin
            h = ymax - ymin
            request_msg.roi = (x, y, w, h)
            self.add_entry_pub.publish(request_msg)
        self.shouldInitialize = False
        self.isInitialized = True

    def _callback_tracker(self, ros_data):
        rospy.loginfo('received tracked data!')
        color = (125,125,0)
        np_arr = np.fromstring(ros_data.img.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        for entity in ros_data.entities:
            #xmin, ymin, xmax, ymax = entity.roi
            x, y, w, h = [int(x) for x in entity.roi]
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            cv2.putText(image, "%s" % (entity.label), (x + 2, y - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, color)
        out_msg = CompressedImage()
        out_msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        self.out_pub.publish(out_msg)
 
        
if __name__ == "__main__":
    rospy.init_node('light_mask_detector', anonymous=True)
    detector = LightMaskDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS  ght mask detector module")




