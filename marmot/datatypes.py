#!/usr/bin/env python3

import numpy as np
from rclpy.time import Time

class Detection():
    def __init__(self, dets_msg, det_msg):
        # Admin
        self.timestamp = Time.from_msg(dets_msg.header.stamp)
       
        # Spatial properties
        self.pose = det_msg.pose
        self.pos = np.array([[det_msg.pose.position.x], [det_msg.pose.position.y], [det_msg.pose.position.z]])
        self.size = np.array([[det_msg.bbox.size.x], [det_msg.bbox.size.y], [det_msg.bbox.size.z]])

        # Semantic Properties
        self.metadata = det_msg.metadata
        self.class_string = det_msg.class_string
        self.class_conf = det_msg.class_confidence

class Track():
    def __init__(self, tracker):
        self.det = tracker.dets.pop()
        self.trk_id = tracker.trk_id_count

        self.class_str = self.det.class_string
        self.n_matches = 1
        self.n_missed_det = 0
        tracker.get_logger().info("Created track from %s" % self.det)