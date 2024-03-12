#!/usr/bin/env python3

import cv2
import sys
import rclpy

import person_detector as pd
import detection_node as dn
#import deep_learning_model_options as do

import person_detector as pd


def main():
    print('cv2.__version__ =', cv2.__version__)
    print('Python version (must be > 3.0):', sys.version)
    assert(int(sys.version[0]) >= 3)
    
    detector = pd.PersonDetector()
    
    default_marker_name = 'body_landmarks'
    node_name = 'DetectBodyLandmarksNode'
    topic_base_name = 'body_landmarks'
    fit_plane = True
    node = dn.DetectionNode(detector, default_marker_name, node_name, topic_base_name, fit_plane)
    node.main()
    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print('interrupt received, so shutting down')

if __name__=="__main__":
    main()