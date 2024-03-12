#!/usr/bin/env python3

import cv2
import rclpy
from ultralytics import YOLO


class PersonDetector:
    def __init__(self):
        # Load the models
        model_path = 'yolov8n-pose.pt'
        self.model = YOLO(model_path)
        self.landmark_names = ["nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder", 
             "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist", 
             "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle"]
        
        # rgba
        self.landmark_color_dict = {'nose': (1.0, 1.0, 1.0, 1.0),
                                    'left_eye': (1.0, 0.0, 0.0, 1.0),
                                    'right_eye': (0.0, 1.0, 0.0, 1.0),
                                    'left_ear': (0.0, 0.0, 1.0, 1.0),
                                    'right_ear': (0.0, 0.0, 1.0, 1.0),
                                    'left_shoulder': (0.0, 1.0, 0.0, 1.0),
                                    'right_shoulder': (0.0, 0.0, 1.0, 1.0),
                                    'left_elbow': (0.0, 1.0, 1.0, 1.0),
                                    'right_elbow': (0.0, 1.0, 1.0, 1.0),
                                    'left_wrist': (0.0, 0.0, 1.0, 1.0),
                                    'right_wrist': (0.0, 1.0, 1.0, 1.0),
                                    'left_hip': (0.0, 1.0, 1.0, 1.0),
                                    'right_hip': (0.0, 0.0, 1.0, 1.0),
                                    'left_knee': (1.0, 0.0, 1.0, 1.0),
                                    'right_knee': (1.0, 0.0, 1.0, 1.0),
                                    'left_ankle': (0.0, 1.0, 1.0, 1.0),
                                    'right_ankle': (0.0, 1.0, 1.0, 1.0)}

        self.landmark_colors = [self.landmark_color_dict[n] for n in self.landmark_names]

        self.num_landmarks = len(self.landmark_names)
        #print('self.num_landmarks =', self.num_landmarks)
        #print('len(self.landmark_colors =', len(self.landmark_colors))
        self.logger = rclpy.logging.get_logger('stretch_deep_perception')

    def get_landmark_names(self):
        return None

    def get_landmark_colors(self):
        return None

    def get_landmark_color_dict(self):
        return self.landmark_color_dict

    def apply_to_image(self, rgb_image, draw_output=False,landmarks_to_detect=None):
        results = self.model(rgb_image)
        keypoints_data = results[0].keypoints.xy[0]


        if not keypoints_data.numel():
            return [({'box':None, 'ypr':None, 'landmarks':{'nose':(0,0),'left_eye':(0,0),'right_eye':(0,0),'left_ear':(0,0),'right_ear':(0,0),
                                                       'left_shoulder':(0,0),'right_shoulder':(0,0),'left_elbow':(0,0),'right_elbow':(0,0),
                                                       'left_wrist':(0,0),'right_wrist':(0,0),'left_hip':(0,0),'right_hip':(0,0),'left_knee':(0,0),
                                                       'right_knee':(0,0),'left_ankle':(0,0),'right_ankle':(0,0),}})], None

        if landmarks_to_detect is None:
            landmarks_to_detect = self.landmark_names
        
        bodies = []
        landmark_dict = {}

        #self.logger.info(f'keypoints_data = {keypoints_data}')

        for name in landmarks_to_detect:
            landmark_index = self.landmark_names.index(name)
            x, y = int(keypoints_data[landmark_index][0]), int(keypoints_data[landmark_index][1])
            landmark_dict[name] = (x, y)
        
        bodies.append({'box':None, 'ypr':None, 'landmarks':landmark_dict})
        output_image = None

        if draw_output:
            if isinstance(rgb_image, str):
                rgb_image = cv2.imread(rgb_image)
            output_image = rgb_image.copy()
            for i in range(len(keypoints_data)):
                x, y = int(keypoints_data[i][0]), int(keypoints_data[0][1])
                cv2.circle(output_image, (x, y), 5, (0, 255, 0), -1)
                cv2.putText(output_image, str(i)+"  "+self.landmark_names[i], (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.imwrite('saved_image.jpg', output_image)

        return bodies, output_image
    
        # return [({'box':None, 'ypr':None, 'landmarks':{'nose':(100,200),'left_eye':(200,200),'right_eye':(300,200),'left_ear':(400,200),'right_ear':(500,200),
        #                                                'left_shoulder':(100,300),'right_shoulder':(100,400),'left_elbow':(100,500),'right_elbow':(100,600),
        #                                                'left_wrist':(200,300),'right_wrist':(200,400),'left_hip':(200,500),'right_hip':(200,600),'left_knee':(200,600),
        #                                                'right_knee':(300,500),'left_ankle':(300,600),'right_ankle':(300,700),}})],output_image