#!/usr/bin/env python3

import cv2
from ultralytics import YOLO


class PersonDetector:
    def __init__(self):
        # Load the models
        #self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # or yolov5m, yolov5l, yolov5x, custom
        model_path = 'yolov8n-pose.pt'
        #ckpt = torch.load(model_path)
        #self.model.load_state_dict(ckpt['model'].state_dict(), strict=False)
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

    def get_landmark_names(self):
        return None

    def get_landmark_colors(self):
        return None

    def get_landmark_color_dict(self):
        return None

    def apply_to_image(self, rgb_image, draw_output=False,landmarks_to_detect=None):
        results = self.model(rgb_image)
        keypoints_data = results[0].keypoints.xy[0]

        if landmarks_to_detect is None:
            landmarks_to_detect = self.landmark_names
        
        bodies = []
        landmark_dict = {};

        for name in landmarks_to_detect:
            landmark_index = self.landmark_names.index(name)
            x, y = int(keypoints_data[landmark_index][0]), int(keypoints_data[landmark_index][1])
            landmark_dict[name] = (x, y)
        
        bodies.append({'box':None, 'ypr':None, 'landmarks':landmark_dict})


        if draw_output:
            if isinstance(rgb_image, str):
                rgb_image = cv2.imread(rgb_image)
            output_image = rgb_image.copy()
            keypoints_data = results[0].keypoints.xy[0]
            for i in range(len(keypoints_data)):
                x, y = int(keypoints_data[i][0]), int(keypoints_data[i][1])
                cv2.circle(output_image, (x, y), 5, (0, 255, 0), -1)
                cv2.putText(output_image, str(i)+"  "+self.landmark_names[i], (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.imwrite('saved_image.jpg', output_image)

        return bodies, output_image