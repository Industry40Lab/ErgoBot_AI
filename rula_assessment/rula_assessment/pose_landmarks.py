
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import os
import logging

script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
parent_parent_dir = os.path.dirname(parent_dir)

folder_path = os.path.join(parent_dir, "configs")


class pose_landmark():
    def __init__(self,Num_persons=1,task_path = folder_path,task_type= "pose_landmarker_heavy.task"):
        self.num_persons = Num_persons
        self.BaseOptions = mp.tasks.BaseOptions
        self.PoseLandmarker = mp.tasks.vision.PoseLandmarker

        self.PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
        self.PoseLandmarkerResult = mp.tasks.vision.PoseLandmarkerResult
        self.VisionRunningMode = mp.tasks.vision.RunningMode

        self.model_path = os.path.join(task_path,task_type)
        self.options = self.PoseLandmarkerOptions(
            base_options=self.BaseOptions(model_asset_path=self.model_path),
            running_mode=self.VisionRunningMode.LIVE_STREAM,
            num_poses = self.num_persons,
            result_callback=self.print_result)
        
        self.landmarker = None
        self.counter_mp = 0
        self.result_pose_detection = None
        self.annotated_frame_pose = None


    def pose_landmarks_frame(self):
        return self.annotated_frame_pose

    def pose_landmarks_frame_flip(self):
        return cv2.flip(self.annotated_frame_pose,1)
    
    def pose_landmarks_coordinate(self):
        return self.result_pose_detection
    
    def print_result(self,result, output_image: mp.Image, timestamp_ms: int):
        # print('pose landmarker result: {}'.format(result))
        
        self.annotated_frame_pose = self.draw_landmarks_on_image(output_image.numpy_view(), result)
        self.result_pose_detection = result
        
    def create_landmarker(self):
        self.landmarker = self.PoseLandmarker.create_from_options(self.options)


    def run_landmarker(self, frame_bgr):
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        frame_rgb_flip = cv2.flip(frame_rgb,1)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb_flip)
        self.counter_mp +=1

        self.landmarker.detect_async(mp_image, self.counter_mp)


    def draw_landmarks_on_image(self,rgb_image, detection_result):
        pose_landmarks_list = detection_result.pose_landmarks
        annotated_image = np.copy(rgb_image)

        # Loop through the detected hands to visualize.
        for idx in range(len(pose_landmarks_list)):
            pose_landmarks = pose_landmarks_list[idx]

            # Draw the hand landmarks.
            pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            pose_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in pose_landmarks
            ])
            solutions.drawing_utils.draw_landmarks(
            annotated_image,
            pose_landmarks_proto,
            solutions.pose.POSE_CONNECTIONS,
            solutions.drawing_styles.get_default_pose_landmarks_style())
            

        return annotated_image
    
    def input_of_assessment(self):
       pose1 = []
       if(len(self.result_pose_detection.pose_landmarks) > 0):
        for id, lm in enumerate(self.result_pose_detection.pose_landmarks[0]):
             # print(lm)
             x_y_z=[]
             x_y_z.append(lm.x)
             x_y_z.append(lm.y)
             x_y_z.append(lm.z)
             x_y_z.append(lm.visibility)
             pose1.append(x_y_z)

       return pose1

if __name__ == "__main__":

    ss = pose_landmark()
    cap = cv2.VideoCapture(0)
    ss.create_landmarker()

    while cap.isOpened():
        ret,frame = cap.read()

        if cv2.waitKey(1) & 0xFF == ord('q'):
         break

        ss.run_landmarker(frame_bgr=frame)
    
        
        if ss.pose_landmarks_frame() is not None:
         cv2.imshow("hand_detection",cv2.cvtColor(ss.pose_landmarks_frame(),cv2.COLOR_RGB2BGR))
        else:
         cv2.imshow("hand_detection",frame)
        
       
    cap.release()
    cv2.destroyAllWindows()