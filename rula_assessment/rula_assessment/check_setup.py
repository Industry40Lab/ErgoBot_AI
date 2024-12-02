from rula_assessment.pose_landmarks import pose_landmark
from rula_assessment.realsense_class import RealSense_Cam
from ament_index_python.packages import get_package_share_directory
import cv2
import os
from rula_assessment.angle_calc import angle_calc, rula_assesment_class, owas_assesment_class

import warnings
warnings.filterwarnings("ignore")
package_share_dir = get_package_share_directory('rula_assessment')
config_file_path = os.path.join(package_share_dir, 'configs')

print(f"Config file path: {config_file_path}")


def main():


    cam = RealSense_Cam()

    pipeline, config = cam.start_real_sense()
    # Start streaming
    pipeline.start(config)

    PoseLandmarks = pose_landmark(task_path=config_file_path,task_type="pose_landmarker_lite.task")
    PoseLandmarks.create_landmarker()
    reula_class = rula_assesment_class()
    ows_class = owas_assesment_class()
    while True:

        depth, frame = cam.get_frame_from_realsense(pipeline,aligned_frame=False)

        
        # cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        # cv2.imshow('Align Example', frame)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

        PoseLandmarks.run_landmarker(frame_bgr=frame)
    
        
        if PoseLandmarks.pose_landmarks_frame() is not None:
            cv2.imshow("pose_detection",cv2.cvtColor(PoseLandmarks.pose_landmarks_frame(),cv2.COLOR_RGB2BGR))

            pose_xyz = PoseLandmarks.input_of_assessment()
            if len(pose_xyz) > 0:

                # ows=ows_class.run(pose_xyz)
                reula_class.run(pose_xyz,None)

                # print(rula)
                # print(rula,reba)
                # if (rula != "NULL") and (reba != "NULL"):
                #     if int(rula)>3:
                #         print("Rapid Upper Limb Assessment Score : "+rula+"Posture not proper in upper body")
                #         print("Posture not proper in upper body","Warning")
                #     else:
                #         print("Rapid Upper Limb Assessment Score : "+rula)
                #     if int(reba)>4:
                #         print("Rapid Entire Body Score : "+reba+"Posture not proper in your body")
                #         print("Posture not proper in your body","Warning")
                #     else:
                #         print("Rapid Entire Body Score : "+reba)
                # else:
                #     print("Posture Incorrect")
                # print(ows)
    
    pipeline.stop()


main()