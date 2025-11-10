# from lib.hrnet.gen_kpts import gen_frame_kpts as hrnet_pose
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
# from point_msgs.msg import Points2DFrame


import argparse
import os
import platform
import sys
import time
import queue 
import torch
from tqdm import tqdm
import natsort
from detector.apis import get_detector
from trackers.tracker_api import Tracker
from trackers.tracker_cfg import cfg as tcfg
from trackers import track
from alphapose.models import builder
from alphapose.utils.config import update_config
from alphapose.utils.detector import DetectionLoader
from alphapose.utils.file_detector import FileDetectionLoader
from alphapose.utils.transforms import flip, flip_heatmap
from alphapose.utils.vis import getTime
from alphapose.utils.webcam_detector import WebCamDetectionLoader, RealsenseDetectionLoader
from alphapose.utils.writer import DataWriter_rs
from colorama import Fore






class proccess_2D(Node):
    def __init__(self):
        super().__init__('proccess_2D')
        self.get_logger().info('Logger Node has been started.')

        # ROS image converter
        self.bridge = CvBridge()
        
        # Publisher for 2D points
        self.publisher = None

        # Publisher for Frame
        self.image_publisher = None


        # Create a CvBridge to convert OpenCV images to ROS messages
        self.br = CvBridge()

       # Configure RealSense pipeline
        width , height, fps= 640, 480, 60

        self.alphapose()

    def alphapose_parser(self):
        parser = argparse.ArgumentParser(description='AlphaPose Demo')
        parser.add_argument('--cfg', type=str, default='/home/shayan/projects/AlphaPose/configs/halpe_coco_wholebody_136/resnet/256x192_res50_lr1e-3_2x-regression.yaml',
                            help='experiment configure file name')
        parser.add_argument('--checkpoint', type=str, default='/home/shayan/projects/AlphaPose/pretrained_models/multi_domain_fast50_regression_256x192.pth',
                            help='checkpoint file name')
        parser.add_argument('--sp', default=False, action='store_true',
                            help='Use single process for pytorch')
        parser.add_argument('--detector', dest='detector',
                            help='detector name', default="yolox-x")
        parser.add_argument('--detfile', dest='detfile',
                            help='detection result file', default="")
        parser.add_argument('--indir', dest='inputpath',
                            help='image-directory', default="")
        parser.add_argument('--list', dest='inputlist',
                            help='image-list', default="")
        parser.add_argument('--image', dest='inputimg',
                            help='image-name', default="")
        parser.add_argument('--outdir', dest='outputpath',
                            help='output-directory', default="./output")
        parser.add_argument('--save_img', default=False, action='store_true',
                            help='save result as image')
        parser.add_argument('--vis', default=False, action='store_true',
                            help='visualize image')
        parser.add_argument('--showbox', default=False, action='store_true',
                            help='visualize human bbox')
        parser.add_argument('--profile', default=False, action='store_true',
                            help='add speed profiling at screen output')
        parser.add_argument('--format', type=str,
                            help='save in the format of cmu or coco or openpose, option: coco/cmu/open')
        parser.add_argument('--min_box_area', type=int, default=0,
                            help='min box area to filter out')
        parser.add_argument('--detbatch', type=int, default=5,
                            help='detection batch size PER GPU')
        parser.add_argument('--posebatch', type=int, default=64,
                            help='pose estimation maximum batch size PER GPU')
        parser.add_argument('--eval', dest='eval', default=False, action='store_true',
                            help='save the result json as coco format, using image index(int) instead of image name(str)')
        parser.add_argument('--gpus', type=str, dest='gpus', default="0",
                            help='choose which cuda device to use by index and input comma to use multi gpus, e.g. 0,1,2,3. (input -1 for cpu only)')
        parser.add_argument('--qsize', type=int, dest='qsize', default=1024,
                            help='the length of result buffer, where reducing it will lower requirement of cpu memory')
        parser.add_argument('--flip', default=False, action='store_true',
                            help='enable flip testing')
        parser.add_argument('--debug', default=False, action='store_true',
                            help='print detail information')
        parser.add_argument('--device_name', type=str, default=None, nargs='+',
                            help='camera device name')
        parser.add_argument('--active_sides', type=int, nargs='+', default=[-1],
                            help='front = 0, right = 1, left = 2, to decide which of the sides should be activated.')
        # parser.add_argument('--camera_view', type=int, default=0, choices=[-1, 0, 1, 2],
        #                     help='the camera view from front, left, right with order of front = 0, right = 1, left = 2, general agle = -1')
        """----------------------------- Video options -----------------------------"""
        parser.add_argument('--save_video', dest='save_video',
                            help='whether to save rendered video', default=False, action='store_true')
        parser.add_argument('--vis_fast', dest='vis_fast',
                            help='use fast rendering', action='store_true', default=False)
        """----------------------------- Tracking options -----------------------------"""
        parser.add_argument('--pose_flow', dest='pose_flow',
                            help='track humans in video with PoseFlow', action='store_true', default=False)
        parser.add_argument('--pose_track', dest='pose_track',
                            help='track humans in video with reid', action='store_true', default=False)

        args = parser.parse_args()
        cfg = update_config(args.cfg)

        if platform.system() == 'Windows':
            args.sp = True

        args.gpus = [int(i) for i in args.gpus.split(',')] if torch.cuda.device_count() >= 1 else [-1]
        args.device = torch.device("cuda:" + str(args.gpus[0]) if args.gpus[0] >= 0 else "cpu")
        args.detbatch = args.detbatch * len(args.gpus)
        args.posebatch = args.posebatch * len(args.gpus)
        args.tracking = args.pose_track or args.pose_flow or args.detector=='tracker'
        return args, cfg



    def alphapose(self):
        args, cfg = self.alphapose_parser()
        print(args.device_name)
        print(args.active_sides)
        assert len(args.device_name) == len(args.active_sides), 'the number of device do not match the number of active sides for cameras.'

        if not args.sp:
            torch.multiprocessing.set_start_method('forkserver', force=True)
            torch.multiprocessing.set_sharing_strategy('file_system')

        if self.publisher is None:
            self.publisher = {}
            self.image_publisher = {}
            for side in args.active_sides:
                # from the input argumet it will be one of theese three side points
                if side == 1:
                    self.publisher['right'] = self.create_publisher(Float32MultiArray, 'right_points_2D', 10)
                    self.image_publisher['right'] = self.create_publisher(Image, 'right_frame_2D', 10)
                    print(Fore.GREEN + 'Right side detector is activateted.......')
                elif side == 0: 
                    self.publisher['front'] = self.create_publisher(Float32MultiArray, 'front_points_2D', 10)
                    self.image_publisher['front'] = self.create_publisher(Image, 'front_frame_2D', 10)
                    print(Fore.GREEN + 'Front side detector is activateted.......')
                elif side == 2:
                    self.publisher['left'] = self.create_publisher(Float32MultiArray, 'left_points_2D', 10)
                    self.image_publisher['left'] = self.create_publisher(Image, 'left_frame_2D', 10)
                    print(Fore.GREEN + 'Left side detector is activateted.......')
                elif side == -1:
                    self.publisher['general'] = self.create_publisher(Float32MultiArray, 'points_2D', 10)
                    self.image_publisher['general'] = self.create_publisher(Image, 'frame_2D', 10)
                    print(Fore.GREEN + 'Detector is activateted.......')

        

        # Load detection loader
        det_loader = {}
        det_worker = {}
        for idx in range(len(args.active_sides)):
            det_loader[args.active_sides[idx]] = RealsenseDetectionLoader(get_detector(args), cfg, args, device_name = args.device_name[idx])
            det_worker[args.active_sides[idx]] = det_loader[args.active_sides[idx]].start()

        # Load pose model
        pose_model = builder.build_sppe(cfg.MODEL, preset_cfg=cfg.DATA_PRESET)

        print('Loading pose model from %s...' % (args.checkpoint,))
        pose_model.load_state_dict(torch.load(args.checkpoint, map_location=args.device))
        pose_dataset = builder.retrieve_dataset(cfg.DATASET.TRAIN)
        if args.pose_track:
            tracker = Tracker(tcfg, args)
        if len(args.gpus) > 1:
            pose_model = torch.nn.DataParallel(pose_model, device_ids=args.gpus).to(args.device)
        else:
            pose_model.to(args.device)
        pose_model.eval()

        runtime_profile = {
            'dt': [],
            'pt': [],
            'pn': []
        }
        mode = 'webcam'

        queueSize = 2 if mode == 'webcam' else args.qsize

        writer = {}
        for side in args.active_sides:
            writer[side] = DataWriter_rs(cfg, args, save_video=False, queueSize=queueSize)
            writer[side].start()



        print('Starting webcam demo, press Ctrl + C to terminate...')
        sys.stdout.flush()

        batchSize = args.posebatch
        if args.flip:
            batchSize = int(batchSize / 2)
        try:
            idx = 0
            while True:
                with torch.no_grad():
                    for side in det_loader.keys():
                        (inps, orig_img, im_name, boxes, scores, ids, cropped_boxes) = det_loader[side].read()
                        if orig_img is None:
                            break
                        if boxes is None or boxes.nelement() == 0:
                            writer[side].save(None, None, None, None, None, orig_img, im_name)
                            continue
                        # Pose Estimation
                        inps = inps.to(args.device)
                        datalen = inps.size(0)
                        leftover = 0
                        if (datalen) % batchSize:
                            leftover = 1
                        num_batches = datalen // batchSize + leftover
                        hm = []
                        for j in range(num_batches):
                            inps_j = inps[j * batchSize:min((j + 1) * batchSize, datalen)]
                            if args.flip:
                                inps_j = torch.cat((inps_j, flip(inps_j)))
                            hm_j = pose_model(inps_j)
                            if args.flip:
                                hm_j_flip = flip_heatmap(hm_j[int(len(hm_j) / 2):], pose_dataset.joint_pairs, shift=True)
                                hm_j = (hm_j[0:int(len(hm_j) / 2)] + hm_j_flip) / 2
                            hm.append(hm_j)
                        hm = torch.cat(hm)
                        if args.pose_track:
                            boxes,scores,ids,hm,cropped_boxes = track(tracker,args,orig_img,inps,boxes,hm,cropped_boxes,im_name,scores)
                        hm = hm.cpu()
                        writer[side].save(boxes, scores, ids, hm, cropped_boxes, orig_img, im_name)

                        if writer[side].count_results() > 0:
                            try:
                                keypoints, vis_frame = writer[side].key_points.get_nowait()
                                # keypoints, vis_frame = writer.get_keypoints()
                                final_kpt = np.hstack((keypoints['keypoints'],keypoints['kp_score']))


                                # Create a ROS2 message
                                msg = Float32MultiArray()
                                msg.data = final_kpt.flatten().tolist()
                                # self.publisher.publish(msg)

                                # Convert OpenCV image (BGR) to ROS2 Image message
                                img_msg = self.br.cv2_to_imgmsg(vis_frame, encoding='bgr8')

                                if side == 1:
                                    self.publisher['right'].publish(msg) 
                                    self.image_publisher['right'].publish(img_msg) 
                                elif side == 0: 
                                    self.publisher['front'].publish(msg) 
                                    self.image_publisher['front'].publish(img_msg) 
                                elif side == 2:
                                    self.publisher['left'].publish(msg) 
                                    self.image_publisher['left'].publish(img_msg) 
                                elif side == -1:
                                    self.publisher['general'].publish(msg) 
                                    self.image_publisher['general'].publish(img_msg) 

                                # self.image_publisher.publish(img_msg)
                                # self.get_logger().info('{0} Body point is publishing ... '.format(idx))
                                idx += 1
                            except queue.Empty:
                                # Convert OpenCV image (BGR) to ROS2 Image message
                                img_msg = self.br.cv2_to_imgmsg(orig_img, encoding='bgr8')
                                # self.image_publisher.publish(img_msg)
                                if side == 1:
                                    self.image_publisher['right'].publish(img_msg) 
                                elif side == 0: 
                                    self.image_publisher['front'].publish(img_msg) 
                                elif side == 2:
                                    self.image_publisher['left'].publish(img_msg) 
                                elif side == -1:
                                    self.image_publisher['general'].publish(img_msg) 
                                # self.get_logger().info('{0} No body point ... '.format(idx))



        except Exception as e:
            print(repr(e))
            # print(e)
            print('An error as above occurs when processing the images, please check it')
            pass
        except KeyboardInterrupt:
                for side in det_loader.keys():
                    writer[side].stop()
                    det_loader[side].terminate()
                    writer[side].terminate()
                    writer[side].clear_queues()
                    det_loader[side].clear_queues()


def main(args=None):
    rclpy.init(args=args)
    node = proccess_2D()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()