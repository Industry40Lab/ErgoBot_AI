import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float32MultiArray, String, Int16
import copy
import argparse
import math
import queue
import threading
import time
from body_data.msg import BodyMsg


# Table A
tableA_in = np.array([
    [1, 2, 2, 2, 2, 3, 3, 3],
    [2, 2, 2, 2, 3, 3, 3, 3],
    [2, 3, 3, 3, 3, 3, 4, 4],
    [2, 3, 3, 3, 3, 4, 4, 4],
    [3, 3, 3, 3, 3, 4, 4, 4],
    [3, 3, 4, 4, 4, 4, 5, 5],
    [3, 3, 4, 4, 4, 4, 5, 5],
    [3, 4, 4, 4, 4, 4, 5, 5],
    [4, 4, 4, 4, 4, 5, 5, 5],
    [4, 4, 4, 4, 4, 5, 5, 5],
    [4, 4, 4, 5, 5, 5, 6, 6],
    [4, 4, 4, 5, 5, 5, 6, 6],
    [5, 5, 5, 5, 5, 6, 6, 7],
    [5, 6, 6, 6, 6, 7, 7, 7],
    [6, 6, 6, 7, 7, 7, 7, 8],
    [7, 7, 7, 7, 7, 8, 8, 8],
    [8, 8, 8, 8, 8, 9, 9, 9],
    [9, 9, 9, 9, 9, 9, 9, 9]
])

# Table B
tableB_in = np.array([
    [1, 3, 2, 3, 3, 4, 5, 5, 6, 6, 7, 7],
    [2, 3, 2, 3, 4, 5, 5, 5, 6, 7, 7, 7],
    [3, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7],
    [5, 5, 5, 6, 6, 7, 7, 7, 7, 7, 8, 8],
    [7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8],
    [8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9]
])

# Table C
tableC_in = np.array([
    [1, 2, 3, 3, 4, 5, 5],
    [2, 2, 3, 4, 4, 5, 5],
    [3, 3, 3, 4, 4, 5, 6],
    [3, 3, 3, 4, 5, 6, 6],
    [4, 4, 4, 5, 6, 7, 7],
    [4, 4, 5, 6, 6, 7, 7],
    [5, 5, 6, 6, 7, 7, 7],
    [5, 5, 6, 7, 7, 7, 7]
])


# body part mapping on the 17 keypoints
body_part = {
    "pelvis": 0,
    "rhip": 1,
    "rknee": 2,
    "rankle": 3,
    "lhip": 4,
    "lknee": 5,
    "lankle": 6,
    "back": 7,
    "neck": 8,
    "nose": 9,
    "head": 10,
    "lshoulder": 11,
    "lelbow": 12,
    "lwrist": 13,
    "rshoulder": 14,
    "relbow": 15,
    "rwrist": 16
}

# scaling the body points coordinate to between 0 and 1 base on the resulotion
def crop_scale(motion, scale_range=[1, 1]):
    '''
        Motion: [(M), T, 17, 3].
        Normalize to [-1, 1]
    '''
    result = copy.deepcopy(motion)
    valid_coords = motion[motion[..., 2]!=0][:,:2]
    if len(valid_coords) < 4:
        return np.zeros(motion.shape)
    xmin = min(valid_coords[:,0])
    xmax = max(valid_coords[:,0])
    ymin = min(valid_coords[:,1])
    ymax = max(valid_coords[:,1])
    ratio = np.random.uniform(low=scale_range[0], high=scale_range[1], size=1)[0]
    scale = max(xmax-xmin, ymax-ymin) * ratio
    if scale==0:
        return np.zeros(motion.shape)
    xs = (xmin+xmax-scale) / 2
    ys = (ymin+ymax-scale) / 2
    result[...,:2] = (motion[..., :2]- [xs,ys]) / scale
    result[...,:2] = (result[..., :2] - 0.5) * 2
    result = np.clip(result, -1, 1)
    return result


# converting the halpe keypoints to 17 standard h3.6 order for my ease since I code that before
def halpe2h36m(x):
    '''
        Input: x (T x V x C)  
       //Halpe 26 body keypoints
    {0,  "Nose"},
    {1,  "LEye"},
    {2,  "REye"},
    {3,  "LEar"},
    {4,  "REar"},
    {5,  "LShoulder"},
    {6,  "RShoulder"},
    {7,  "LElbow"},
    {8,  "RElbow"},
    {9,  "LWrist"},
    {10, "RWrist"},
    {11, "LHip"},
    {12, "RHip"},
    {13, "LKnee"},
    {14, "Rknee"},
    {15, "LAnkle"},
    {16, "RAnkle"},
    {17,  "Head"},
    {18,  "Neck"},
    {19,  "Hip"},
    {20, "LBigToe"},
    {21, "RBigToe"},
    {22, "LSmallToe"},
    {23, "RSmallToe"},
    {24, "LHeel"},
    {25, "RHeel"},
    '''
    T, V, C = x.shape
    y = np.zeros([T,17,C])
    y[:,0,:] = x[:,19,:]
    y[:,1,:] = x[:,12,:]
    y[:,2,:] = x[:,14,:]
    y[:,3,:] = x[:,16,:]
    y[:,4,:] = x[:,11,:]
    y[:,5,:] = x[:,13,:]
    y[:,6,:] = x[:,15,:]
    y[:,7,:] = (x[:,18,:] + x[:,19,:]) * 0.5
    y[:,8,:] = x[:,18,:]
    y[:,9,:] = x[:,0,:]
    y[:,10,:] = x[:,17,:]
    y[:,11,:] = x[:,5,:]
    y[:,12,:] = x[:,7,:]
    y[:,13,:] = x[:,9,:]
    y[:,14,:] = x[:,6,:]
    y[:,15,:] = x[:,8,:]
    y[:,16,:] = x[:,10,:]
    return y
    
# angle calculator
def points2angle(a, b, c):
    # Calculate the vectors AB and BC
    AB = b - a
    BC = b - c 

    # Calculate the dot product and magnitudes of AB and BC
    dot_product = np.dot(AB, BC)
    magnitude_AB = np.linalg.norm(AB)
    magnitude_BC = np.linalg.norm(BC)

    # Calculate the cosine of the angle
    cosine_angle = dot_product / (magnitude_AB * magnitude_BC)

    # Calculate the angle in radians and convert to degrees
    angle_rad = np.arccos(cosine_angle)
    angle_deg = np.degrees(angle_rad)
    return 180 - angle_deg


# Vectorized form of the formula point from line distance
def point_to_line_distance(A, B, P):
    num = abs((B[0] - A[0])*(A[1] - P[1]) - (A[0] - P[0])*(B[1] - A[1]))
    denom = np.linalg.norm(B - A)
    return num / denom

# converting and standrdization base on my previous work to get the 17 keypoints of on hundered something
class Point_Transformer():
    def __init__(self, vid_size=None, scale_range=None):
        self.vid_size = vid_size
        self.scale_range = scale_range
        self.idx = 0

    def transform(self,point):
        point = np.array(point).reshape(1, -1, 3)
        point = halpe2h36m(point)
        if self.vid_size:
            w, h = self.vid_size
            scale = min(w,h) / 2.0
            point[:,:,:2] = point[:,:,:2] - np.array([w, h]) / 2.0
            point[:,:,:2] = point[:,:,:2] / scale
            motion = point
        if self.scale_range:
            motion = crop_scale(point, self.scale_range)
        return motion.astype(np.float32).reshape(-1,3)

class rula_calculator(Node):
    def __init__(self):
        super().__init__('back_data')
        self.get_logger().info('Logger Node has been started.')
        self.kpts_keeper = None
        self.vid_size = (640,480)
        self.args = self.rula_arg_parser()
        self.r_side_reciever = self.create_subscription(Float32MultiArray, 'right_points_2D', self.right_point2D_reciever, 1)
        self.l_side_reciever = self.create_subscription(Float32MultiArray, 'left_points_2D', self.left_point2D_reciever, 1)
        self.front_reciever = self.create_subscription(Float32MultiArray, 'front_points_2D', self.front_point2D_reciever, 1)
        

        self.publishers_front = self.create_publisher(String, 'front_values', 10)
        self.publishers_right = self.create_publisher(String, 'right_values', 10)
        self.publishers_left = self.create_publisher(String, 'left_values', 10)


        self.publisher_score_right = self.create_publisher(Int16, 'right_rula_score', 10)
        self.publisher_score_left = self.create_publisher(Int16, 'left_rula_score', 10)

        self.publisher_full_body_data = self.create_publisher(BodyMsg, 'full_body_data', 10)


        self.front_results = queue.Queue(maxsize=10)
        self.r_side_results = queue.Queue(maxsize=10)
        self.l_side_results = queue.Queue(maxsize=10)

        self.score_thread = threading.Thread(target=self.publishing_thread)
        self.score_thread.start()


    
    def publishing_thread(self):
        while True: 
            front_value = None
            try:
                # Try to get from either queue with timeout
                front_value = self.front_results.get_nowait()
            except queue.Empty:
                continue  # Nothing to do

            # Now check queue2 and queue3 (non-blocking)
            left_value = None 
            right_value = None
            try:
                right_value = self.r_side_results.get_nowait()
            except queue.Empty:
                pass

            try:
                left_value = self.l_side_results.get_nowait()
            except queue.Empty:
                pass

            if left_value is not None and right_value is not None:
                self.rula_score('both', front=front_value, left=left_value,right=right_value)
            elif left_value is not None:
                self.rula_score('left', front=front_value, left=left_value)
            elif right_value is not None:
                self.rula_score('right', front=front_value, right=right_value)
                
            
            # time.sleep(0.01)  # Small sleep to avoid tight loop



    def rula_arg_parser(self):
        parser = argparse.ArgumentParser(description='Rula calculator additional input')
        parser.add_argument('--arm_support', type=int, default=0, choices=[0, -1],
                            help='if arm supported -1 else 0')
        parser.add_argument('--muscle_use', type=int, default=0, choices=[0, 1],
                            help='fill it base on the muscle score of table between 0 and 1')
        parser.add_argument('--load_score', type=int, default=0, choices=[0, 1, 2, 3],
                            help='fill it base on the carring load of table between 0, 1, 2, 3')
        parser.add_argument('--leg_support', type=int, default=2, choices=[1, 2],
                            help='fill it base on the leg support score 1, 2')
        parser.add_argument('--score_show', default=False, action='store_true',
                            help='showing score instead of angles if it is true')
        parser.add_argument('--vis', default=True, action='store_true',
                            help='showing values')

        # args = parser.parse_args()
        args, unknown = parser.parse_known_args()

        return args

    #################################### right side values and angles ###################################3

    def right_point2D_reciever(self, msg):
        points = np.array(msg.data).reshape(-1,3)

        if self.kpts_keeper is None:
            self.kpts_keeper = Point_Transformer(vid_size=self.vid_size, scale_range=[1,1])
        
        points = self.kpts_keeper.transform(points)
        shoulder_base = np.array([points[body_part["rshoulder"]][0], points[body_part["rshoulder"]][1] - 0.5])
        up_hand_angle =  points2angle(points[body_part["relbow"]][:2], points[body_part["rshoulder"]][:2], shoulder_base)
        low_hand_angle = points2angle(points[body_part["rshoulder"]][:2], points[body_part["relbow"]][:2], points[body_part["rwrist"]][:2])
        head_angle = max(points2angle(points[body_part["head"]][:2], points[body_part["neck"]][:2], points[body_part["back"]][:2]) - 15, 0)
        trunk_base= np.array([points[body_part["pelvis"]][0], points[body_part["pelvis"]][1] - 0.5])
        trunk = points2angle(points[body_part["neck"]][:2], points[body_part["pelvis"]][:2], trunk_base)
        if math.isnan(up_hand_angle):
            up_hand_angle = 0

        if math.isnan(low_hand_angle):
            low_hand_angle = 0

        if math.isnan(head_angle):
            head_angle = 0

        if math.isnan(trunk):
            trunk = 0

        try:
            self.r_side_results.put(np.array([up_hand_angle, low_hand_angle, head_angle, 180 - trunk]), block=False)
        except queue.Full:
            # print("Queue is full, right side skipped.")
            pass






    #################################### left side values and angles ###################################3
    
    def left_point2D_reciever(self, msg):
        points = np.array(msg.data).reshape(-1,3)

        # transform the 138 keypoint to 17 useful keypoints according to the 3.6M order
        if self.kpts_keeper is None:
            self.kpts_keeper = Point_Transformer(vid_size=self.vid_size, scale_range=[1,1])
        
        points = self.kpts_keeper.transform(points)
        shoulder_base = np.array([points[body_part["lshoulder"]][0], points[body_part["lshoulder"]][1] - 0.5])
        up_hand_angle =  points2angle(points[body_part["lelbow"]][:2], points[body_part["lshoulder"]][:2], shoulder_base)
        low_hand_angle = points2angle(points[body_part["lshoulder"]][:2], points[body_part["lelbow"]][:2], points[body_part["lwrist"]][:2])
        head_angle = max(points2angle(points[body_part["head"]][:2], points[body_part["neck"]][:2], points[body_part["back"]][:2]) - 15, 0)
        trunk_base= np.array([points[body_part["pelvis"]][0], points[body_part["pelvis"]][1] - 0.5])
        trunk = points2angle(points[body_part["neck"]][:2], points[body_part["pelvis"]][:2], trunk_base)

        if math.isnan(up_hand_angle):
            up_hand_angle = 0


        if math.isnan(low_hand_angle):
            low_hand_angle = 0

        if math.isnan(head_angle):
            head_angle = 0

        if math.isnan(trunk):
            trunk = 0
        
        # left_values = 'left up angle: {0} \n' \
        #     'left low angle: {1} \n' \
        #     'head angle: {2} \n' \
        #     'trunk angle: {3} \n' \
        #     .format(int(up_hand_angle), int(low_hand_angle), int(head_angle), int(trunk))
        # print(left_values)


        try:
            self.l_side_results.put(np.array([up_hand_angle, low_hand_angle, head_angle, 180 -  trunk]), block=False)

        except queue.Full:
            # print("Queue is full, left side skipped.")
            pass






    def front_point2D_reciever(self, msg):
        points = np.array(msg.data).reshape(-1,3)
        
        # transform the 138 keypoint to 17 useful keypoints according to the 3.6M order
        if self.kpts_keeper is None:
            self.kpts_keeper = Point_Transformer(vid_size=self.vid_size, scale_range=[1,1])
        points = self.kpts_keeper.transform(points)


        # <110 yes
        shoulder_angle_r = points2angle(points[body_part["rshoulder"]][:2], points[body_part["neck"]][:2],points[body_part["back"]][:2])
        r_raised = 1 if shoulder_angle_r < 110 else 0
        shoulder_angle_l = points2angle(points[body_part["lshoulder"]][:2], points[body_part["neck"]][:2],points[body_part["back"]][:2])
        l_raised = 1 if shoulder_angle_l < 110 else 0

        # <140 yes
        r_up_abduction_angle = points2angle(points[body_part["relbow"]][:2], points[body_part["rshoulder"]][:2], points[body_part["rhip"]][:2])
        r_up_abduction = 1 if r_up_abduction_angle < 140 else 0
        l_up_abduction_angle = points2angle(points[body_part["lelbow"]][:2], points[body_part["lshoulder"]][:2], points[body_part["lhip"]][:2])
        l_up_abduction = 1 if l_up_abduction_angle < 140 else 0

        #  .5<v<.7 no
        r_low_abduction_dis = point_to_line_distance(points[body_part["neck"]][:2], points[body_part["pelvis"]][:2], points[body_part["rwrist"]][:2])
        r_low_abduction = 0 if r_low_abduction_dis > .35 and r_low_abduction_dis < .65 else 1
        l_low_abduction_dis = point_to_line_distance(points[body_part["neck"]][:2], points[body_part["pelvis"]][:2], points[body_part["lwrist"]][:2])
        l_low_abduction = 0 if l_low_abduction_dis > .35 and l_low_abduction_dis < .65 else 1

        # >.1 yes
        neck_twist = point_to_line_distance(points[body_part["neck"]][:2], points[body_part["head"]][:2], points[body_part["nose"]][:2])
        n_twist = 1 if neck_twist > 0.5 else 0


        # 55<v<70 no
        neck_bending =  points2angle(points[body_part["lshoulder"]][:2], points[body_part["neck"]][:2], points[body_part["head"]][:2])
        n_bending = 0 if neck_bending > 50 and neck_bending < 73 else 1
 

        # 72<v<78 no
        side_bending = points2angle(points[body_part["neck"]][:2], points[body_part["pelvis"]][:2], points[body_part["rhip"]][:2])
        s_bending = 0 if side_bending > 72 and side_bending < 82 else 1

        trunk_twist_r = point_to_line_distance(points[body_part["neck"]][:2], points[body_part["back"]][:2], points[body_part["rshoulder"]][:2])
        trunk_twist_l = point_to_line_distance(points[body_part["neck"]][:2], points[body_part["back"]][:2], points[body_part["lshoulder"]][:2])
        trunk_twist = 1 if trunk_twist_l < 0.5 or trunk_twist_r < 0.5  else 0


        # print('rihgt shoulder angle: {0} \n' \
        # 'left shoulder angle: {1} \n' \
        # 'right abduction angle: {2} \n' \
        # 'left abduction angle: {3} \n' \
        # 'right low abduction dis: {4} \n' \
        # 'left  low abduction dis: {5} \n' \
        # 'neck twist dis: {6} \n' \
        # 'neck  bending angle: {7} \n' \
        # 'side bending angle: {8} \n' \
        # 'right trunk twist dis: {9} \n' \
        # 'left  trunk twist dis: {10} \n' \
        # .format(shoulder_angle_r, shoulder_angle_l, r_up_abduction_angle, l_up_abduction_angle, r_low_abduction_dis, l_low_abduction_dis, neck_twist, neck_bending, side_bending, trunk_twist_r, trunk_twist_l)
        # )

        try:
            self.front_results.put(np.array([r_raised, l_raised, r_up_abduction, l_up_abduction, r_low_abduction, l_low_abduction, n_twist, n_bending, s_bending]), block=False)
        except queue.Full:
            # print("Queue is full, right side skipped.")
            pass






    def rula_calculation(self, side_values, front_values):
        
        up_hand_angle, low_hand_angle, head_angle, trunk = side_values
        raised, up_abduction, low_abduction, n_twist, n_bending, s_bending = front_values

  
        # calculation of the upper hands score
        up_score = 1
        if up_hand_angle > 20 and up_hand_angle < 45:
            up_score = 2
        elif up_hand_angle > 45 and up_hand_angle < 90:
            up_score = 3
        elif up_hand_angle > 90:
            up_score = 4
        # adding abduction and shoulder raised to upper hand
        up_score += raised + up_abduction + self.args.arm_support


        # low hand score calculation
        if low_hand_angle > 60 and low_hand_angle < 100:
            lower_score = 1
        else:
            lower_score = 2
        
        lower_score += low_abduction

        # final score of hand and wrist (wrist not calculated and set to 1 score as default)
        up_final = tableA_in[((up_score - 1) * 3 + lower_score) - 1, 0]
        up_final += self.args.muscle_use + self.args.load_score
        ###############################################################################################

        # neck score calculation
        neck_score = 1

        if head_angle > 0 and head_angle < 10:
            neck_score = 1
        elif head_angle >= 10 and head_angle < 20:
            neck_score = 2
        elif head_angle >= 20:
            neck_score = 3
        else:
            neck_score = 4
        
        neck_score += n_twist + n_bending

        # trunk score calculation
        trunk_score = 1
        
        if trunk > 0 and trunk < 10:
            trunk_score = 1
        elif trunk >= 10 and trunk < 20 :
            trunk_score = 2
        elif trunk >= 20 and trunk < 60:
            trunk_score = 3
        else:
            trunk_score = 4
        
        trunk_score += s_bending

        # trunk, neck final score
        trunk_neck_score = tableB_in[neck_score - 1, (trunk_score * 2 + self.args.leg_support) - 1]
        trunk_neck_score += self.args.muscle_use + self.args.load_score


        # The final Rula score retrieving
        # print('UP final value {0}\nDown final value {1}'.format(min(up_final, 6) - 1,  min(trunk_neck_score, 11 ) - 1))
        final_score = tableB_in[min(up_final, 6) - 1, min(trunk_neck_score, 11 ) - 1]
        
        return final_score, up_score, lower_score, up_final, neck_score, trunk_score, trunk_neck_score




    
    def rula_score(self, side, front, left=None, right=None):
        whole_body_msg = BodyMsg()
        r_raised, l_raised, r_up_abduction, l_up_abduction, r_low_abduction, l_low_abduction, n_twist, n_bending, s_bending = front
        
        whole_body_msg.right_shoulder = int(r_raised)
        whole_body_msg.left_shoulder = int(l_raised)
        whole_body_msg.right_up_abduction = int(r_up_abduction)
        whole_body_msg.left_up_abduction = int(l_up_abduction)
        whole_body_msg.right_low_abduction = int(r_low_abduction)
        whole_body_msg.left_low_abduction = int(l_low_abduction)
        whole_body_msg.neck_twist = int(n_twist)
        whole_body_msg.neck_bending = int(n_bending)
        whole_body_msg.side_bending = int(s_bending)

        whole_body_msg.right_arm_up = 0.0 
        whole_body_msg.left_arm_up = 0.0
        whole_body_msg.right_low_angle = 0.0
        whole_body_msg.left_low_angle = 0.0
        whole_body_msg.neck_angle = 0.0
        whole_body_msg.trunk_angle = 0.0
        whole_body_msg.left_rula_score = 0
        whole_body_msg.right_rula_score = 0
        whole_body_msg.up_arm_score_right = 0
        whole_body_msg.up_arm_score_left = 0
        whole_body_msg.lower_arm_score_right = 0
        whole_body_msg.lower_arm_score_left = 0
        whole_body_msg.neck_score = 0
        whole_body_msg.trunk_score = 0
        whole_body_msg.right = False
        whole_body_msg.left =  False

        # final_score, up_score, lower_score, up_final, neck_score, trunk_score, trunk_neck_score
         

        if side == 'left' or side == 'both':
            l_up_hand_angle, l_low_hand_angle, l_head_angle, l_trunk = left
            rula_score_left = self.rula_calculation([l_up_hand_angle, l_low_hand_angle,  l_head_angle, l_trunk], [l_raised, l_up_abduction, l_low_abduction, n_twist, n_bending, s_bending ])
            whole_body_msg.left_arm_up = l_up_hand_angle
            whole_body_msg.left_low_angle = l_low_hand_angle
            whole_body_msg.neck_angle = l_head_angle
            whole_body_msg.trunk_angle = l_trunk

            whole_body_msg.left_rula_score = int(rula_score_left[0])
            whole_body_msg.up_arm_score_left = int(rula_score_left[1])
            whole_body_msg.lower_arm_score_left = int(rula_score_left[2])
            whole_body_msg.neck_score = int(rula_score_left[4])
            whole_body_msg.trunk_score = int(rula_score_left[5])
            whole_body_msg.left = True

        if side == 'right' or side == 'both':
            r_up_hand_angle, r_low_hand_angle, r_head_angle, r_trunk = right
            rula_score_right = self.rula_calculation([r_up_hand_angle, r_low_hand_angle, r_head_angle, r_trunk], [r_raised, r_up_abduction, r_low_abduction, n_twist, n_bending, s_bending ])
            whole_body_msg.right_arm_up = r_up_hand_angle
            whole_body_msg.right_low_angle = r_low_hand_angle
            whole_body_msg.up_arm_score_right = int(rula_score_right[1])
            whole_body_msg.lower_arm_score_right = int(rula_score_right[2])
            whole_body_msg.right = True

            if side == 'both':
                whole_body_msg.neck_angle =  (whole_body_msg.neck_angle + r_head_angle)/2
                whole_body_msg.trunk_angle = (whole_body_msg.trunk_angle + r_trunk )/2
                whole_body_msg.neck_score = int((whole_body_msg.neck_score  + rula_score_right[4])/2)
                whole_body_msg.trunk_score = int((whole_body_msg.trunk_score + rula_score_right[5])/2)
            else:
                whole_body_msg.neck_angle =  r_head_angle
                whole_body_msg.trunk_angle = r_trunk
                whole_body_msg.neck_score = int(rula_score_right[4])
                whole_body_msg.trunk_score = int(rula_score_right[5])

            whole_body_msg.right_rula_score = int(rula_score_right[0])

        self.publisher_full_body_data.publish(whole_body_msg)



def main(args=None):
    rclpy.init(args=args)
    node = rula_calculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()