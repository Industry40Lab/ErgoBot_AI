import math
import numpy as np
import pandas as pd
import os
from ament_index_python.packages import get_package_share_directory


package_share_dir = get_package_share_directory('rula_assessment')
rula_file_path = os.path.join(package_share_dir, 'configs','rula_score')
reba_file_path = os.path.join(package_share_dir, 'configs','reba_score')
ows_file_path = os.path.join(package_share_dir, 'configs','ows_score')







class rula_assesment_class():
    
    def __init__(self):
        upper_degree_point = [[-20, 20, 1], [-180, -20, 2], [20, 45, 2], [45, 90, 3], [90, 180, 4]] 
        lower_degree_point = [[60, 100, 1], [0, 60, 2], [100, 180, 2]]
        wrist_degree_point = [[-10, 10, 1], [-30, -10, 1], [ 10, 30, 1], [30, 90, 3], [-90, -30, 3]]
        neck_degree_point =  [[0, 10, 1], [ 10, 20, 2], [ 20, 180, 3], [-180, 0, 4]]
        trunk_degree_point = [[0, 10, 1], [10, 20, 2], [ 20, 60, 3], [60, 180, 4]]
        self.degree_point_list = [upper_degree_point, lower_degree_point, wrist_degree_point, neck_degree_point, trunk_degree_point]
        self.tableA = pd.read_csv(os.path.join(rula_file_path,"TableA.csv"))
        self.tableB = pd.read_csv(os.path.join(rula_file_path,"TableB.csv"))
        self.tableC = pd.read_csv(os.path.join(rula_file_path,"TableC.csv"))

    def point_scores_cal(self, angle, score_list):
        for range in score_list:
            if angle < range[1] and angle > range[0]:
                return range[2]
        return None
        # return [long_arm_angle, 180 - short_arm_angle, 180 - wrist_angle, 180 - neck_angle, 180 - trunk_angle]

    def run(self, right_pose_feature, left_pose_feature):
        left_side_angles, right_side_angle = self.angle_side_cal(left_pose_feature,0), self.angle_side_cal(right_pose_feature,1) 
        if not right_side_angle is None:
            rula_point_scores_right = [self.point_scores_cal(right_side_angle[i], self.degree_point_list[i]) for i in range(len(self.degree_point_list))]
            if not None in rula_point_scores_right:
                final_point = self.retrieve_final_score(rula_point_scores_right)
                # print(final_point)
                return final_point


        # if not right_side_angle is None:
            # rula_point_scores_left = [self.point_scores_cal(right_side_angle[i], self.degree_point_list[i]) for i in range(len(self.degree_point_list))]


        return None

    
    def retrieve_final_score(self, rula_points):
        wrist_arm_score = self.tableA.loc[(self.tableA['UpperArm']==rula_points[0]) & (self.tableA['LowerArm']==rula_points[1]), ['2WT1']].values[0][0]
        print(wrist_arm_score)
        neck_body = self.tableB.loc[ self.tableB['Neck']==rula_points[3] , [str(rula_points[1]) + '1']].values[0][0] +1
        print(neck_body)
        final_score = self.tableC.loc[ self.tableC['Score']== wrist_arm_score , [str(neck_body)]].values[0][0]
        return final_score


        




    def angle_side_cal(self, pose, side, visibility_threshold=0.1):
        if len(pose)== 0:
            return
        # left is 1, and right is 0
        side_list = [[12,14,16,24,18,8,26],[11,13,15,23,17,7,25]]

        shoulder = np.array(pose[side_list[side][0]])
        elbow = np.array(pose[side_list[side][1]])
        wrist = np.array(pose[side_list[side][2]])
        hip = np.array(pose[side_list[side][3]])
        palm = np.array(pose[side_list[side][4]])
        ear = np.array(pose[side_list[side][5]])
        knees = np.array(pose[side_list[side][6]])


        if any(point[3] < visibility_threshold for point in [ear, shoulder, elbow, wrist, hip, palm, knees]):
            # print('invisible degree')
            # print('ear {0}, shoulder {1}, elbow {2}, wrist {3}, hip {4}, palm {5}, knees {6}'.format(ear[3], shoulder[3], elbow[3], wrist[3], hip[3], palm[3], knees[3]))
            return None
        long_arm_angle = self.points2angle(hip[:3],shoulder[:3], elbow[:3])
        short_arm_angle = self.points2angle(shoulder[:3], elbow[:3], wrist[:3])
        wrist_angle = self.points2angle(elbow[:3],wrist[:3], palm[:3])
        neck_angle = self.points2angle(hip[:3],shoulder[:3], ear[:3])
        trunk_angle = self.points2angle(knees[:3],hip[:3],shoulder[:3])

        # print(long_arm_angle, 180 - short_arm_angle, 180 - wrist_angle, 180 - neck_angle, 180 - trunk_angle)

        return [long_arm_angle, 180 - short_arm_angle, 180 - wrist_angle, 180 - neck_angle, 180 - trunk_angle]
    

    def points2angle(self, a, b, c):
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
        return angle_deg
    

class rula_assesment_class():
    
    def __init__(self):
        upper_degree_point = [[-20, 20, 1], [-180, -20, 2], [20, 45, 2], [45, 90, 3], [90, 180, 4]] 
        lower_degree_point = [[60, 100, 1], [0, 60, 2], [100, 180, 2]]
        wrist_degree_point = [[-10, 10, 1], [-30, -10, 1], [ 10, 30, 1], [30, 90, 3], [-90, -30, 3]]
        neck_degree_point =  [[0, 10, 1], [ 10, 20, 2], [ 20, 180, 3], [-180, 0, 4]]
        trunk_degree_point = [[0, 10, 1], [10, 20, 2], [ 20, 60, 3], [60, 180, 4]]
        self.degree_point_list = [upper_degree_point, lower_degree_point, wrist_degree_point, neck_degree_point, trunk_degree_point]
        self.tableA = pd.read_csv(os.path.join(rula_file_path,"TableA.csv"))
        self.tableB = pd.read_csv(os.path.join(rula_file_path,"TableB.csv"))
        self.tableC = pd.read_csv(os.path.join(rula_file_path,"TableC.csv"))

    def point_scores_cal(self, angle, score_list):
        for range in score_list:
            if angle < range[1] and angle > range[0]:
                return range[2]
        return None
        # return [long_arm_angle, 180 - short_arm_angle, 180 - wrist_angle, 180 - neck_angle, 180 - trunk_angle]

    def run(self, right_pose_feature, left_pose_feature):
        left_side_angles, right_side_angle = self.angle_side_cal(right_pose_feature,0), self.angle_side_cal(right_pose_feature,1) 
        if not right_side_angle is None:
            rula_point_scores_right = [self.point_scores_cal(right_side_angle[i], self.degree_point_list[i]) for i in range(len(self.degree_point_list))]
            if not None in rula_point_scores_right:
                final_point = self.retrieve_final_score(rula_point_scores_right)
                # print(final_point)
                return final_point


        # if not right_side_angle is None:
            # rula_point_scores_left = [self.point_scores_cal(right_side_angle[i], self.degree_point_list[i]) for i in range(len(self.degree_point_list))]


        return None

    
    def retrieve_final_score(self, rula_points):
        wrist_arm_score = self.tableA.loc[(self.tableA['UpperArm']==rula_points[0]) & (self.tableA['LowerArm']==rula_points[1]), ['2WT1']].values[0][0]
        neck_body = self.tableB.loc[ self.tableB['Neck']==rula_points[3] , [str(rula_points[1]) + '1']].values[0][0] +1
        final_score = self.tableC.loc[ self.tableC['Score']== wrist_arm_score , [str(neck_body)]].values[0][0]
        return final_score


        




    def angle_side_cal(self, pose, side, visibility_threshold=0.1):
        if len(pose)== 0:
            return
        # left is 1, and right is 0
        side_list = [[12,14,16,24,18,8,26],[11,13,15,23,17,7,25]]

        shoulder = np.array(pose[side_list[side][0]])
        elbow = np.array(pose[side_list[side][1]])
        wrist = np.array(pose[side_list[side][2]])
        hip = np.array(pose[side_list[side][3]])
        palm = np.array(pose[side_list[side][4]])
        ear = np.array(pose[side_list[side][5]])
        knees = np.array(pose[side_list[side][6]])


        if any(point[3] < visibility_threshold for point in [ear, shoulder, elbow, wrist, hip, palm, knees]):
            # print('invisible degree')
            # print('ear {0}, shoulder {1}, elbow {2}, wrist {3}, hip {4}, palm {5}, knees {6}'.format(ear[3], shoulder[3], elbow[3], wrist[3], hip[3], palm[3], knees[3]))
            return None
        long_arm_angle = self.points2angle(hip[:2],shoulder[:2], elbow[:2])
        short_arm_angle = self.points2angle(shoulder[:2], elbow[:2], wrist[:2])
        wrist_angle = self.points2angle(elbow[:2],wrist[:2], palm[:2])
        neck_angle = self.points2angle(hip[:2],shoulder[:2], ear[:2])
        trunk_angle = self.points2angle(knees[:2],hip[:2],shoulder[:2])

        print('upper body angle {0}, lower arm angle {1}, wrist angle {2}, neck angle {3}, trunk angle {4}'.format(int(long_arm_angle), int(180 - short_arm_angle), int(180 - wrist_angle), int(180 - neck_angle), int(180 - trunk_angle)))

        return [long_arm_angle, 180 - short_arm_angle, 180 - wrist_angle, 180 - neck_angle, 180 - trunk_angle]
    

    def points2angle(self, a, b, c):
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
        return angle_deg









class owas_assesment_class():
    
    def __init__(self):
        upper_degree_point = [[0, 90, 1], [90, 180, 2]] 
        trunk_degree_point = [[0, 35, 1], [35, 90, 2]]
        self.degree_point_list = [upper_degree_point, trunk_degree_point]
        self.owsTable = pd.read_csv(os.path.join(ows_file_path,"Ows_Table.csv"))


    def point_scores_cal(self, angle, score_list):
        for range in score_list:
            if angle < range[1] and angle > range[0]:
                return range[2]
        return None

    def run(self, pose_feature):
        side_angles = self.angle_side_cal(pose_feature)
        if not side_angles is None:

            ows_point_scores = [
                self.point_scores_cal(side_angles[0], self.degree_point_list[0]),
                self.point_scores_cal(side_angles[1], self.degree_point_list[0]),
                self.point_scores_cal(side_angles[2], self.degree_point_list[1])
            ]
            
            
            if not None in ows_point_scores:
                final_point = self.retrieve_final_score(ows_point_scores)
                # print(final_point)
                return final_point

        return None

    
    def retrieve_final_score(self, ows_points):
        upper_point = 3 if ows_points[0] == ows_points[1] == 2 else max(ows_points[0], ows_points[1])
        body_point = ows_points[2]
        upper_total_point = str(body_point) + str(upper_point)
        upper_total_point = upper_total_point.replace(" ", '')
        final_score = self.owsTable.loc[self.owsTable['score']== int(upper_total_point) , ['21']].values[0][0]
        return final_score


        


    def angle_side_cal(self, pose, visibility_threshold=0.1):
        if len(pose)== 0:
            return

        side_list = [[12,24,18,26],[11,23,17,25]]

        r_shoulder = np.array(pose[side_list[1][0]])
        r_hip = np.array(pose[side_list[1][1]])
        r_palm = np.array(pose[side_list[1][2]])
        l_shoulder = np.array(pose[side_list[0][0]])
        l_hip = np.array(pose[side_list[0][1]])
        l_palm = np.array(pose[side_list[0][2]])
        knees = np.array(pose[side_list[0][3]])


        if not any(point[3] > visibility_threshold for point in [r_shoulder, l_shoulder, r_hip, l_hip, r_palm, l_palm, knees]):
            # print('invisible degree')
            # print('ear {0}, shoulder {1}, elbow {2}, wrist {3}, hip {4}, palm {5}, knees {6}'.format(ear[3], shoulder[3], elbow[3], wrist[3], hip[3], palm[3], knees[3]))
            return None
        r_long_arm_angle = self.points2angle(r_hip[:2],r_shoulder[:2], r_palm[:2])
        l_long_arm_angle = self.points2angle(r_hip[:2],l_shoulder[:2], l_palm[:2])
        trunk_angle = self.points2angle(knees[:2],l_hip[:2],l_shoulder[:2])

        # print('right_arm_angle {0}, left arm angle {1}, trunk {2}'.format(r_long_arm_angle, l_long_arm_angle, 180 - trunk_angle))
        return [r_long_arm_angle, l_long_arm_angle, 180 - trunk_angle]
    

    def points2angle(self, a, b, c):
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
        return angle_deg