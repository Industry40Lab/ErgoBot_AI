import tkinter
import customtkinter
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16
import os
from PIL import Image, ImageDraw
import numpy as np
import cv2
from colorama import Fore
import time
from PIL import ImageTk, Image
from ament_index_python.packages import get_package_share_directory
import cv2
import os
import warnings
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge, CvBridgeError
import random  # just for demonstration of progressbar value update
from hri_msgs.msg import IdsList, LiveSpeech
from body_data.msg import BodyMsg
from std_srvs.srv import SetBool
import queue
from ament_index_python.packages import get_package_share_directory
import os

package_name = 'rula_gui'  # Replace with your package name

# Get the absolute path to the package's `share` directory
package_share_path = get_package_share_directory(package_name)

# Example: Load a resource file (e.g., config.yaml) from a "config" subfolder
resource = os.path.join(package_share_path, 'resource')
frame_path = os.path.join(resource, 'no_frame.png')

warnings.filterwarnings("ignore")

class rula_gui(Node):


    def __init__(self):
        super().__init__('rula_gui')
        self.config()



    def user_identifier(self):
        # Create a publisher to the topic
        pub = self.create_publisher(IdsList, '/humans/voices/tracked', 10)
        
        # Create the message
        msg = IdsList()
        msg.ids = ['system_controller']
        # Publish once
        pub.publish(msg)

        self.control_pub = self.create_publisher(LiveSpeech, '/humans/voices/system_controller/speech', 10)





    def ui_design(self):
        customtkinter.set_appearance_mode("System")
        customtkinter.set_default_color_theme("blue")

        # Main app window
        self.app = customtkinter.CTk(fg_color="black")
        self.app.geometry("2000x1350")
        self.app.title("COBOT GUI")
        self.app.resizable(False, False)

        # Common background color
        # common_bg_color = "transparent"
        common_bg_color = "black"

        # Title Label common_bg_color
        title = customtkinter.CTkLabel(self.app, text="RULA BODY POSTURE ASSESSMENT", fg_color=common_bg_color,text_color="white", font=customtkinter.CTkFont(size=20, weight="bold"))
        title.pack(pady=10)

        # === Main Frames ===
        main_left_frame = customtkinter.CTkFrame(self.app, fg_color="black",  width=1280, height=1250)
        main_left_frame.pack(side="left", pady=0, padx=0, fill="x")
        # Create a thin vertical frame
        vertical_line = customtkinter.CTkFrame(self.app, width=2, fg_color="gray")
        vertical_line.pack(side='left', fill='y', padx=(0,10), pady=10) # Fills vertically, with padding

        main_right_frame = customtkinter.CTkFrame(self.app, fg_color="black",  width=650, height=1250)
        main_right_frame.pack(side="left", pady=0, padx=0)


        # ==== Aavtar Creation ===
        # Avatar (to the left of the scroll frame)
        self.avatar_label = customtkinter.CTkLabel(
            main_right_frame,
            text="",
            # image=self.avatar_img,
            width=650,
            height=1200,
            fg_color="#050505",
            corner_radius=8,
            anchor="center"   # centers image inside the label
        )
        self.avatar_label.pack(expand=True, anchor="center", padx=(0, 0))
        self.avatar_img = self.update_avatar()  # returns a CTkImage

        # Frame to hold the 3 camera frames horizontally
        camera_frame = customtkinter.CTkFrame(main_left_frame, fg_color='black')
        camera_frame.pack(pady=2, padx=20, fill="both", expand=True)

        # === Camera Frame 1 ===
        self.frame_left = customtkinter.CTkFrame(camera_frame, fg_color="gray", border_width=2, border_color="black", corner_radius=5)
        self.frame_left.grid(row=0, column=0, padx=10, sticky="n")

        self.camera_frame_left = customtkinter.CTkLabel( self.frame_left, text="",  image=self.left_img_container_, width=410, height=410, fg_color="#333333", corner_radius=10)
        self.camera_frame_left.pack(padx= 5,pady=5)

        self.label_left = customtkinter.CTkLabel(self.frame_left, text="Status: Waiting...\n\n\n\n\n\n\n\n\n",
                                                  font=customtkinter.CTkFont(size=14), text_color="orange")
        self.label_left.pack(pady=5)

        # === Camera Frame 2 ===
        self.frame_center = customtkinter.CTkFrame(camera_frame, fg_color="gray", border_width=2, border_color="black", corner_radius=5)
        self.frame_center.grid(row=0, column=1, padx=10, sticky="n")

        self.camera_frame_center = customtkinter.CTkLabel(self.frame_center, text="", image=self.front_img_container_, width=410, height=410, fg_color="#333333", corner_radius=10)
        self.camera_frame_center.pack(padx= 5,pady=5)

        self.label_center = customtkinter.CTkLabel(self.frame_center, text="Status: Waiting...\n\n\n\n\n\n\n\n\n",
                                                  font=customtkinter.CTkFont(size=14), text_color="orange")
        self.label_center.pack(pady=5)

        # === Camera Frame 3 ===
        self.frame_right = customtkinter.CTkFrame(camera_frame, fg_color="gray", border_width=2, border_color="black", corner_radius=5)
        self.frame_right.grid(row=0, column=2, padx=10, sticky="n")

        self.camera_frame_right = customtkinter.CTkLabel(self.frame_right, text="", image=self.right_img_container_, width=410, height=410, fg_color="#333333", corner_radius=10)
        self.camera_frame_right.pack(padx= 5,pady=5)

        self.label_right = customtkinter.CTkLabel(self.frame_right, text="Status: Waiting...\n\n\n\n\n\n\n\n\n",
                                                  font=customtkinter.CTkFont(size=14), text_color="orange")
        self.label_right.pack(pady=5)


        # === New Section: Button + Avatar + Scrollable Text ===
        action_section = customtkinter.CTkFrame(main_left_frame, fg_color="black")
        action_section.pack(pady=5, padx=20, fill="x")

        # Left-aligned Start/Stop button
        self.action_button = customtkinter.CTkButton(
            action_section, fg_color="green", text="Start Recording", command=self.recording
        )
        self.action_button.pack(side="left", padx=(10, 15), pady=10)

        # Scrollable text area, slightly narrower to make room for the avatar
        self.text_scroll_frame = customtkinter.CTkScrollableFrame(
            action_section, width=400, height=150, fg_color="white"
        )
        self.text_scroll_frame.pack(side="left", padx=5, pady=5, fill="both", expand=True)

        self.text_output = customtkinter.CTkLabel(
            self.text_scroll_frame, text="", wraplength=380, justify="left"
        )
        self.text_output.pack(anchor="w", padx=2, pady=2)



        ##########################################################################
        bottom_frame = customtkinter.CTkFrame(main_left_frame, fg_color=common_bg_color)
        bottom_frame.pack()

        self.progress_label = customtkinter.CTkLabel(bottom_frame, text_color="white", text="Posture Score", font=customtkinter.CTkFont(size=16))
        self.progress_label.pack()

        # Create a subframe with two columns for LEFT and RIGHT progress bars
        progress_container = customtkinter.CTkFrame(bottom_frame, fg_color=common_bg_color)
        progress_container.pack()

        # LEFT progress bar and label
        left_progress_frame = customtkinter.CTkFrame(progress_container, fg_color=common_bg_color)
        left_progress_frame.grid(row=0, column=0, padx=20, pady=5)

        left_label_row = customtkinter.CTkFrame(left_progress_frame, fg_color=common_bg_color)
        left_label_row.pack(anchor="w")

        left_label = customtkinter.CTkLabel(left_label_row, text_color="white", text="LEFT", font=customtkinter.CTkFont(size=16), width=50)
        left_label.pack(side="left", padx=(0, 10))

        self.progressbar_left = customtkinter.CTkProgressBar(left_label_row, width=500, height=25)
        self.progressbar_left.pack(side="left")
        self.progressbar_left.set(0.0)

        self.progress_value_label_left = customtkinter.CTkLabel(left_progress_frame, text_color="white", text="0%", font=customtkinter.CTkFont(size=16))
        self.progress_value_label_left.pack(anchor="e", pady=(5, 0))

        # RIGHT progress bar and label
        right_progress_frame = customtkinter.CTkFrame(progress_container, fg_color=common_bg_color)
        right_progress_frame.grid(row=0, column=1, padx=20, pady=5)

        right_label_row = customtkinter.CTkFrame(right_progress_frame, fg_color=common_bg_color)
        right_label_row.pack(anchor="w")

        right_label = customtkinter.CTkLabel(right_label_row, text_color="white", text="RIGHT", font=customtkinter.CTkFont(size=16), width=50)
        right_label.pack(side="left", padx=(0, 10))

        self.progressbar_right = customtkinter.CTkProgressBar(right_label_row, width=500, height=25)
        self.progressbar_right.pack(side="left")
        self.progressbar_right.set(0.0)

        self.progress_value_label_right = customtkinter.CTkLabel(right_progress_frame, text_color="white", text="0%", font=customtkinter.CTkFont(size=16))
        self.progress_value_label_right.pack(anchor="e", pady=(5, 0))

        # self.update_progressbar()
        self.poll_queues()

        self.app.mainloop()


    def poll_queues(self):
        try:
            frame = self.left_frame_queue.get_nowait()
            self.camera_feed_update(frame, self.camera_frame_left)
        except queue.Empty:
            pass

        try:
            frame = self.right_frame_queue.get_nowait()
            self.camera_feed_update(frame, self.camera_frame_right)
        except queue.Empty:
            pass

        try:
            frame = self.front_frame_queue.get_nowait()
            self.camera_feed_update(frame, self.camera_frame_center)
        except queue.Empty:
            pass

        try:
            msg = self.rula_data_queue.get_nowait()
            self.rula_value_indication(msg)
        except queue.Empty:
            pass


        self.app.after(5, self.poll_queues)  # adjust polling rate as needed


    def camera_feed_update(self, frame, container):

        # Convert frame to PIL Image
        frame_image = Image.fromarray(frame)

        # Resize to fit the image frame, if needed
        frame_image = frame_image.resize((400, 400))

        # Convert the PIL Image to a format Tkinter can use
        photo = ImageTk.PhotoImage(frame_image)

        # Update the image in the label
        container.configure(image=photo)
        container.image = photo  # Keep a reference to avoid garbage collection

        

    def update_rula_left(self, msg):
        self.update_rula(msg, self.progressbar_right, self.progress_value_label_right)    



    def update_rula_right(self, msg):
        self.update_rula(msg, self.progressbar_left, self.progress_value_label_left)    

    def recording(self):
        if self.recording_order.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Recording Service available')
            self.req = SetBool.Request()
            self.req.data = not self.recording_situation
            future = self.recording_order.call_async(self.req)
            future.add_done_callback(self.handle_recording_response) 
        else:
            self.get_logger().warning('the recording service is disabled')


    def handle_recording_response(self, future):
        try:
            response = future.result()
            self.recording_situation = response.success
            message = response.message

            # Update the GUI from the main thread
            def update_button():
                if response.success:
                    self.action_button.configure(text="Stop Recording", fg_color="red")
                else:
                    self.action_button.configure(text="Start Recording", fg_color="green")

            self.app.after(0, update_button)
            self.get_logger().info(f"Service response: {message}")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


    def update_rula(self, value, container, value_container):
        container.set(value / 7)
        value_container.configure(text=f"{value}")

        # Change color based on value
        if value <= 2:
            color = "green"
        elif 2 < value <= 4:
            color = "yellow"
        elif 4 < value <= 6:
            color = "orange"
        else:
            color = "red"
            msg = LiveSpeech()
            msg.final = 'Just paraphrase and repeate this sentence and say it to user that: Your working posture need adjustment and can risk your long-term health. '
            # Publish once
            self.control_pub.publish(msg)

        container.configure(progress_color=color)

        # Update again after 2 seconds
        # self.app.after(2000, container)


    def rula_value_update(self, msg):
        self.rula_data_queue.put(msg)



    def update_text(self, text, container, holder):
        # container = customtkinter.CTkLabel(holder, text_color='orange', text=text, wraplength=500)
        # container.pack(pady=5)
        container.configure(text = text)






    def left_frame_update(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.left_frame_queue.put(frame)

    def right_frame_update(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.right_frame_queue.put(frame)
        
    def front_frame_update(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.front_frame_queue.put(frame)

############################## chat update ##########################################
    def robot_chat(self, msg):
        communicator = customtkinter.CTkLabel(self.text_scroll_frame, text="[COBOT]:\t"+msg.data, text_color='red')
        communicator.pack(anchor="w", padx=5)
   
    def user_chat(self, msg):
        communicator = customtkinter.CTkLabel(self.text_scroll_frame, text="[USER]:\t"+msg.final, text_color='black')
        communicator.pack(anchor="w", padx=5)



############################### Node main function ###################################
######################################################################################

    def config(self):
        self.user_identifier()
        
        self.recording_order = self.create_client(SetBool, 'recording')
        self.recording_situation = False

        self.left_frame_queue = queue.Queue()
        self.right_frame_queue = queue.Queue()
        self.front_frame_queue = queue.Queue()
        self.rula_data_queue = queue.Queue()

        # ==============================
        # CONFIG: assign color per part
        # ==============================
        self.part_colors = {
            "upper_hand_left": "green",
            "upper_hand_right": "green",
            "lower_hand_left": "green",
            "lower_hand_right": "green",
            "neck": "green",
            "back": "green",
            "leg_left": "green",
            "leg_right": "green",
        }
        



        self.color_map = {
            "red": (255, 0, 0, 255),
            "green": (0, 200, 0, 255),
            "yellow": (255, 220, 0, 255),
        }
        self.avatar = None

        # Create a subscription for processed frames 
        self.subscription_front_image_ = self.create_subscription(
            ROSImage,
            '/front_frame_2D',
            self.front_frame_update,
            10  # QoS history depth
        )
        

        self.subscription_right_image_ = self.create_subscription(
            ROSImage,
            '/right_frame_2D',
            self.right_frame_update,
            10  # QoS history depth
        )
        
        self.subscription_left_image_ = self.create_subscription(
            ROSImage,
            '/left_frame_2D',
            self.left_frame_update,
            10  # QoS history depth
        )
        ###############################################################
        ###############################################################
        ###############################################################
        ###############################################################
        ###############################################################
        # Create a subscription for angles and values
        ###############################################################
        ###############################################################
        ###############################################################
        ###############################################################
        ###############################################################
        self.subscription_whole_body_info_ = self.create_subscription(
            BodyMsg,
            '/full_body_data',
            self.rula_value_update,
            10  # QoS history depth
        )


        # Create a CvBridge to convert OpenCV images to ROS messages
        self.br = CvBridge()

        # Load and display the image
        self.front_img_container_ = customtkinter.CTkImage(
            light_image=Image.open(frame_path),
            dark_image=Image.open(frame_path),
            size=(400, 400)
        )

        # Load and display the image
        self.left_img_container_ = customtkinter.CTkImage(
            light_image=Image.open(frame_path),
            dark_image=Image.open(frame_path),
            size=(400, 400)
        )

        # Load and display the image
        self.right_img_container_ = customtkinter.CTkImage(
            light_image=Image.open(frame_path),
            dark_image=Image.open(frame_path),
            size=(400, 400)
        )

        gui_thread = threading.Thread(target=self.ui_design)
        gui_thread.start()
        # Subscribe to a topic
        self.cobot_answer = self.create_subscription(
            String,
            '/speak_text',
            self.robot_chat,
            10
        )
        self.user_order = self.create_subscription(
            LiveSpeech, 
            '/humans/voices/user/speech',
            self.user_chat,
            10
        )


    def rula_value_indication(self, msg):
        # neck and trunk
        if msg.neck_score <= 2:
            self.part_colors["neck"] = "green"
        elif msg.neck_score > 2 and msg.neck_score < 5:
            self.part_colors["neck"] = "yellow"
        elif msg.neck_score > 4:
            self.part_colors["neck"] = "red"

        if msg.trunk_score <= 2:
            self.part_colors["back"] = "green"
        elif msg.trunk_score > 2 and msg.trunk_score < 5:
            self.part_colors["back"] = "yellow"
        elif msg.trunk_score > 4:
            self.part_colors["back"] = "red"
        # print('The trunk score {0}\nThe neck score {1}'.format(msg.trunk_score, msg.neck_score))


        if msg.right and not msg.left:

            front_values = 'rihgt shoulder raised: {0} \n' \
                    'left shoulder raised: {1} \n' \
                    'right abduction: {2} \n' \
                    'left abduction: {3} \n' \
                    'right low abduction: {4} \n' \
                    'left  low abduction: {5} \n' \
                    'neck twist: {6} \n' \
                    'neck  bending: {7} \n' \
                    'side bending: {8} \n' \
                    .format(msg.right_shoulder, msg.left_shoulder, 
                            msg.right_up_abduction ,                       
                            msg.left_up_abduction,
                            msg.right_low_abduction,
                            msg.left_low_abduction,
                            msg.neck_twist,
                            msg.neck_bending,
                            msg.side_bending
                            )


            right_values = 'rihgt up angle: {0:.2f} \n' \
                'right low angle: {1:.2f} \n' \
                'head angle: {2:.2f} \n' \
                'trunk angle: {3:.2f} \n' \
                .format(msg.right_arm_up,
                        msg.right_low_angle,
                        msg.neck_angle,
                        msg.trunk_angle)
        


            # Avatar Color
            # upper arms
            if msg.up_arm_score_right <= 2:
                self.part_colors["upper_hand_right"] = "green"
            elif msg.up_arm_score_right > 2 and msg.up_arm_score_right < 5:
                self.part_colors["upper_hand_right"] = "yellow"
            elif msg.up_arm_score_right > 4:
                self.part_colors["upper_hand_right"] = "red"

            # lower arms
            if msg.lower_arm_score_right <= 1:
                self.part_colors["lower_hand_right"] = "green"
            elif msg.lower_arm_score_right > 1 and msg.lower_arm_score_right < 3:
                self.part_colors["lower_hand_right"] = "yellow"
            elif msg.lower_arm_score_right > 2:
                self.part_colors["lower_hand_right"] = "red"

            _ = self.update_avatar()
            self.update_text(right_values, self.label_right, self.frame_right)
            self.update_text(front_values, self.label_center, self.frame_center)
            self.update_rula(msg.right_rula_score, self.progressbar_right, self.progress_value_label_right)

        if msg.left and not msg.right:

            front_values = 'rihgt shoulder raised: {0} \n' \
                    'left shoulder raised: {1} \n' \
                    'right abduction: {2} \n' \
                    'left abduction: {3} \n' \
                    'right low abduction: {4} \n' \
                    'left  low abduction: {5} \n' \
                    'neck twist: {6} \n' \
                    'neck  bending: {7} \n' \
                    'side bending: {8} \n' \
                    .format(msg.right_shoulder, msg.left_shoulder, 
                            msg.right_up_abduction ,                       
                            msg.left_up_abduction,
                            msg.right_low_abduction,
                            msg.left_low_abduction,
                            msg.neck_twist,
                            msg.neck_bending,
                            msg.side_bending
                            )

            left_values = 'left up angle: {0:.2f} \n' \
                'left low angle: {1:.2f} \n' \
                'head angle: {2:.2f} \n' \
                'trunk angle: {3:.2f} \n' \
                .format(msg.left_arm_up,
                        msg.left_low_angle,
                        msg.neck_angle,
                        msg.trunk_angle
                        )
            

            if msg.up_arm_score_left <= 2:
                self.part_colors["upper_hand_left"] = "green"
            elif msg.up_arm_score_left > 2 and msg.up_arm_score_left < 5:
                self.part_colors["upper_hand_left"] = "yellow"
            elif msg.up_arm_score_left > 4:
                self.part_colors["upper_hand_left"] = "red"

            # lower arms
            if msg.lower_arm_score_left <= 1:
                self.part_colors["lower_hand_left"] = "green"
            elif msg.lower_arm_score_left > 1 and msg.lower_arm_score_left < 3:
                self.part_colors["lower_hand_left"] = "yellow"
            elif msg.lower_arm_score_left > 2:
                self.part_colors["lower_hand_left"] = "red"

            _ = self.update_avatar()
            self.update_text(left_values, self.label_left, self.frame_left)
            self.update_text(front_values, self.label_center, self.frame_center)
            self.update_rula(msg.left_rula_score, self.progressbar_left, self.progress_value_label_left)   


        if msg.right and msg.left:

            front_values = 'right shoulder raised: {0} \n' \
                    'left shoulder raised: {1} \n' \
                    'right abduction: {2} \n' \
                    'left abduction: {3} \n' \
                    'right low abduction: {4} \n' \
                    'left  low abduction: {5} \n' \
                    'neck twist: {6} \n' \
                    'neck  bending: {7} \n' \
                    'side bending: {8} \n' \
                    .format(msg.right_shoulder, msg.left_shoulder, 
                            msg.right_up_abduction ,                       
                            msg.left_up_abduction,
                            msg.right_low_abduction,
                            msg.left_low_abduction,
                            msg.neck_twist,
                            msg.neck_bending,
                            msg.side_bending
                            )

            left_values = 'left up angle: {0:.2f} \n' \
                'left low angle: {1:.2f} \n' \
                'head angle: {2:.2f} \n' \
                'trunk angle: {3:.2f} \n' \
                .format(msg.left_arm_up,
                        msg.left_low_angle,
                        msg.neck_angle,
                        msg.trunk_angle
                        )
            
            right_values = 'right up angle: {0:.2f} \n' \
                'right low angle: {1:.2f} \n' \
                'head angle: {2:.2f} \n' \
                'trunk angle: {3:.2f} \n' \
                .format(msg.right_arm_up,
                        msg.right_low_angle,
                        msg.neck_angle,
                        msg.trunk_angle)


            if msg.up_arm_score_left <= 2:
                self.part_colors["upper_hand_left"] = "green"
            elif msg.up_arm_score_left > 2 and msg.up_arm_score_left < 5:
                self.part_colors["upper_hand_left"] = "yellow"
            elif msg.up_arm_score_left > 4:
                self.part_colors["upper_hand_left"] = "red"

            # lower arms
            if msg.lower_arm_score_left <= 1:
                self.part_colors["lower_hand_left"] = "green"
            elif msg.lower_arm_score_left > 1 and msg.lower_arm_score_left < 3:
                self.part_colors["lower_hand_left"] = "yellow"
            elif msg.lower_arm_score_left > 2:
                self.part_colors["lower_hand_left"] = "red"

            # Avatar Color
            # upper arms
            if msg.up_arm_score_right <= 2:
                self.part_colors["upper_hand_right"] = "green"
            elif msg.up_arm_score_right > 2 and msg.up_arm_score_right < 5:
                self.part_colors["upper_hand_right"] = "yellow"
            elif msg.up_arm_score_right > 4:
                self.part_colors["upper_hand_right"] = "red"

            # lower arms
            if msg.lower_arm_score_right <= 1:
                self.part_colors["lower_hand_right"] = "green"
            elif msg.lower_arm_score_right > 1 and msg.lower_arm_score_right < 3:
                self.part_colors["lower_hand_right"] = "yellow"
            elif msg.lower_arm_score_right > 1:
                self.part_colors["lower_hand_right"] = "red"
                
            _ = self.update_avatar()
            self.update_text(right_values, self.label_right, self.frame_right)
            self.update_text(left_values, self.label_left, self.frame_left)
            self.update_text(front_values, self.label_center, self.frame_center)
            self.update_rula(msg.left_rula_score, self.progressbar_left, self.progress_value_label_left)   
            self.update_rula(msg.right_rula_score, self.progressbar_right, self.progress_value_label_right)








    def update_avatar(self):

        # ==============================
        # Create canvas
        # ==============================
        W, H = 650, 1200
        img = Image.new("RGBA", (W, H), (255, 255, 255, 0))
        draw = ImageDraw.Draw(img)

        cx = W // 2  # center x
        cy = H // 2

        # Helper for rounded rectangle
        def rrect(xy, radius, color):
            draw.rounded_rectangle(xy, radius, fill=color, outline="black", width=3)

        # ==============================
        # Neck
        # ==============================
        neck_box = (cx - 100, cy - 500, cx + 100, cy - 350)
        rrect(neck_box, 90, self.color_map[self.part_colors["neck"]])
        neck_box = (cx - 25, cy - 350, cx + 25, cy - 300)
        rrect(neck_box, 10, self.color_map[self.part_colors["neck"]])



        # Back
        back_box = (cx - 130, cy - 300, cx + 130, cy )
        rrect(back_box, 30, self.color_map[self.part_colors["back"]])

        # Upper Hands (left/right)
        upper_hand_h = 190
        upper_hand_w = 60
        l_upper = (back_box[0] - upper_hand_w, cy - 280, back_box[0], cy - 280 + upper_hand_h)
        r_upper = (back_box[2], cy - 280, back_box[2] + upper_hand_w, cy - 280 + upper_hand_h)
        rrect(l_upper, 20, self.color_map[self.part_colors["upper_hand_left"]])
        rrect(r_upper, 20, self.color_map[self.part_colors["upper_hand_right"]])

        # Lower Hands (left/right)
        lower_hand_h = 120
        lower_hand_w = 60
        l_lower = (l_upper[0], l_upper[3], l_upper[0] + lower_hand_w, l_upper[3] + lower_hand_h)
        r_lower = (r_upper[0], r_upper[3], r_upper[0] + lower_hand_w, r_upper[3] + lower_hand_h)
        rrect(l_lower, 15, self.color_map[self.part_colors["lower_hand_left"]])
        rrect(r_lower, 15, self.color_map[self.part_colors["lower_hand_right"]])

        # Legs (now SEPARATED left and right)
        leg_w, leg_h = 60, 320
        gap = 20  # gap between legs

        # Left leg
        l_leg = (cx - gap//2 - leg_w, back_box[3], cx - gap//2, back_box[3] + leg_h)
        rrect(l_leg, 25, self.color_map[self.part_colors["leg_left"]])

        # Right leg
        r_leg = (cx + gap//2, back_box[3], cx + gap//2 + leg_w, back_box[3] + leg_h)
        rrect(r_leg, 25, self.color_map[self.part_colors["leg_right"]])


        # Resize nicely for the small avatar slot
        img_small = img.resize((650, 1200), Image.LANCZOS)

        # Return a CTkImage (works best with CustomTkinter)
        avatar_ctk = customtkinter.CTkImage(light_image=img_small, dark_image=img_small, size=(650, 1200))
        self.avatar_label.configure(image=avatar_ctk)
        return avatar_ctk


def main(args=None):
    rclpy.init(args=args)
    
    # Create a subscriber node
    rul_ui = rula_gui()

    # Spin to keep the node running and responsive to callbacks
    rclpy.spin(rul_ui)

    # Shutdown after exiting
    rul_ui.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
