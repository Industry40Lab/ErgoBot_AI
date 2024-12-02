import tkinter
import customtkinter
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import ollama
import cv2
from colorama import Fore
import pyttsx3
import time
from PIL import ImageTk, Image
from rula_assessment.pose_landmarks import pose_landmark
from rula_assessment.realsense_class import RealSense_Cam
from ament_index_python.packages import get_package_share_directory
import cv2
import os
from rula_assessment.angle_calc import owas_assesment_class
import warnings
warnings.filterwarnings("ignore")

class ergo_gui(Node):


    def __init__(self):
        super().__init__('ergo_gui')

        
        self.publisher_ = self.create_publisher(String, '/robot_listening', 10)
        self.publish_userInfo_ = self.create_publisher(String, '/recieved_order', 10)
        self.publisher_command_ = self.create_publisher(String, '/classified_order', 10)
        self.publisher_notifSender_ = self.create_publisher(String, '/recieved_notif', 10)
        # Create a subscription to the /received_command topic
        self.subscription = self.create_subscription(
            String,
            '/received_command',
            self.listener_callback,
            10  # QoS history depth
        )
        
        # Create a subscription to the /received_command topic
        self.subscription_llm = self.create_subscription(
            String,
            '/llm_answer',
            self.llm_answer,
            10  # QoS history depth
        )


        # Create a subscription to the /received_command topic
        self.notif_data_llm = self.create_subscription(
            String,
            '/notif_llm',
            self.notif_call_back,
            10  # QoS history depth
        )





        # Load and display the image
        self.image = customtkinter.CTkImage(
            light_image=Image.open('/home/i40lab/arise_usecase2_ws/src/ergo_gui/resource/no_frame.png'),
            dark_image=Image.open('/home/i40lab/arise_usecase2_ws/src/ergo_gui/resource/no_frame.png'),
            size=(360, 400)
        )

        gui_thread = threading.Thread(target=self.ui_design)
        gui_thread.start()

        self.setup()



    def setup(self):

        #voic answer setting up
        self.engine = pyttsx3.init()

        # Set the speech rate if necessary (optional)
        rate = self.engine.getProperty('rate')
        # msg = String()
        # msg.data = "Welcom the oprator and introduce yourself to oprator."
        # self.publisher_.publish(msg)
        self.on_notif = True
        self.dis = False

        self.ows_class = owas_assesment_class()

        # rula assessment files reading
        self.package_share_dir = get_package_share_directory('rula_assessment')
        self.config_file_path = os.path.join(self.package_share_dir, 'configs')
        print(f"Config file path: {self.config_file_path}")

        self.cam = RealSense_Cam()

        self.pipeline, config = self.cam.start_real_sense()
        # Start streaming
        self.pipeline.start(config)

        self.PoseLandmarks = pose_landmark(task_path=self.config_file_path,task_type="pose_landmarker_lite.task")
        self.PoseLandmarks.create_landmarker()


        # Adjust speed
        self.engine.setProperty('rate', 150)  
        print('here')
        video_thread = threading.Thread(target=self.camera_feed_update)
        video_thread.start()



    def ui_design(self):
        customtkinter.set_appearance_mode("System")
        customtkinter.set_default_color_theme("blue")

        # Initialize main app window
        self.app = customtkinter.CTk()
        self.app.geometry("920x560")
        self.app.title("COBOT GUI")
        self.app.resizable(False, False)

        # Define a common background color for the main layout components
        common_bg_color = "transparent"  # Example background color (dark gray)

        # Title Label
        title = customtkinter.CTkLabel(self.app, text="Welcome to ergo cobot graphic user interface", fg_color=common_bg_color)
        title.pack(padx=10, pady=10)

        # Main Frame that will hold the left (image) and right (components) frames
        main_frame = customtkinter.CTkFrame(self.app, fg_color=common_bg_color)  # Set background color
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)

        # Left frame for the image
        lef_frame = customtkinter.CTkFrame(main_frame, width=360, fg_color=common_bg_color)  # Set background color
        lef_frame.grid(row=0, column=0, sticky="nsw")  # Left side


        self.image_frame = customtkinter.CTkLabel(lef_frame, text="", image=self.image, fg_color=common_bg_color)  # Set background color
        self.image_frame.pack(fill="both", expand=True)
        
        self.warnning_font = customtkinter.CTkFont("Times",16)
        self.posture_score = customtkinter.CTkLabel(lef_frame, text="Posture Score Not Available", fg_color=common_bg_color, text_color='orange', font=self.warnning_font)  # Set background color
        self.posture_score.pack(pady=5)

        # Right frame for the rest of the components
        right_frame = customtkinter.CTkFrame(main_frame, fg_color=common_bg_color)  # Set background color
        right_frame.grid(row=0, column=1, padx=20, pady=10, sticky="nsew")  # Right side


        main_frame.columnconfigure(1, weight=1)

        # Input frame for command box and button
        input_frame = customtkinter.CTkFrame(right_frame, fg_color=common_bg_color)  # Set background color
        input_frame.pack(pady=10)


        # Command box and Send button
        self.text_command = tkinter.StringVar()
        command_box = customtkinter.CTkEntry(
            input_frame, width=350, height=40, placeholder_text="Please Enter your command",
            placeholder_text_color='black', textvariable=self.text_command, fg_color=common_bg_color  # Set background color
        )
        command_box.grid(row=0, column=0, padx=(5, 0))

        self.send_button = customtkinter.CTkButton(input_frame, text="Send", command=self.send_command)
        self.send_button.grid(row=0, column=1, padx=(5, 5))
        # self.send_button.bind('<Enter>',self.send_command_k)


        # Separator and Voice Recorder button
        separator = customtkinter.CTkLabel(right_frame, text="OR", fg_color=common_bg_color)  # Set background color
        separator.pack(pady=10)

        self.recorder_button = customtkinter.CTkButton(right_frame, text="Voice Recorder", command=self.voice_record)
        self.recorder_button.pack(pady=10)
        # self.recorder_button.bind('<Return>',self.voice_record_k)

        # Disable buttons initially
        self.send_button.configure(state="disabled")
        self.recorder_button.configure(state="disabled")

        # Communication section label
        separator = customtkinter.CTkLabel(right_frame, text="----------------------------------------------------------------------------------", fg_color=common_bg_color)  # Set background color
        separator.pack(pady=5)

        comm_label = customtkinter.CTkLabel(right_frame, text="Communication Data", fg_color=common_bg_color)  # Set background color
        comm_label.pack(pady=5)

        # Scrollable frame for communication data
        self.scrollable_frame = customtkinter.CTkScrollableFrame(right_frame, width=350, height=200)
        self.scrollable_frame.pack(padx=10, pady=10)
        # Leave scrollable_frame without specific background color to make it stand out

        # Example content inside the scrollable frame
        communication = customtkinter.CTkLabel(self.scrollable_frame, text='starting communication...')
        communication.pack(anchor="w", padx=10)

        # Run main loop
        self.app.mainloop()


    # def voice_record_k(self,event):
    #     self.voice_record()
    def voice_record(self):
        self.send_button.configure(state="disabled")
        self.recorder_button.configure(state="disabled")
        msg = String()
        msg.data = "robot_ready"
        self.publisher_.publish(msg)

    # def send_command_k(self,event):
    #     self.send_command()
    def send_command(self):

        self.send_button.configure(state="disabled")
        self.recorder_button.configure(state="disabled")

        order = self.text_command.get()
        
        self.get_logger().info(Fore.RED + "User:\n")
        print( Fore.GREEN + order)

        communicator = customtkinter.CTkLabel(self.scrollable_frame, text="User:\n", text_color='red')
        communicator.pack(anchor="w", padx=5)

        communication = customtkinter.CTkLabel(self.scrollable_frame, text_color='green', text=order, wraplength=340)
        communication.pack(anchor="w", padx=10)

        self.scrollable_frame._parent_canvas.yview_moveto(1.0)


        msg = String()
        msg.data = order
        self.publish_userInfo_.publish(msg)


    def llm_answer(self,msg):
        self.generate_command(msg.data)




    def publish_robot_command(self, ans):
        order = ans.split("]")
        message = order[1]
        order_info = order[0]
        for char in ["[","(",")",":"," "]:
            order_info = order_info.replace(char,"")
        # topic , value =  order_info.split(",")
        msg = String()
        msg.data = order_info
        self.publisher_command_.publish(msg)
        self.get_logger().info(Fore.BLUE + "Following command is sent: {0}".format(order_info))
        return message



    def listener_callback(self, msg):
        self.get_logger().info(Fore.RED + "User:\n")
        print( Fore.GREEN + msg.data)

        communicator = customtkinter.CTkLabel(self.scrollable_frame, text="User:\n", text_color='red')
        communicator.pack(anchor="w", padx=5)

        communication = customtkinter.CTkLabel(self.scrollable_frame, text_color='green', text=msg.data, wraplength=340)
        communication.pack(anchor="w", padx=10)
        self.scrollable_frame._parent_canvas.yview_moveto(1.0)


        data = String()
        data.data = str(msg.data)
        self.publish_userInfo_.publish(data)


    def notif_call_back(self, msg):
        self.generate_command(msg.data, False)
        self.on_notif = False


    def generate_command(self, msg, order_base = True):

        if order_base:
            msg = self.publish_robot_command(msg)

        self.engine.say(str(msg))
        self.engine.runAndWait()
        text =  "COBOT:\n"
        self.get_logger().info( Fore.RED + text)
        communicator = customtkinter.CTkLabel(self.scrollable_frame, text=text, text_color='red')
        communicator.pack(anchor="w", padx=5)
        
        communication = customtkinter.CTkLabel(self.scrollable_frame, text_color='green', text='', wraplength=320)
        communication.pack(anchor="w", padx=10)

        for chunk in msg:
            text = chunk
            print( Fore.GREEN + chunk, end='', flush=True)
            communication.configure(text= communication._text + text )
            time.sleep(0.075)  # Small delay between words, if necessary
            self.scrollable_frame._parent_canvas.yview_moveto(1.0)


        self.send_button.configure(state="normal")
        self.recorder_button.configure(state="normal")
        print("\n" + "_"*30)






    def camera_feed_update(self):
        while True:
            depth, frame = self.cam.get_frame_from_realsense(self.pipeline,aligned_frame=False)
            self.PoseLandmarks.run_landmarker(frame_bgr=frame)

            if frame is not None:
                if self.PoseLandmarks.pose_landmarks_frame() is not None:

                    frame = self.PoseLandmarks.pose_landmarks_frame()
                    pose_xyz = self.PoseLandmarks.input_of_assessment()
                    if len(pose_xyz) > 0:
                        ows = self.ows_class.run(pose_xyz)
                        if ( not ows is None):
                            if int(ows)==2:
                                self.posture_score.configure(text = f"Posture not proper in upper body", text_color='red')
                                if (not self.on_notif) and not self.dis:
                                    msg = String()
                                    msg.data = "Inform the oprator with a short notification that oprator is in bad posture."
                                    self.publisher_notifSender_.publish(msg)
                                    self.on_notif = True
                                    self.dis = True

                            else:
                                self.posture_score.configure(text = f"Proper posture in upper body", text_color='green')
                            # if int(reba)>4:
                            #     print("Rapid Entire Body Score : "+reba+"Posture not proper in your body")
                            #     print("Posture not proper in your body","Warning")
                            # else:
                            #     print("Rapid Entire Body Score : "+reba)
                        else:
                            self.posture_score.configure(text="Posture Score Not Available", text_color='orange')
                            # print("Posture Incorrect")
                    # Convert frame to RGB (OpenCV uses BGR)
                    # cv2.imshow("pose_detection",frame)

                    # Convert frame to PIL Image
                    frame_image = Image.fromarray(frame)

                    # Resize to fit the image frame, if needed
                    frame_image = frame_image.resize((360, 400))

                    # Convert the PIL Image to a format Tkinter can use
                    self.photo = ImageTk.PhotoImage(frame_image)

                    # Update the image in the label
                    self.image_frame.configure(image=self.photo)
                    self.image_frame.image = self.photo  # Keep a reference to avoid garbage collection

                    cv2.waitKey(25)
                else:
                    self.posture_score.configure(text="Posture Score Not Available", text_color='orange')

            else:

                img = cv2.imread('/home/i40lab/arise_usecase2_ws/src/ergo_gui/resource/no_frame.png')

                # Convert frame to RGB (OpenCV uses BGR)
                frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                
                # Convert frame to PIL Image
                frame_image = Image.fromarray(frame)

                # Resize to fit the image frame, if needed
                frame_image = frame_image.resize((360, 400))

                # Convert the PIL Image to a format Tkinter can use
                self.photo = ImageTk.PhotoImage(frame_image)

                # Update the image in the label
                self.image_frame.configure(image=self.photo)
                self.image_frame.image = self.photo  # Keep a reference to avoid garbage collection
    
                # converted from BGR to RGB 
                color_coverted = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 

                # Displaying the converted image 
                pil_image = Image.fromarray(color_coverted) 
                self.image.configure(
                        light_image=pil_image,
                        dark_image=pil_image,    
                        size=(360, 400)
                        )
        self.pipeline.stop()


########################################### Node main function ###########################################
######################################################################################

def main(args=None):
    rclpy.init(args=args)
    
    # Create a subscriber node
    jarvis = ergo_gui()

    # Spin to keep the node running and responsive to callbacks
    rclpy.spin(jarvis)

    # Shutdown after exiting
    jarvis.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
