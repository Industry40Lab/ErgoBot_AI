import tkinter
import customtkinter
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import ollama
from colorama import Fore
import time

class cobot_llm(Node):


    def __init__(self):
        super().__init__('cobot_llm')

        # main prompt setting up 
        f = open("/home/i40lab/arise_usecase2_ws/src/llm_communicator/resource/robot_commands.txt", "r")
        self.system_prompt = f.read()
        
        self.publisher_ = self.create_publisher(String, '/robot_listening', 10)
        self.publisher_command_ = self.create_publisher(String, '/user_order', 10)
        self.publisher_notif_ = self.create_publisher(String, '/notif_llm', 10)
        self.publisher_llm_answer_ = self.create_publisher(String, '/llm_answer', 10)

        # Create a subscription to the /received_command topic
        self.subscription_user_order = self.create_subscription(
            String,
            '/recieved_order',
            self.user_order_callback,
            10  # QoS history depth
        )

        # Create a subscription to the /received_command topic
        self.subscription_user_order = self.create_subscription(
            String,
            '/recieved_notif',
            self.recieved_notif,
            10  # QoS history depth
        )
        # self.subscription  # prevent unused variable warning
        self.introduction()

    def recieved_notif(self, msg):
        stream = ollama.chat(
        model='llama3.2',
        messages=[ {
            'role': 'system',
            'content': 'You are an AI assistant your duty is to communicate and notify the oprator base on user command. Do not mention you are LLM mode. your name is cobot one. Give short answers unless user ask you for long one, base on recieved order by the user provide a notification to the oprator. your audince for your answer is oprator so do not mention user in your answers.',
            },
            {'role': 'user', 
            'content': msg.data}
            ],
        )
        msg = String()
        msg.data = stream['message']['content']

        self.publisher_notif_.publish(msg)

    def introduction(self):
        stream = ollama.chat(
        model='llama3.2',
        messages=[ {
            'role': 'system',
            'content': 'You are an AI assistant. Do not mention you are LLM mode. your name is cobot one. Give me short answers unless I ask you for long one.',
            },
            {'role': 'user', 
            'content': 'Intoduce yourself and welcome me in a very short form.'}
            ],
        )
        msg = String()
        msg.data = stream['message']['content']

        self.publisher_notif_.publish(msg)


    def user_order_callback(self, msg):

        stream = ollama.chat(
        model='llama3.2',
        messages=[
            {
            'role': 'system',
            'content': self.system_prompt
            },
            {'role': 'user', 'content': msg.data}],
        )
        msg = String()
        msg.data = stream['message']['content']

        self.publisher_llm_answer_.publish(msg)










########################################### Node main function ###########################################
######################################################################################

def main(args=None):
    rclpy.init(args=args)
    
    # Create a subscriber node
    jarvis = cobot_llm()

    # Spin to keep the node running and responsive to callbacks
    rclpy.spin(jarvis)

    # Shutdown after exiting
    jarvis.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
