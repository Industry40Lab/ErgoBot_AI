<p align="center">
  <a href="https://www.industry40lab.org/"><img src="materials/polimi_logo.png" alt="Image 1" width="200"/></a>
  <a href="https://arise-middleware.eu/"><img src="materials/Screenshot%20from%202024-12-03%2011-00-00.png" alt="Image 2" width="200"/></a>
</p>

<h1 style="display: flex; align-items: center; justify-content: space-between;">
    ErgoBot_AI: LLM-Enhanced Human-Robot Interaction for Optimized Ergonomic Posture
</h1>

![Static Badge](https://img.shields.io/badge/Ubuntu-22.04-orange)
![Static Badge](https://img.shields.io/badge/Python-3.10-blue)
![Static Badge](https://img.shields.io/badge/ROS2-iron-blue)
![Static Badge](https://img.shields.io/badge/ROS2-humble-blue)

## Description:

<p align="center">
  <img src="materials/diagram2.png" alt="overall diagram"/>
</p>

This project is the second use case of <a href="https://arise-middleware.eu/">ARISE</a> project, This work consists of two modules (i) the ergonomics assessment of operator using the ROS4HRI capable of 3D pose estimation of the human body, (ii) the speech command to the robot using LLM. The modules are described in the following.

<ul>
  <li>Moveit2 based UR5e controller interface</li>
  <li>LLM (Ollama) ROS2 implementation for intent classification</li>
  <li>Voice Transcriber based on (VOSK)</li>
  <li>Active human posture tracker</li>
  <li>Graphical User Interface to communicate with operator</li>
</ul>

In summary, for the first scale-up of the ARISE project in this work, the LLM model has been implemented in the ROS2 framework to facilitate human/robot interaction. The operator sends a vocal command to the robot through a microphone, then the speech signal is transcribed using the VOSK API. Consequently, the transcribed command is forwarded to the Ollama 3.2 model through a ROS2 topic for the identification of the operator’s intention(s). As a result, the intention(s) is translated to low-level robot commands for the trajectory planning via MoveIt2. 

<hr>

### UR5e Moveit2 controller

You can find the code related to this module in the package of <a href="https://github.com/Industry40Lab/ErgoBot_AI/tree/main/arm_controller">arm_controller</a>, where you can see the c++ moveit2 interface associated with moving the universal robot (UR5e in our case) in ROS2. In this package, the <a href="https://github.com/Industry40Lab/ErgoBot_AI/blob/main/arm_controller/src/holding_controller.cpp">file</a> contains the interface and in this code, the topics intended to move the robot is analysed and the needed movement will be executed. 


<hr>

### LLM implementation in ROS2

In this project, the LLM implemented to classify the operator intent and to ease the communication between the oprator and the robot. The package related to this task is the <a href="https://github.com/Industry40Lab/ErgoBot_AI/tree/main/llm_communicator">llm_communicator</a>. To implement this part of the project we used, we used <a href="https://github.com/ollama/ollama/tree/main
">**OLLAMA3.2**</a> model.<br> 

This package has two main objectives:

<ol>
  <li>Notifing the oprator in the case of any notification is needed to communicate to the oprator, such as adjusting the posture.</li>
  <li>classifying the intended action of the oprator for robot movement.</li>
</ol>

For the second objective the LLM need to classify the user intent from the list of provided possible intent at the package in the _resource
/robot_commands.txt_. These intents are considered as the system prompt to the LLM model. 

<hr>

### Voice Transcriber

To communicate efficiently with the robot or more especificly with the LLM model of the robot, a voice transcriber is implemented using the <a href="https://github.com/alphacep/vosk-api
">**VOSK API**</a>. Therefore in the package <a href="https://github.com/Industry40Lab/ErgoBot_AI/tree/main/voice_command/voice_command
">**voice_command**</a>, you can find the related code. 



