<p align="center">
  <a href="https://www.industry40lab.org/"><img src="materials/polimi_logo.png" alt="Image 1" width="200"/></a>
  <a href="https://arise-middleware.eu/"><img src="materials/Screenshot%20from%202024-12-03%2011-00-00.png" alt="Image 2" width="200"/></a>
</p>

<h1 style="display: flex; align-items: center; justify-content: space-between;">
    ErgoBot_AI: LLM-Enhanced Human-Robot Interaction for Optimized Ergonomic Posture
</h1>

## Description:

This project is the second use case of <a href="https://arise-middleware.eu/">ARISE</a> project, This work consists of two modules (i) the ergonomics assessment of operator using the ROS4HRI capable of 3D pose estimation of the human body, (ii) the speech command to the robot using LLM. The modules are described in the following.

<ul>
  <li>Moveit2 based UR5e controller interface</li>
  <li>LLM (Ollama) ROS2 implementation for intent classification</li>
  <li>Voice Transcriber based on (VOSK)</li>
  <li>Active human posture tracker</li>
  <li>Graphical User Interface to communicate with operator</li>
</ul>

<hr>

### UR5e Moveit2 controller

You can find the code related to this module in the package of <a href="https://github.com/Industry40Lab/ErgoBot_AI/tree/main/arm_controller">arm_controller</a>, where you can see the c++ moveit2 interface associated with moving the universal robot (UR5e in our case) in ROS2. In this package, the <a href="https://github.com/Industry40Lab/ErgoBot_AI/blob/main/arm_controller/src/holding_controller.cpp">file</a> contains the interface and in this code the topics intended to move the robot is analysed and the needed movement will be executed. 
