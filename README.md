<p align="center">
  <a href="https://www.industry40lab.org/"><img src="materials/polimi_logo.png" alt="Polimi Logo" width="200"/></a>
  <a href="https://arise-middleware.eu/"><img src="materials/Screenshot%20from%202024-12-03%2011-00-00.png" alt="ARISE Screenshot" width="200"/></a>
</p>

<h1 style="display: flex; align-items: center; justify-content: space-between;">
    ErgoBot_AI: LLM-Enhanced Human-Robot Interaction for Optimized Ergonomic Posture
</h1>

![Static Badge](https://img.shields.io/badge/Ubuntu-24.04-orange)
![Static Badge](https://img.shields.io/badge/Python-3.10-blue)
![Static Badge](https://img.shields.io/badge/ROS2-iron-blue)
![Static Badge](https://img.shields.io/badge/ROS2-humble-blue)
![Static Badge](https://img.shields.io/badge/ROS2-Jazzy-blue)

## Description:

<p align="center">
  <img src="materials/diagram2.png" alt="Overall Diagram"/>
</p>

This project is the second use case of the <a href="https://arise-middleware.eu/">ARISE</a> project. This work consists of two main sections: (i) the ergonomics assessment of an operator using ROS4HRI capable of 3D pose estimation of the human body, and (ii) speech commands to the robot using an LLM. The modules are described below:

<ul>
  <li>MoveIt2-based UR5e controller interface</li>
  <li>LLM (Ollama) ROS2 implementation for intent classification</li>
  <li>Voice transcriber based on VOSK</li>
  <li>Active human posture tracker</li>
  <li>Graphical User Interface for operator communication</li>
</ul>

In summary, this work represents the first scale-up of the ARISE project, where the LLM model has been implemented in the ROS2 framework to facilitate human-robot interaction. The operator sends a vocal command to the robot through a microphone. The speech signal is transcribed using the VOSK API. The transcribed command is then forwarded to the Ollama 3.2 model via a ROS2 topic for the identification of the operator’s intention(s). As a result, the identified intention(s) are translated into low-level robot commands for trajectory planning via MoveIt2. 

<hr>

#### UR5e MoveIt2 Controller

You can find the code for this module in the <a href="https://github.com/Industry40Lab/ErgoBot_AI/tree/main/arm_controller">**arm_controller**</a> package. The <a href="https://github.com/Industry40Lab/ErgoBot_AI/blob/main/arm_controller/src/holding_controller.cpp">`holding_controller.cpp`</a> file contains the MoveIt2 interface that processes topics for robot movement and executes the necessary motions. 

<hr>

#### LLM Implementation in ROS2

This project uses an LLM to classify operator intent and streamline communication between the operator and the robot. The package related to this task is the <a href="https://github.com/Industry40Lab/ErgoBot_AI/tree/main/llm_communicator">**llm_communicator**</a>. To implement this, we used the <a href="https://github.com/ollama/ollama/tree/main">**OLLAMA 3.2**</a> model.

This package has two main objectives:

<ol>
  <li>Notifying the operator in case of important alerts, such as posture adjustments.</li>
  <li>Classifying the operator's intended actions for robot movement.</li>
</ol>

For the second objective, the LLM classifies user intent from a predefined list of possible intents found in the `_resource/robot_commands.txt` file. These intents are provided as a system prompt to the LLM model. 

<hr>

#### Voice Transcriber

To enable efficient communication between the operator and the robot, particularly with the LLM model, a voice transcriber has been implemented using the <a href="https://github.com/alphacep/vosk-api">**VOSK API**</a>. The relevant code can be found in the <a href="https://github.com/Industry40Lab/ErgoBot_AI/tree/main/voice_command/voice_command">**voice_command**</a> package.

<hr>

#### Human Posture Tracker

To assess user posture, we currently use Mediapipe. The assessment follows the **Rapid Upper Limb Assessment (RULA)** method, which evaluates ergonomic risks related to upper extremity musculoskeletal disorders (MSDs). RULA considers the biomechanical and postural load requirements on the neck, trunk, and upper extremities. The <a href="https://github.com/Industry40Lab/ErgoBot_AI/tree/main/rula_assessment">**rula_assessment**</a> package contains all the necessary information for this task.

<hr>

### GUI

The graphical user interface for operator-robot communication is implemented in the <a href="https://github.com/Industry40Lab/ErgoBot_AI/tree/main/ergo_gui">**ergo_gui**</a> package.

<p align="center">
  <img src="materials/BAD_POSE.png" alt="GUI Screenshot"/>
</p>

<hr>

# Current Status and Plans

The project is still in development. We are working on the following improvements:

| Package | Current Status | Plans |
|---------|----------------|-------|
| <a href="https://github.com/Industry40Lab/ErgoBot_AI/tree/main/rula_assessment">**rula_assessment**</a> | Currently using <a href="https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker">**Mediapipe**</a> | Development of <a href="https://github.com/ros4hri">**ROS4HRI**</a> |
| <a href="https://github.com/Industry40Lab/ErgoBot_AI/tree/main/llm_communicator">**llm_communicator**</a> | Currently working with a self-implemented <a href="https://github.com/ollama/ollama/tree/main">**OLLAMA 3.2**</a> model | Implementation of RPK from PAL Robotics |

<hr>

# Run Instructions

To run the code, execute the following commands in sequence:

**Step 1: Start the camera and GUI**  
```bash
ros2 run ergo_gui ergo_gui
```

Step 2: Start the LLM communicator
Run this command to activate the "Send Command" button:

```bash
ros2 run llm_communicator cobot_llm
```
Step 3: Activate the voice command system
Set the VOSK model path in the code, build the package, and then run:

```bash
ros2 run voice_command voice_command_system
```

Step 4: Start the MoveIt Config, and UR5e controllers
Run these commands one by one:

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 launch_rviz:=false
```

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:="ur5e" launch_rviz:=false
```

Step 5: Activate arm control with the LLM model

```bash
ros2 launch arm_controller arm_handler.launch.py ur_type:="ur5e"
```



