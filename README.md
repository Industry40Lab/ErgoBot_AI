<p align="center">
  <a href="https://www.industry40lab.org/"><img src="materials/polimi_logo.png" alt="Polimi Logo" width="200"/></a>
  <a href="https://arise-middleware.eu/"><img src="materials/Screenshot%20from%202024-12-03%2011-00-00.png" alt="ARISE Screenshot" width="200"/></a>
</p>

<h1 style="display: flex; align-items: center; justify-content: space-between;">
    ErgoBot_AI: LLM-Enhanced Human-Robot Interaction for Optimized Ergonomic Posture
</h1>

<a href="https://ubuntu.com/blog/tag/ubuntu-24-04-lts">![Static Badge](https://img.shields.io/badge/Ubuntu-24.04-orange)</a>
<a href="https://www.python.org/downloads/release/python-3100/">![Static Badge](https://img.shields.io/badge/Python-3.10-blue)</a>
<a href="https://docs.ros.org/en/humble/index.html">![Static Badge](https://img.shields.io/badge/ROS2-humble-blue)</a>
<a href="https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html">![Static Badge](https://img.shields.io/badge/ROS2-Jazzy-blue)</a>

# Description:

<p align="center">
  <img src="materials/M24_archpng.png" alt="Overall Arch"/>
</p>

This project is the second use case of the <a href="https://arise-middleware.eu/">ARISE</a> project. The system integrates three primary functionalities: operator posture assessment, LLM-based human-robot interaction, and data recording/visualization. It uses three cameras (Intel RealSense) to monitor the operator, extracts body landmarks using AlphaPose from three angles(Front, Left, and Right), calculates the Rapid Upper Limb Assessment (RULA) score, and presents this information along with camera feeds to the operator via a Graphical User Interface (GUI). The GUI also enables voice command input, which is transcribed by the Whisper model, processed by an LLM (Llama3) within the RPK framework, and then used to control a UR5e robot via MoveIt2. RULA data is communicated via NGSI-LD to Arise middleware for historical data recording and visualization in Grafana.

<hr>

# Core Components

## Operator Monitoring and RULA Calculation

* **<a href="/Ros_src/point_2D_extractor/point_2D_extractor/point_2D.py">AlphaPose Model:</a>** This model is responsible for extracting and tracking **2D body landmarks** of the operator from the camera feeds. Please follow the instruction to install the Alphapose from its <a href="https://github.com/MVIG-SJTU/AlphaPose">**official website**</a>.
* **Cameras:** Three **Intel RealSense cameras** are used to capture the operator's movements and posture from different angles. To setup the use of **AlphaPose** in **ROS2** with **Intel RealSense**, please replace `./alphapose/detector/yolox_api.py` with the installed AlphaPose `yolox_api.py` in the detector folder, and also replace the content in `alphapose/utils` with equivalent files in the installed AlphaPose utils.

* **<a href="/ergobot_poc/build/ros2/rula_calculator/rula_calculator/rula_calculator.py">RULA Calculator:</a>** This component calculates the **Rapid Upper Limb Assessment (RULA) score** based on the extracted body landmarks. This calculation is performed within the **Vulcanexus Docker environment**. 
* **<a href="/ergobot_poc/build/ros2/rula_gui/rula_gui/rulaGui.py">GUI:</a>** This component is responsible for indicating the **camera feeds**, **LLM output**, activating the **voice command**, and indicating the **RULA and operator's body information output**. This component also runs within the **Vulcanexus Docker environment**.
* **<a href="/ergobot_poc/conf/orionld/config-dds.json">NGSI-LD Communication:</a>** RULA information from the RULA Calculator is communicated using the **NGSI-LD standard**. This allows the data to be recorded by the **Arise middleware**.

## Operator Interface (GUI)
The GUI provides the operator with real-time feedback and control. It is implemented within a Vulcanexus Docker environment.
* **Visual Feedback:** Displays live camera feeds, visual representations of the extracted **body landmarks**, and the calculated **RULA score**.
* **Command Input:** **Voice command activation button**, which sends activation of transcribed voice commands for the LLM processing.
* **Audio Feedback:** **Vocal output** from the LLM is provided via a **<a href="/Ros_src/tts_system/tts_system/tts_engine.py">Text-to-Speech (TTS) sound engine</a>**.

## Robot Control

* **<a href="/Ros_src/arise_rpk/">LLM (Llama3) in RPK Framework:</a>** This **large language model** processes the operator's transcribed voice commands, understands their intent, and generates appropriate robot commands. This operates within the **RPK framework**.
* **<a href="/Ros_src/voice_transcriber/voice_transcriber/voice_transcriber.py">Voice Transcriber (Whisper Model):</a>** This component transcribes the operator's **voice commands into text**, which are then sent to the LLM.
* **<a href="/Ros_src/ur_controler/">MoveIt2:</a>** A robotic manipulation platform used for **motion planning**, **inverse kinematics**, and **collision checking**. It receives commands from the LLM and calculates trajectories for the **UR5e robot**.
* **<a href="/Ros_src/ur_controler/">UR5e Driver:</a>** Interfaces with the **UR5e robot**, receiving trajectories from MoveIt2 and executing the movements.

## 📊 Data Management and Visualization

* **Arise Middleware:** Records the **RULA information** communicated via **NGSI-LD**, providing **historical data storage**.
* **<a href="/ergobot_poc/conf/grafana/dashboards/Rula Information-1760105669922.json">Grafana Dashboard:</a>** Visualizes the **historical RULA data**, allowing for **analysis and monitoring** of operator posture over time.

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



