FROM eprosima/vulcanexus:jazzy-desktop 

RUN mkdir -p /ros2-ws/src
WORKDIR /ros2-ws/src

RUN apt update && \
    apt install -y x11-apps

# Update system and install required dependencies
RUN apt update && apt install -y \
    git wget curl gnupg2 lsb-release \
    build-essential cmake pkg-config \
    libusb-1.0-0-dev libgtk-3-dev \
    libglfw3-dev libgl1-mesa-dev \
    libssl-dev \
    python3-pip python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Clone and build librealsense
# WORKDIR /opt
# RUN git clone https://github.com/IntelRealSense/librealsense.git && \
#     cd librealsense && \
#     mkdir build && cd build && \
#     cmake .. -DCMAKE_BUILD_TYPE=Release \
#              -DBUILD_EXAMPLES=false \
#              -DFORCE_LIBUVC=true && \
#     make -j$(nproc) && \
#     make install

# RUN curl -fsSL https://ollama.com/install.sh | sh
# RUN ollama serve
# RUN ollama pull llama3.2
RUN pip install colorama --break-system-packages
RUN pip install customtkinter --break-system-packages
# RUN pip install torch torchvision --extra-index-url https://download.pytorch.org/whl/cu113 --break-system-packages

# COPY ./arise_rpk /ros2-ws/src/arise_rpk
COPY ./hri_actions_msgs /ros2-ws/src/hri_actions_msgs
# COPY ./point_2D_extractor /ros2-ws/src/point_2D_extractor
COPY ./rula_calculator /ros2-ws/src/rula_calculator
COPY ./rula_gui /ros2-ws/src/rula_gui
COPY ./body_data /ros2-ws/src/body_data
# COPY ./tts_system /ros2-ws/src/tts_system
# COPY ./voice_transcriber /ros2-ws/src/voice_transcriber
# COPY ./AlphaPose /AlphaPose
COPY ./hri_msgs /ros2-ws/src/hri_msgs
# FROM pytorch/pytorch:2.2.0-cuda11.8-cudnn8-runtime

RUN git clone --depth=1  https://github.com/eProsima/vulcanexus.git tmp_dir
    
RUN mv tmp_dir/code/turtlesim .
RUN rm -rf tmp_dir
WORKDIR /ros2-ws
RUN source /opt/vulcanexus/jazzy/setup.bash && \
    colcon build

RUN export ROS_DOMAIN_ID=0
RUN source /ros2-ws/install/setup.bash
# # Alphapose installation #############################
# RUN apt update && apt install -y libyaml-dev

# # Set environment variables for CUDA
# ENV PATH=/usr/local/cuda/bin:$PATH
# ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# # Install Python dependencies
# RUN pip install cython --break-system-packages

# # Build AlphaPose in development mode
# WORKDIR /AlphaPose
# RUN python3 setup.py build develop --user


# ENTRYPOINT ["/bin/bash", "-c", "sleep infinity"] 
CMD ["bash", "-c",  "source install/setup.bash && export ROS_DOMAIN_ID=0 && ros2 launch rula_gui rula_run.launch.py"]
