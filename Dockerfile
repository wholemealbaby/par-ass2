# 1. Start from the existing workspace image to save time/bandwidth
FROM rmit/aiil_workspace:latest

SHELL ["/bin/bash", "-c"]

# 2. Layer on the specific transformation dependencies
USER root
RUN apt update && apt install -y \
    # ROS-specific transformations logic
    ros-jazzy-tf-transformations \
    # Python-specific geometry libraries
    #python3-transforms3d \
    python3-pip \
    && apt clean && rm -rf /var/lib/apt/lists/*

# 3. Ensure the Python transformations3d is available to the ROS environment
RUN pip install --no-cache-dir transforms3d --break-system-packages

# 4. Install direnv
RUN apt update && apt install -y direnv \
    && apt clean && rm -rf /var/lib/apt/lists/*

# 5. Add direnv hook to bashrc
RUN echo 'eval "$(direnv hook bash)"' >> /root/.bashrc

WORKDIR /ros2_ws

# The environment remains identical to the base, just with these tools added