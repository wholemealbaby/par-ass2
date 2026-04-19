# 1. Start from the existing workspace image to save time/bandwidth
FROM rmit/aiil_workspace:latest

SHELL ["/bin/bash", "-c"]

# 2. Layer on the specific transformation dependencies
USER root


# 4. Install direnv
RUN apt update && apt install -y direnv \
    && apt clean && rm -rf /var/lib/apt/lists/*

# 5. Add direnv hook to bashrc
RUN echo 'eval "$(direnv hook bash)"' >> /root/.bashrc

WORKDIR /ros2_ws

# The environment remains identical to the base, just with these tools added