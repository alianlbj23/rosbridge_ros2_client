FROM ros:humble-ros-core-jammy

# 1. Install pip3, colcon, bash-completion and roslibpy
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3-pip \
      python3-colcon-common-extensions \
      bash-completion && \
    pip3 install roslibpy && \
    rm -rf /var/lib/apt/lists/*

# 2. Create and set workspace as the default working directory
WORKDIR /workspace
RUN mkdir -p src

# 3. Enable ROS 2 & colcon auto-completion and auto-sourcing in every new shell
RUN echo "source /opt/ros/humble/setup.bash"       >> /etc/bash.bashrc && \
    echo "source /usr/share/bash-completion/completions/colcon" >> /etc/bash.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" \
                                                     >> /etc/bash.bashrc

# 4. Use bash -lc so that ENTRYPOINT/CMD shells pick up /etc/bash.bashrc
SHELL ["/bin/bash", "-lc"]

# 5. By default, launch an interactive bash in /workspace
CMD ["bash"]
