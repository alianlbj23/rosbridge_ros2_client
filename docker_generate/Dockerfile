FROM ros:humble-ros-core-jammy

# 安裝 pip3 並安裝 roslibpy
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3-pip && \
    pip3 install roslibpy && \
    rm -rf /var/lib/apt/lists/*

# （以下可按需加入其他套件、workspace 之類的設定）
