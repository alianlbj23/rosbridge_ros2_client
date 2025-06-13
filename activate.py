#!/usr/bin/env python3
"""
run_container.py

這個腳本會啟動名為 ros_lib_py:0.0.0 的 Docker 映像，
並將當前目錄下的 ros2_ws 資料夾掛載到容器內的 /workspace/ros2_ws。
"""

import os
import subprocess
import sys

def main():
    # 取得腳本所在目錄
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # 本地 ros2_ws 路徑
    host_ws = os.path.join(script_dir, "ros2_ws")
    if not os.path.isdir(host_ws):
        print(f"錯誤：找不到 ros2_ws 資料夾 ({host_ws})", file=sys.stderr)
        sys.exit(1)

    # 容器內掛載點
    container_ws = "/ros2_ws"

    # 組裝 docker run 指令
    cmd = [
        "docker", "run", "--rm", "-it",
        "-v", f"{host_ws}:{container_ws}",
        "-p", "9090:9090",
        "ros_lib_py:0.0.0"
    ]

    print("執行指令：", " ".join(cmd))
    # 執行容器
    ret = subprocess.run(cmd)
    sys.exit(ret.returncode)

if __name__ == "__main__":
    main()

