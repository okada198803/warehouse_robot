# ROS 2 Humble + Gazebo Fortress + Nav2 + rosbridge (arm64 / Apple Silicon 対応)
# ベース: ブラウザ(noVNC)でデスクトップを表示できるイメージ
FROM tiryoh/ros2-desktop-vnc:humble

USER root
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-gz \
      ros-humble-ros-ign-bridge \
      ros-humble-ros-ign-gazebo \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-nav2-simple-commander \
      ros-humble-slam-toolbox \
      ros-humble-rosbridge-server \
      ros-humble-robot-state-publisher \
      ros-humble-xacro \
      ros-humble-tf2-ros \
      python3-colcon-common-extensions \
      xterm \
    && rm -rf /var/lib/apt/lists/*

# ソフトウェアレンダリング(GPU なし環境)向け設定
ENV LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1

# ログイン時に ROS 環境とワークスペースを読み込む
RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc && \
    echo '[ -f /home/ubuntu/ros2_ws/install/setup.bash ] && source /home/ubuntu/ros2_ws/install/setup.bash' >> /etc/bash.bashrc
