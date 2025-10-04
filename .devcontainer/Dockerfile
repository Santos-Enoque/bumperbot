FROM ros:jazzy

ARG USERNAME=ubuntu

# Don't ask for sudo password
RUN echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME
sudo apt install -y build-essential git tmux ros-dev-tools bsdmainutils pulseaudio alsa-utils ros-jazzy-rmw-zenoh-cpp psmisc nodejs npm
# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video,dialout $USERNAME

# Install system dependencies including Zenoh RMW
RUN sudo apt update && \
    sudo apt upgrade -y && \
    sudo apt install -y build-essential git tmux ros-dev-tools bsdmainutils pulseaudio alsa-utils ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-rmw-zenoh-cpp psmisc nodejs npm

# Setup environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> ~/.bashrc && \
    echo "pgrep -x rmw_zenohd >/dev/null || (ros2 run rmw_zenoh_cpp rmw_zenohd &)" >> ~/.bashrc

ARG WORKSPACE_NAME=
RUN if [ -d "/${WORKSPACE_NAME}/install" ]; then \
    echo "source /${WORKSPACE_NAME}/install/setup.bash" >> ~/.bashrc; \
    fi

# Setup colcon and Git autocompletion
RUN echo "source /usr/share/colcon_cd/function/colcon_cd-argcomplete.bash" >> ~/.bashrc && \
    echo "source /usr/share/bash-completion/completions/git" >> ~/.bashrc



# Tmux enhancements
RUN echo "set -g base-index 1\n\
    set -g pane-base-index 1\n\
    set -g status-style bg=default\n\
    bind H resize-pane -L 5\n\
    bind J resize-pane -D 5\n\
    bind K resize-pane -U 5\n\
    bind L resize-pane -R 5\n\
    bind h select-pane -L\n\
    bind l select-pane -R\n\
    bind k select-pane -U\n\
    bind j select-pane -D" >> ~/.tmux.conf
