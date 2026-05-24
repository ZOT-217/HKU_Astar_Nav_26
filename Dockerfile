FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

# ── System dependencies ─────────────────────────────────────────────
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    # PCL / Eigen / OpenCV
    libpcl-dev \
    libeigen3-dev \
    libopencv-dev \
    # g2o (graph optimization for hdl_graph_slam)
    libg2o-dev \
    # OpenMP (Point-LIO, hdl_graph_slam)
    libomp-dev \
    # Python deps (hdl_graph_slam scripts)
    python3-scipy \
    python3-progressbar \
    python3-pip \
    # Misc
    wget \
    git \
    cmake \
    build-essential \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ── Livox SDK (required by livox_ros_driver2) ────────────────────────
WORKDIR /tmp
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git && \
    cd Livox-SDK && \
    mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && \
    rm -rf /tmp/Livox-SDK

# ── ROS workspace skeleton ───────────────────────────────────────────
RUN mkdir -p /workspace/src
WORKDIR /workspace

# Source ROS and create a bashrc snippet for convenience
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> /root/.bashrc

# Entrypoint: start a bash shell that sources ROS automatically
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
