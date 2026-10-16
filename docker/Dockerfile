# PX4 SITL + Gazebo Harmonic, plus the Python tooling for this project.
#
# The base is PX4's official prebuilt simulation image, pinned by digest so a
# rebuild always gets the exact same PX4/Gazebo binaries. See DESIGN.md §2 for
# why this is v1.18.0-rc1 rather than v1.17.0.
FROM px4io/px4-sitl-gazebo:v1.18.0-rc1@sha256:aa4bbf35f0bd6bdb3269cc96e317c8e78b87f1fc769c734c0818094074e1cc0a

# python3-venv/pip: the image ships Python 3.12 but no pip.
# ffmpeg: turns saved camera frames into demo videos.
# libgl/libxkb/xcb libs + a font: OpenCV's bundled Qt needs them for
# cv2.imshow via WSLg (Qt no longer ships its own fonts).
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-venv python3-pip ffmpeg fonts-dejavu-core \
      libgl1 libglib2.0-0 libxkbcommon-x11-0 libxcb-icccm4 libxcb-image0 \
      libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 libxcb-shape0 \
      libxcb-xinerama0 libxcb-xkb1 libxcb-cursor0 libfontconfig1 libdbus-1-3 \
 && rm -rf /var/lib/apt/lists/*

# --system-site-packages keeps Gazebo's apt-installed Python bindings
# (gz.transport13, gz.msgs10) importable from inside the venv.
RUN python3 -m venv --system-site-packages /opt/venv
ENV PATH=/opt/venv/bin:$PATH

COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /workspace
ENV PYTHONPATH=/workspace \
    PYTHONUNBUFFERED=1 \
    QT_QPA_FONTDIR=/usr/share/fonts/truetype/dejavu

# The base image's entrypoint launches PX4. Clear it so the same image can
# also run our Python tools; compose.yaml sets the sim entrypoint explicitly.
ENTRYPOINT []
CMD ["bash"]
