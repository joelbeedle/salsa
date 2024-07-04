FROM    ubuntu:22.04

# for the VNC connection
EXPOSE 5900
# for the browser VNC client
EXPOSE 5901
# Use environment variable to allow custom VNC passwords
ENV VNC_PASSWD=123456

# Make sure the dependencies are met
ENV APT_INSTALL_PRE="apt -o Acquire::ForceIPv4=true update && DEBIAN_FRONTEND=noninteractive apt -o Acquire::ForceIPv4=true install -y --no-install-recommends"
ENV APT_INSTALL_POST="&& apt clean -y && rm -rf /var/lib/apt/lists/*"
# Make sure the dependencies are met
RUN eval ${APT_INSTALL_PRE} python3-dev gdb pkg-config libspdlog-dev wayland-protocols python3-pip libwayland-dev ninja-build tigervnc-standalone-server tigervnc-common tigervnc-tools fluxbox eterm cmake g++ gcc xterm git net-tools python3 python3-numpy ca-certificates scrot libx11-dev libxrandr-dev libxcursor-dev libudev-dev libopenal-dev libflac-dev libogg-dev libvorbis-dev libopenal-dev libpthread-stubs0-dev libxinerama-dev libglfw3 libglfw3-dev libxi-dev libxkbcommon-dev ${APT_INSTALL_POST}

# Install VNC. Requires net-tools, python and python-numpy
RUN git clone --branch v1.4.0 --single-branch https://github.com/novnc/noVNC.git /opt/noVNC
RUN git clone --branch v0.11.0 --single-branch https://github.com/novnc/websockify.git /opt/noVNC/utils/websockify
RUN ln -s /opt/noVNC/vnc.html /opt/noVNC/index.html

# Install pandas, matplotlib, seaborn
RUN pip3 install pandas matplotlib seaborn scipy

# Add menu entries to the container
RUN echo "?package(bash):needs=\"X11\" section=\"DockerCustom\" title=\"Xterm\" command=\"xterm -ls -bg black -fg white\"" >> /usr/share/menu/custom-docker && update-menus

# Set timezone to UTC
RUN ln -snf /usr/share/zoneinfo/UTC /etc/localtime && echo UTC > /etc/timezone

# Add in a health status
HEALTHCHECK --start-period=10s CMD bash -c "if [ \"`pidof -x Xtigervnc | wc -l`\" == "1" ]; then exit 0; else exit 1; fi"

# Add in non-root user
ENV UID_OF_DOCKERUSER 1000
RUN useradd -m -s /bin/bash -g users -u ${UID_OF_DOCKERUSER} dockerUser
RUN chown -R dockerUser:users /home/dockerUser && chown dockerUser:users /opt

USER dockerUser

# Copy various files to their respective places
COPY --chown=dockerUser:users container_startup.sh /opt/container_startup.sh
COPY --chown=dockerUser:users x11vnc_entrypoint.sh /opt/x11vnc_entrypoint.sh
# Subsequent images can put their scripts to run at startup here
RUN mkdir /opt/startup_scripts

USER root

WORKDIR /root/testbed

COPY . .

RUN git submodule update --init --recursive
RUN rm -rf build && mkdir build
RUN cmake -S . -B build -GNinja
RUN cmake --build build
# USER dockerUser
# WORKDIR /

ENTRYPOINT ["/opt/container_startup.sh"]
