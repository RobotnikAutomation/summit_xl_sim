FROM osrf/ros:melodic-desktop-full
MAINTAINER Guillem Gari <ggari@robontik.es>

# Non Root user
ARG user_name=ros
ARG user_uid=1000
ARG user_home=/home/$user_name
ARG user_shell=/bin/bash
ARG ck_dir=$user_home/catkin_ws
ARG ck_src_dir=$ck_dir/src
ARG ros_brup_pkg=rostful_bringup

RUN useradd -m -d $user_home -s $user_shell -u $user_uid $user_name \
	&& echo "PS1='\[\e]0;\u@\h: \w\a\]${debian_chroot:+($debian_chroot)}\[\033[01;33m\]\u\[\033[00m\]@\[\033[01;31m\]\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ~/.bashrc

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
	&& apt-get install -q -y \
		wget \
		apt-utils \
		# dialog \
		sudo \
	&& apt-get clean -q -y \
	&& apt-get autoremove -q -y \
	&& rm -rf /var/lib/apt/lists/* \
	&& echo '%ros ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

RUN apt-get update \
	&& apt upgrade -y \
	&& apt-get clean -q -y \
	&& apt-get autoremove -q -y \
	&& rm -rf /var/lib/apt/lists/*

RUN apt-get update \
	&& apt-get install -q -y \
		python3-vcstool \
	&& apt-get clean -q -y \
	&& apt-get autoremove -q -y \
	&& rm -rf /var/lib/apt/lists/*

USER $user_name

RUN mkdir -p $ck_src_dir
WORKDIR $ck_dir

WORKDIR $ck_dir

COPY --chown=$user_name \
	repos/summit_xl_sim_devel.repos \
	/tmp

RUN true \
	&& vcs import --input /tmp/summit_xl_sim_devel.repos \
	&& true

RUN  true \
	&& rosdep update \
	&& echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections \
	&& sudo apt-get update \
	&& rosdep install --from-paths src --ignore-src -y \
	&& sudo apt-get clean -q -y \
	&& sudo apt-get autoremove -q -y \
	&& sudo rm -rf /var/lib/apt/lists/*