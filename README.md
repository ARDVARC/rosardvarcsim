# rosardvarcsim

## Introduction
This is a repository for the `rosardvarcsim` ROS package. The `rosardvarcsim` package contains specific configuration information and utility scripts to run the
ARDVARC Gazebo+PX4 simulator.

## Usage
See **Setup** below first. Once set up, run the simulation with:
```
roslaunch rosardvarcsim main_sim.launch
```

To not load the Gazebo GUI (which slows things down quite a bit), run:
```
roslaunch rosardvarcsim main_sim.launch gui:=false
```
The current incantation is as follows (this launches the sim make sure all mains are checked out and current and re `catkin_make`)(updated 4/19/24)
```
roslaunch rosardvarcsim sim_and_fsw.launch fake_bluetooth_angles:=false fake_bluetooth_pointing_vectors:=true
```
## Setup

If you're using WSL I'd recommend following [these instructions to set up your graphics card to work with WSL](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps). It should make Gazebo run a lot faster.

Next add libgstreamer stuff for the camera:
```
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio -y
```

Next this package requires a local clone and build of the PX4 repo:
```
cd path/to/wherever/you/want \
&& git clone https://github.com/PX4/PX4-Autopilot.git --recursive \
&& bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx \
&& cd PX4-Autopilot \
&& DONT_RUN=1 make px4_sitl_default gazebo-classic \
&& DONT_RUN=1 make px4_sitl gazebo-classic_typhoon_h480 \
&& echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$(pwd)" >> ~/.bashrc \
&& echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic" >> ~/.bashrc \
&& echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$(pwd)/build/px4_sitl_default/build_gazebo-classic" >> ~/.bashrc \
&& echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models" >> ~/.bashrc \
&& echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/build/px4_sitl_default/build_gazebo-classic" >> ~/.bashrc \
&& source ~/catkin_ws/devel/setup.bash \
&& exec bash
```

As well as `mavros`:
```
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs \
&& wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
&& sudo bash ./install_geographiclib_datasets.sh \
&& rm install_geographiclib_datasets.sh
```

Then you'll need to clone this repo into your catkin workspace and build it:
```
cd ~/catkin_ws/src \
&& git clone https://github.com/ARDVARC/rosardvarcsim.git \
&& cd .. \
&& catkin_make
```

You'll need `scipy`:
```
pip install scipy
```

Finally add our model paths to your `.bashrc` by running:
```
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/rosardvarcsim/models" >> ~/.bashrc \
&& exec bash
```
