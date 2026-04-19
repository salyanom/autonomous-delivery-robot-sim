#!/bin/bash
set -e


apt-get update
apt-get upgrade -y

rosdep update


cd ~/ros2_ws/src
git clone --recursive https://github.com/AuTURBO/RDSim.git
cd ~/ros2_ws/src/RDSim/
git submodule update --remote


apt-get install -y libzmq3-dev libboost-dev
cd ~/ros2_ws/src/RDSim/rdsim_submodules/BehaviorTree.CPP
mkdir build
cd build
cmake ..
make
sudo make install

cd ~/ros2_ws
rosdep install --ignore-src --rosdistro humble --from-paths ./src/RDSim/rdsim_submodules/navigation2 -y

