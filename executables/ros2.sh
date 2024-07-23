#!/bin/bash

#---------- After executing $ sudo apt update && sudo apt upgrade

THISDIR=$(realpath `dirname $0`) #Get this directory


info() { #Function to print a message ">>>" in blue color
    echo ""
    echo -e "\e[34m>>>\e[0m ${@}"
}

echo -e "\n"
info "WARNING: IF YOU HAVE ANOTHER ROS DISTRO, COMMENT THE 'source /opt/ros/OTHER_ROS_DISTRO/setup.bash' LINE IN THE .bashrc file."
echo "Press Enter to continue ..."
read


cd $HOME
echo "[Set the ROS2 version and name of the ros2 workspace]"
name_ros_version='humble'
ros2_pkg='desktop'
name_workspace='colcon_ws'
info "\e[34mThe ROS_DOMAIN_ID setup will be 0. You can modify it later in the .bashrc file\e[0m"

info "ROS2 version: ${name_ros_version}"
info "ROS2 packages: ${ros2_pkg}"
info "ROS 2 workspace: $HOME/${name_workspace} \n"

echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION (or wait for 10 seconds)"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C] NOW"
read -t 10


echo "[Set Locale]"
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "[Setup Sources]"
sudo rm -rf /var/lib/apt/lists/* && sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null'

sudo apt update #----------------

info "[Installing ROS 2 packages] ..."
sleep 2
sudo apt install -y ros-$name_ros_version-$ros2_pkg
sleep 3

info "[Environment setup]"
sleep 3 #
source /opt/ros/$name_ros_version/setup.sh
sudo apt install -y python3-argcomplete python3-colcon-common-extensions python3-vcstool python3-rosdep2

info "[Make the $name_workspace and test colcon build]"
sleep 3 #
mkdir -p $HOME/$name_workspace/src
cd $HOME/$name_workspace
colcon build --symlink-install

rosdep update


info "[Set the ROS evironment and alias]"
sleep 3 #
sh -c "echo \"alias nb='nano ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias sb='source ~/.bashrc'\" >> ~/.bashrc"

sh -c "echo \"source /opt/ros/$name_ros_version/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source ~/$name_workspace/install/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash\" >> ~/.bashrc"

sh -c "echo \"export ROS_DOMAIN_ID=0\" >> ~/.bashrc"

#exec bash

info "The source bash lines were added to the .bash file\n"
sleep 3 #
echo -e "source /opt/ros/$name_ros_version/setup.bash"
echo -e "source ~/$name_workspace/install/setup.bash"
echo -e "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash \n"

cd $THIS_DIR
info "[ROS 2 Installation Complete!!!]"
 


