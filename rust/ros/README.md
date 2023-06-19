# ROS

This will contain all of the code needed to interact with ROS.

## Compiling

Make sure you have ROS Noetic installed, the following should get you started:

```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-ros-base -y
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Run the following to verify that the installation worked
rostopic list
# It should print something like "ERROR: Unable to communicate with master!"
```

You also need to add a few custom ROS messages that Mirte uses:

```sh
mkdir -p ~/catkin_ws/src

# Location does not really matter, we will delete this right after
git clone https://github.com/AntoniosBarotsis/mirte-ros-packages
sudo cp ./mirte-ros-packages/mirte_msgs /opt/ros/noetic/share -r
cp ./mirte-ros-packages/mirte_msgs ~/catkin_ws/src -r
rm -rf ./mirte-ros-packages
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> The `catkin_make` command might fail if `python3` is not in your path for whatever reason in which 
> case, running `catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3` (or to whatever your path is)
> might help.

If all went well, you should be able to now run a `cargo build`.

## Running Outside of Mirte

It is also decently easy to run the code from your workstation and have it communicate with Mirte
instead of running it on Mirte itself. For this to work, your workstation and Mirte must be
connected to the same WiFi.

- Get Mirte's IP
- Set `ROS_MASTER_URI=http://<mirte's ip>:11311`
- Run `rostopic info /webcam/image_raw` and get the Publisher's url (should look something like
  `http://Mirte-3E973C:43205/`)
- Add the hostname (`Mirte-3E973C` from the previous example) to your `/etc/hosts` file like so:
  `<mirte's ip><TAB><mirte's hostname>`
- Running `rostopic echo /webcam/image_raw` should now fill your terminal with numbers
