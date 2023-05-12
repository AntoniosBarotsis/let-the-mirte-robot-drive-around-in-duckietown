# ROS

This will contain all of the code needed to interact with ROS.

## Compiling

Make sure you have ROS installed (we are once again not using Windows).

The following should get you started

```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-ros-base -y
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Run the following to verify that the installation worked
rostopic list
```

## Running Outside of Mirte

- Get Mirte's IP
- Set `ROS_MASTER_URI=http://<mirte's ip>:11311`
- Run `rostopic info /webcam/image_raw` and get the Publisher's url (should look something like
  `http://Mirte-3E973C:43205/`)
- Add the hostname (`Mirte-3E973C` from the previous example) to your `/etc/hosts` file like so:
  `<mirte's ip><TAB><mirte's hostname>`
- Running `rostopic echo /webcam/image_raw` should now fill your terminal with numbers
