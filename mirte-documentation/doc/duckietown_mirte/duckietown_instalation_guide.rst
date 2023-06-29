==============================
Duckietown  Installation Guide
==============================

+++++++++++++++++++++++++++++++++
Install Mirte Duckietown messages
+++++++++++++++++++++++++++++++++

In this first step you will install the custom `mirte_duckietown_msgs` that are used for communication from and to ROS.

.. code-block:: shell-session

    # copy the messages over to Mirte
    you@your_machine:~$ scp path/to/mirte_duckietown_msgs mirte@mirte:~

    # copy the messages to the right directories
    mirte@mirte:~$ sudo cp mirte_duckietown_msgs /opt/ros/noetic/share -r
    mirte@mirte:~$ cp mirte_duckietown_msgs ~/mirte_ws/src -r

    # remove message folder as it is not needed here anymore
    mirte@mirte:~$ rm -rf mirte_duckietown_msgs

    # remove old build and make all packages again
    mirte@mirte:~/mirte_ws$ rm -rf build devel
    mirte@mirte:~/mirte_ws$ catkin_make

+++++++++++++
Upload binary
+++++++++++++

In this step you will copy the binary that creates a ROS node and publishes to it.

.. code-block:: shell-session

    # copy binary to home folder of Mirte
    you@your_machine:~$ scp path/to/mirte-rs mirte@mirte:~

++++++++++++++++++
Configure AprilTag
++++++++++++++++++

In this step you will install and configure Apriltags that are used for detecting the traffic signs from duckietown.

.. code-block:: shell-session

    # update current packages and install Apriltags on Mirte
    mirte@mirte:~$ sudo apt-get update
    mirte@mirte:~$ sudo apt install ros-noetic-apriltag-ros -y

    # copy all neccesary launch and config files over to Mirte
    you@your_machine:~$ scp path/to/continuous_detection.launch mirte@mirte:~
    you@your_machine:~$ scp path/to/settings.yaml mirte@mirte:~
    you@your_machine:~$ scp path/to/tags.yaml mirte@mirte:~

    # move the files into the right directories
    mirte@mirte:~$ sudo mv continuous_detection.launch /opt/ros/noetic/share/apriltag_ros/launch/continuous_detection.launch
    mirte@mirte:~$ sudo mv settings.yaml /opt/ros/noetic/share/apriltag_ros/config/settings.yaml
    mirte@mirte:~$ sudo mv tags.yaml /opt/ros/noetic/share/apriltag_ros/config/tags.yaml

++++++++++++++++++++++
Install python package
++++++++++++++++++++++

In this step you will install the python package that contains the API for controlling Mirte in the Duckietown environment.

.. code-block:: shell-session

    # copy over source code
    you@your_machine:~$ scp -r path/to/python/folder mirte@mirte:~

    # install build tool
    mirte@mirte:~$ python3 -m pip install --upgrade build

    # build the python package
    mirte@mirte:~/python$ python3 -m build

    # update numpy if neccesary
    mirte@mirte:~/python$ pip install numpy --upgrade

    # uninstall old version and install new one
    mirte@mirte:~/python$ pip3 uninstall mirte -y
    mirte@mirte:~/python$ pip3 install dist/mirte-1.0-py3-none-any.whl

    # remove the source code as it is not needed here anymore
    mirte@mirte:~$ rm -rf python

++++++++++++++++++++
Update web interface
++++++++++++++++++++

Use instructions from `here <../mirte_development.html>`_ to update the web interface to contain blocks for Duckietown

++++++++++++++++++++++
Let Mirte drive around
++++++++++++++++++++++

Launching all ROS nodes neccerary for letting Mirte drive around in Duckietown

.. code-block:: shell-session

    # start ROS node for computer vision
    mirte@mirte:~$ ./mirte-rs
    # launch ROS node for detection april tags
    roslaunch apriltag_ros continuous_detection.launch
