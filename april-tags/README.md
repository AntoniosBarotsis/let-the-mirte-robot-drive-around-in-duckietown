# April Tag Configuration

This folder does not include any code but rather a few config files that are meant to be placed
in Mirte itself to get April tag detection to work.

## Set-Up

First, install the [April tags](http://wiki.ros.org/apriltag_ros) library **in Mirte**:

```sh
sudo apt install ros-noetic-apriltag-ros -y
```

- Copy the [launch-file](./continuous_detection.launch) into 
  `/opt/ros/noetic/share/apriltag_ros/launch/continuous_detection.launch`
- Copy the [settings file](./settings.yaml) into 
  `/opt/ros/noetic/share/apriltag_ros/config/settings.yaml`
- Copy the [tag definitions](./tags.yaml) into `/opt/ros/noetic/share/apriltag_ros/config/tags.yaml`

## Running

You can start the April tag node by running

```sh
roslaunch apriltag_ros continuous_detection.launch
```

Be sure to run this on the robot, not on your device as the network overhead of transferring
uncompressed images causes problems.

You can then get the data with:

```sh
rostopic echo /tag_detections
```

which contains messages of type
[`AprilTagDetection`](http://docs.ros.org/en/api/apriltag_ros/html/msg/AprilTagDetection.html).

> If you want to `echo` on a separate device, make sure you have installed the apt package from step
> 1 first.

## Notes

Adding new tags is trivially easy as seen in [`tags.yaml`](./tags.yaml). Do note that if the node
detects a tag with an id that is *not* listed in that file, it will throw a warning and *not*
publish anything to the topic. The warning conveniently contains the id of the new tag however so
it is easy to just add it to the `yaml` file.

We have also included the name of each tag which is not included in the
[topic data](http://docs.ros.org/en/api/apriltag_ros/html/msg/AprilTagDetection.html) unfortunately
but it is helpful to have in the file itself.

As of writing this, very little effort has been put into optimizing this node, we just made sure
that it can run adequately fast alongside our main computer vision code. That said, if optimization
is needed in the future, the places to look into would be the
[docs](http://wiki.ros.org/apriltag_ros#Parameters) and the [settings](./settings.yaml) file. The
`tag_decimate` parameter in particular seems to have a very strong performance-to-accuracy trade-off
for example.
