Duckietown ROS API
==================

The Duckietown part of Mirte has several topics were data is published. 
All this data can be used to make Mirte drive around in Duckietown.

/line-segments      
    ROS topic for line segment data. The /line-segments topic publishes a list of line-segments. A line-segment has three components:

    #. A colour idicating which colour the line is
    #. A start Point
    #. An end Point

/stop-line          
    ROS topic for stop-line data. The /stop-line topic publishes the location of a single stop-line. This single stop-line has the following two components:

    #. An Origin
    #. A Direction

/tag-detections     
    ROS topic for AprilTag data. The /tag-detections publishes ...

/obstacles          
    ROS topic for obstacle data. The /obstacles topic publishes a list of obstacles. An obstacle has the following three components:

    #. A Point for its location in the image
    #. A diamater 
    #. The type of Object which is either a Duck or Mirte-bot

/lanes              
    ROS topic for lane data. The /lanes topic topic publishes a single lane. This lane has the following three components:

    #. A left line
    #. A right line
    #. A middle line 
