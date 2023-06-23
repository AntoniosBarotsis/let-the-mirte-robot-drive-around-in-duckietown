Stop line 
=========

.. tabs::

    .. group-tab:: Blockly

        |pic_duck_2|
        
    .. group-tab:: Python-simple 

        .. code-block:: python 

            from mirte_robot import robot
            mirte=robot.createRobot()
            from mirte_duckietown import duckietown
            camera=duckietown.createCamera(mirte)
            import time

            # When it sees a stop line stop following the lane
            camera.startFollowing()
            wait_cond = (camera.seesStopLine())
            while not(wait_cond):
	            time.sleep(.1)
	            wait_cond = (camera.seesStopLine())
            camera.stopFollowing()

.. |pic_duck_2| image:: duckie_images/stop_lane_blockly.png