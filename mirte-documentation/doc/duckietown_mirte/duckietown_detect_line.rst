Following a lane
================

.. tabs::

    .. group-tab:: Blockly

        .. code-block:: python 

            print("placeholder")

    .. group-tab:: Python-simple

        .. code-block:: python

            from mirte_robot import robot
            mirte=robot.createRobot()
            from mirte_duckietown import duckietown
            camera=duckietown.createCamera(mirte)
            import time

            # this makes the Mirte bot follow the line for 30 seconds
            camera.startFollowing()
            time.sleep(30)
            camera.stopFollowing()

    .. group-tab:: Python-hard 

        .. code-block:: python 

            print("placeholder")
