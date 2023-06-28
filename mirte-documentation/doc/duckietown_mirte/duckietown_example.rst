===========
One example
===========

This will show two examples for driving around in Duckietown. 
One example will be very simple while the other one is more complicated.

++++++++++++++
Simple example
++++++++++++++

This example does the following:

#. Start following the road
#. When it sees a stop-line stop following

.. tabs:: 

    .. group-tab:: Blockly

        |pic-duck-1|
    
    .. group-tab:: Python-simple

        .. code-block:: python 

            from mirte_robot import robot
            mirte=robot.createRobot()
            from mirte_duckietown import duckietown
            camera=duckietown.createCamera(mirte)
            import time

            camera.startFollowing()
            wait_cond = (camera.seesStopLine())
            while not(wait_cond):
	            time.sleep(.1)
	            wait_cond = (camera.seesStopLine())
            camera.stopFollowing()

+++++++++++++++++++
Complicated example
+++++++++++++++++++

This example does the following:

#. 

.. tabs:: 

    .. group-tab:: Blockly

        placeholder text

    .. group-tab:: Python-simple

        .. code-block:: python

            print("placeholder")



.. |pic-duck-1| image:: duckie_images/simple_example.jpg