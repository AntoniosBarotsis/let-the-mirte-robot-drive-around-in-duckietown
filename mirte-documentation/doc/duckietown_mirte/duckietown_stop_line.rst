Stop line 
=========

.. tabs::

    .. group-tab:: Python-simple

        .. code-block:: python 

            from mirte_robot import robot
            mirte=robot.createRobot()
            from mirte_duckietown import duckietown
            camera = duckietown.createCamera()
            import time


            while True:
            mirte.setMotorSpeed('left', 40)
            mirte.setMotorSpeed('right', 40)
            if camera.seesStopLine():
            mirte.setMotorSpeed('left', 0)
            mirte.setMotorSpeed('right', 0)
            time.sleep(5)
        
    .. group-tab:: Python-hard 

        .. code-block:: python 

            print("hello world")
