April 2:
- Extension announced
- Assigned node responsibilities: paul on node 1, karan on node 2, tom on node 3
- Planned approach and considered contingencies and logistics of accurate object placement within map
- Planned on using a frontier exploration algorithm with a wall following fall back
  April 3:
- Created GitHub issues for for each node in the repository
- Added basic repository setup with a doc folder and agents integration for GitHub issues cli
- Added digitized planning notes made in previous meeting yesterday
April 7: 
- Created initial package with configuration files such as constants.py and params.yaml 
April 8:
- Met with the full team in the lab 
- We all tried compiling the package in rosject
    - We all ran into lots of issues with this because I had added the navigation stack to the launch file which was unnecessary and caused the build and launch to continually fail
- We identified that there are AIIL and gazebo navigation stack variations, we need to make sure the right one is being launched whether or not we're in the real environment or the simulation.
- After not making much progress on this day we decided that it was probably best to avoid trying to troubleshoot the rosject until we have some code to actually test, especially when we've booked time in the lab
- We also discuss the potential use of the explorelite package
April 9:
- Paul and I met in the lab to test out the explorelite package and design an approach to our frontier exploration
- The explorelite package worked reasonably well with some optimization issues identified. Brayden informed me that many students often have issues with the package and sometimes it's easier to just build your own. 
- Paul investigated the explorelite package to consider reverse engineering it. We identified the key explorer node file and will use it as a reference for our own implementation.
- I've made a significant start on the logic for node 3. specifically the bread crumbing and storage of waypoints for the path of exploration transformed from the map.
April 10:
- I came into the lab early and set up the find object 2d session with all of the start hazard marker images
- implemented the return path breadcrumbing and the return trajectory calculation
  - the calculation uses two thresholds to thin waypoints a minimum distance and a minimum rotation angle. If either of these thresholds is met the waypoint is valid and will be included in the return trajectory. The return trajectory is path based
April 11
- added a test script for node three to monitor published paths and simulate home trigger. Node 3 is behaving as expected.
- implemented a dev container (tf_transformations dependency removed)
April 13 
- Began working on contingentcies, implemented a trigger listener node to listen to keyboard strokes however had issues detecting keyboard strokes in the headless terminal especially when it was launched as a node and is wrapped by ros2
- Added stub test scripts for nodes 1 and 2 for later use
April 14
- began setup of find_object_2d, created custom launch file in the package and tested find object 2d app still works expected and loads the correct session.
- implemented a object handler and object classes for object detection and the hazard manager and hazard classes for object identification
April 16
- Ceased all work on vision and market detection on Tim's instructions 
- Began testing my implementation for node 3 on the robot
April 17
- implemented a twist multiplexer so that I can run the whole configuration without navigation, having issues preventing nav2 from taking control
April 18
- Node 3 is working well in testing mode with no movement on the robot now I need to test it with movement
- Tried to get a few runs today but we wasted a lot of time on hot fixes and having issues with node one and three integrating
- Got one good clip of returning home working but it is not a great performance in the path is not accurate
April 19
- Went into the lab very early this morning but I kept getting no motors errors on the robots and eventually all of the robots were gone and I didn't get one the whole day
- Spent time trying to get it to launch on the previous generation robots but there were lots of compatibility issues and some of them were missing the images for the docker configuration and the wifi was extremely limited because of the amount of people

April 20,21,22 away

April 23 DEMO DAY
- Did the demonstration today we were doing a lot of changes right beforehand which was a bad sign
- We got two runs in the ten minutes the first one the robot crashed after navigating for about fifteen seconds
- The second run we couldn't get the stack to launch fast enough

April 24
- Implemented a topic bagging script to capture all of the topic outputs in case I can't get good footage
- I also improved the rviz configuration to change the color of planning, exploration, and return paths to be distinguishable. 
- Spent a lot of time trying to fix random synchronization issues with the hardware and the software like the navigation stack or map not being available
- Not really any good footage of Node 3
- Karan got the start marker detection working

April 25
- Again we're having a lot of synchronization issues so we completely removed the synchronization checks because it increased the complexity and there was topic blocking
- Still having lots of issues with node 1 blocking infinitely while it waits for the navigation stack even though it's loaded
- Karan got the has a detection working in the logs and had the code for the transformations and the map placement however it was not working

April 26
- Just worked on the report all day and improving documentation. Also standardized the compiled evidence naming format.
- exported all of the topic bags so that they are easily inspectable 