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
