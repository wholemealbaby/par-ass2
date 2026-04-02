## Relevant Files from Rosbot Demo Package
Navigation:
nav2_example.py (uses action client)
publish_nav_path.py (uses topics)
goToPose.py (uses topics)

Hazard detection and mapping:
publish_hazard.py
repeater_example.py
transform_example.py

## Node 1 (Navigation Logic & pid controller)
### Triggers
- Listens To Node2 (is there a dedicated topic for this in the spec?)
  - For contingencies
    - Return home (home trigger)
    - Start exploration (start trigger)
    - Teleop keyboard (teleop trigger)
  - For objects recognised in the camera (object trigger)
  - For automatic start after sensing the first card (init trigger)
  - For automatic finish, this may be redundant (finish trigger)
- avoids static obstacles
  - Build on obstacle avoidance code from the construct example
  - Some contingency for light flickers in the Lidar
- Subscribe From Node3 (is there a dedicated topic for this in the spec?)
  - For path planning and retracing
  - Need to publish positions to Node3 for path planning (this sounds like actions)
  - Need to subscribe to the path from Node3 for finish trigger (This sounds like action behavior)
- Searched the unexplored frontier
  - Could start with basic wall follower assuming no cycles
  - Navigation stack won't enter unexplored territory
  - Need to send waypoints to frontiers territory to fully explore
  - Do we need local global and emergency path planning?

Disscussion:
- is there too many responsibilities for this one node could we offload anything more to the path planning node

## Node 2 (Hazard Market Detection and map placement)
- Signals the start trigger when start card is detected
- needs to use find_object_2D topic to read detected hazard signs and then place them in the map using the published hazard code
- Needs to use the ids for each hazard from the spec
- Needs to transform the location of the image from the depth camera into the map frame
- Auto trigger returned to home after five hazards have been mapped (NOT INCLUDING START)
- We could potentially trigger a reporting behavior from the navigation node when a hazard marker is detected to get close enough for accurate report to prevent inconsistencies
  - For more accurate positioning we could use the mode position of the detections for that particular marker to make sure that it is mapped to the right position

## Node 3 (Path Planning & Retracing)
- Reverse the path and invert the orientations for the poses published by the navigation node
