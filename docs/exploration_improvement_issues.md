
## 🔄 Modify Existing: Issue #3 — Node 1: Autonomous exploration strategy

**Add to the existing body:**

```markdown
## Additional Improvements

### Frontier Clustering & Centroid Targeting
Instead of treating every frontier pixel as a candidate goal, group adjacent frontier
cells into clusters and target their centroids. This eliminates jitter caused by
the robot chasing single noisy pixels near walls or corners.

### Information Gain Heuristic
Replace the "Nearest Frontier" selection strategy with a scored heuristic:
  Score = Unknown Area Revealed / Distance to Goal
This ensures the robot prioritises frontiers that open into large unmapped regions
over nearby dead-ends, reducing total exploration time.

### Event-Based Replanning
Remove any timer-based replanning loop. Replanning is computationally expensive on
large costmaps and should only be triggered by meaningful events, not a fixed clock.

## Technical Suggestions (additions)
- Run a connected-components pass (e.g. `scipy.ndimage.label` or a BFS flood-fill)
  over frontier cells before scoring; discard clusters smaller than 5–10 cells
- Store each cluster's size and centroid; pass centroid as the `NavigateToPose` goal
- Compute information gain by counting unknown cells (`-1`) in a radius around each
  frontier centroid using the `/map` OccupancyGrid
- Trigger replanning only on Nav2 action callbacks: `SUCCEEDED`, `ABORTED`, `CANCELED`,
  or on receipt of `/snc_hazard_signal`

## Acceptance Criteria (additions)
- [ ] No single-pixel frontier goals are ever sent to Nav2
- [ ] Clusters smaller than the minimum threshold are discarded before scoring
- [ ] Frontier selection demonstrably prefers high-information-gain goals over
      nearest goals in a test environment with varied room sizes
- [ ] Replanning is event-driven only — no periodic timer calls BFS on the costmap
```

---

## 🆕 Issue #11 — Node 1: Scan-on-arrival behaviour

**Labels:** `node-1`, `navigation`

```markdown
## Description
After arriving at each frontier goal, the robot performs a structured camera sweep
before selecting the next frontier. Markers are invisible to LiDAR, so the camera
must actively survey each new vantage point. This "scan-on-arrival" step maximises
the chance of detecting a marker every time the robot reaches a new position.

## Technical Suggestions
- After `NavigateToPose` returns `SUCCEEDED`, publish angular velocity commands to
  `/cmd_vel` to rotate the robot in place before resuming frontier selection
- Choose between a full 360° spin or a ±90° sweep depending on time budget;
  a 360° spin at ~0.4 rad/s takes ~16 seconds — tune this to the 4-minute limit
- Publish `STATUS_SCANNING` on `/snc_status` during the sweep so other nodes are
  aware the robot is not navigating
- Gate the next `NavigateToPose` call so it cannot be sent until the sweep coroutine
  or action completes; use `asyncio` or a state machine flag
- Integrate with the hazard signal: if `/snc_hazard_signal` fires mid-sweep, abort
  the sweep early and handle the marker interrupt (see Issue #12)

## Acceptance Criteria
- [ ] Robot performs a rotation sweep after every successful `NavigateToPose` goal
- [ ] No new frontier goal is dispatched until the sweep is fully complete
- [ ] Angular velocity is tuned so camera frames have sufficient dwell time per
      frame (verify no motion blur causes missed detections)
- [ ] `STATUS_SCANNING` is published on `/snc_status` for the duration of the sweep
- [ ] Sweep is interruptible when `/snc_hazard_signal` is received mid-rotation
```

---

## 🆕 Issue #12 — Node 1: Marker interrupt & dynamic goal pre-emption

**Labels:** `node-1`, `navigation`, `vision`

```markdown
## Description
When the camera detects a marker mid-exploration, the robot should immediately
cancel its current Nav2 frontier goal and navigate to an optimal observation
position in front of the detected marker. This "interrupt" behaviour ensures the
robot reacts to detections in real time rather than finishing an irrelevant goal
first, which could allow the marker to leave the camera's field of view.

## Technical Suggestions
- Subscribe to the camera node's marker detection topic (e.g. a `PoseStamped`
  or `geometry_msgs/PointStamped` publishing the marker's estimated world position)
- On receipt, call `nav2_action_client.cancel_goal_async()` to abort the current
  `NavigateToPose` goal before sending a new one
- Calculate an observation waypoint offset from the marker pose: project a point
  0.5–1.0 m back along the marker's surface normal so the robot faces the marker
  at an optimal distance (use `tf2` to transform between frames as needed)
- Send a new `NavigateToPose` goal to the observation waypoint
- After arriving, trigger the scan-on-arrival sweep (Issue #11) to confirm the
  marker with the camera at close range
- Once the marker is confirmed and logged, resume frontier exploration from the
  current position
- Guard against rapid re-triggering: ignore further interrupts for the same marker
  ID for at least 10 seconds after confirmation

## Acceptance Criteria
- [ ] Robot cancels its active Nav2 goal within one control loop cycle of receiving
      a marker detection message
- [ ] Robot navigates to a computed observation point 0.5–1.0 m from the marker,
      not to the marker's exact pose
- [ ] After arrival, the scan-on-arrival sweep fires automatically before resuming
      exploration
- [ ] The same marker ID cannot trigger a second interrupt within 10 seconds
- [ ] Exploration resumes correctly from the interrupted position after confirmation
- [ ] Behaviour is tested end-to-end in simulation with a known marker placement
```

---

## 🆕 Issue #13 — Node 1: Marker blacklist for failed approach zones

**Labels:** `node-1`, `navigation`

```markdown
## Description
If the robot repeatedly fails to reach a detected marker's observation waypoint
(e.g. due to a narrow gap, occluding obstacle, or costmap inflation), it should
blacklist that coordinate temporarily to prevent an infinite loop of failed Nav2
goals targeting the same unreachable location.

## Technical Suggestions
- Maintain an in-memory list of blacklisted `(x, y)` positions with a timestamp
  of when each was added
- Before dispatching any observation waypoint (from Issue #12) or frontier goal,
  check if it falls within a configurable radius (e.g. 0.3 m) of a blacklisted
  entry; if so, discard the goal and select the next best candidate
- Increment a per-marker failure counter each time a `NavigateToPose` goal returns
  `ABORTED` or `CANCELED` for that waypoint; blacklist after 3 consecutive failures
- Expire blacklist entries after 60 seconds so the robot can retry if its map or
  position estimate has improved
- Log each blacklist event to the ROS2 logger (`rclpy.logging`) at `WARN` level,
  including the coordinate and expiry time, for debugging during the challenge

## Acceptance Criteria
- [ ] A waypoint that fails 3 consecutive times is added to the blacklist
- [ ] Blacklisted waypoints are not retried until the 60-second expiry elapses
- [ ] Frontier and observation goals are checked against the blacklist before being
      dispatched to Nav2
- [ ] Blacklist entries are logged with coordinate and expiry timestamp at WARN level
- [ ] After expiry, the robot is able to retry the previously blacklisted location
- [ ] Blacklist logic is unit-tested independently of Nav2 and the camera node
```

---

## 🆕 Issue #14 — Nav2: Goal safety — push frontier goals into free space

**Labels:** `infrastructure`, `navigation`

```markdown
## Description
LiDAR-derived frontier cells often sit directly on or adjacent to wall boundaries,
inside the costmap's inflation radius. Nav2's local planner will frequently reject
or abort these goals because they appear too close to an obstacle. This issue
covers a pre-dispatch validation step that nudges each goal point into confirmed
free space before it is sent to Nav2, reducing unnecessary goal failures.

## Technical Suggestions
- After computing a frontier centroid (Issue #3), read the costmap value at that
  cell from `nav2_msgs/Costmap` or directly from the `/global_costmap/costmap` topic
- If the cell value exceeds a configurable threshold (e.g. `> 50` on a 0–254 scale,
  i.e. inside the inflation radius), iterate outward from the centroid in a spiral
  or radial search until a cell with cost `< 10` (free) is found
- Cap the search radius at 0.5 m; if no free cell is found within that radius,
  discard the frontier and add it to the blacklist (Issue #13)
- Apply the same validation to observation waypoints generated by Issue #12 before
  they are dispatched
- Consider adding a `minimum_frontier_distance_from_wall` ROS2 parameter so the
  threshold can be tuned at launch time without recompiling

## Acceptance Criteria
- [ ] No Nav2 goal is dispatched with a costmap value above the configured threshold
- [ ] The nudge search finds a valid free-space cell and adjusts the goal position
      before the `NavigateToPose` call is made
- [ ] If no free cell is found within 0.5 m, the frontier is blacklisted rather
      than retried immediately
- [ ] The threshold and search radius are exposed as ROS2 parameters in
      `config/nav2_params.yaml` or an equivalent config file
- [ ] Goal failure rate in a mapping test run is measurably reduced compared to
      the baseline without this fix
```
