## Manual
In the manual, the steps for running the code are explained. 
1. Open a terminal and run `roscore`. 
Following this step starts ROS.
2. Open a new terminal and run `r2start`.
Following this step, both the gazebo and rviz visualizations ar opened. In gazebo, additioal objects can be added. In rviz, the line segments can be vizualized by adding ther Markers: ...
3. Open a new terminal and run `rosrun danique simple_node`.
To change the area of the cart and the side which the cart is facing can both be changed. However, this needs to be changed in the code by adjusting the values `...` and `...`.

## Goal of the code
The goal of the code is for the RoPod to position accurately in front of the cart for docking. Since the docking
mechanism is not placed within the FoV of LiDAR, a local localization method is implemented for positioning. Thus, the
RoPod will use objects around the cart to perform closed-loop position and therefore, maintaining accuracy.

## Functions
### MakingPointCloud
**Description**: In this function the laser scan data is transformed from a polar coordinate system to a cartesian
coordinate system.
**Input values**: the laser scan, the point cloud.
**Output values**: -

### MakingLineSegments
**Description**: In this function the point cloud is used to make line segments of the surroundings. The line
segments are made with a mean residuals approach. Every line segment will be given a begin point, end point, direction
vector and a certainty.
**Input values**: segments, point cloud, start point, additional points, previous residual.
**Output values**: -

### RotationDifference
**Description**: In this function the angular difference between the initial robot frame and the segment is calculated.
**Input values**: the begin point and the end point.
**Output values**: angle

### TransformPosition
**Description**: In this function a point coordinate is transformed from one frame to another frame.
**Input values**: a point and a frame
**Output values**: vector of x and y

### TransformPositionB
**Description**: In this function a point coordinate is transformed from one frame to another frame.
**Input values**: a point and a frame
**Output values**: vector of x and y

### FindPoseDiff
**Description**: In this function the difference between two poses is calculated in the initial robot pose frame.
**Input values**:  frame and frame
**Output values**: vector of x, y and angle.

### ResetSegmentFrame
**Description**: In this function the measured segments are transformed from the current robot frame to the initial
robot frame.
**Input values**: vector of segments and frame.
**Output values**: -

### FindingEntrance
**Description**: In this function every combination of two segments is tested, such that they have the same direction
vector and are the width of one/two door(s) apart. With the assumption that one door is 83-88 cm wide.
**Input values**: vector of segments and frame.
**Output values**: vector of x, y and angle.

### FindAreaPose
**Description**: In this function the two points of the docking side of the area where the cart can be found are tested,
to see which one is closer.
**Input values**: a line and a pose
**Output values**: -

### Wait
**Description**: In this function the measured point cloud is used to see if there is any dynamic object in the way for driving,
if so it will tell teh RoPod to wait until the dynamic object passed by.
**Input values**: point cloud, distance and a pose.
**Output values**: boolean.

### CalculateSpeed
**Description**: In this function the speed for the RoPod is calculated while satisfying velocity and acceleration
constraints. The velocity is only published for lateral movements or the angular movement.
**Input values**: pose, pose, frequency, distance and point cloud.
**Output values**: vector of x, y and angle.

### SetTwistMessage
**Description**: In this function the calculated speed is set for the actual twist message to publish. Mostly, to
reduce lines of code in the node.
**Input values**: message and pose.
**Output values**: -

### CompareSegments
**Description**: This function compares two different measurements of segments to find the segments that are the
same/similar.
**Input values**: vector of segments, vector of segments and pose.
**Output values**: vector segments.

### CertaintyFilter
**Description**: This function filters the segments based on the certainty given initially to each line segment.
**Input values**: vector of segments, amount and segment.
**Output values**: vector segments.

### FindObjects
**Description**: This function filters the segments on the FoV the robot in its desired pose and filters afterwards for
segments that perpendicular to the side of the cart to be docked.
**Input values**: vector of segments, pose and segment.
**Output values**: vector segments.

### FindHighestCertainty
**Description**: Function to find the highest value in a vector.
**Input values**: segments and segment.
**Output values**: number of of the value in the vector.

### FindCart
**Description**: In this function all line segments are tested to be in the area of the cart and with a similar direction
vector of the side of the cart to be docked.
**Input values**: segments, line and line.
**Output values**: a segment.

### FindDistance
**Description**: In this function the distance between the begin and end point of the segment and the desired pose of the
robot is calculated.
**Input values**: pose, pose and segments.
**Output values**: vector of distances.

### FindDesiredPose
**Description**: In this function the desired pose of the robot in front of the cart is calculated.
**Input values**: segment, distance and distance.
**Output values**: vector of x, y and angle.

### CalculateError
**Description**: In this function the difference between teh desired distance to objects and the actual distance to the
objects is calculated.
**Input values**: vector of distances and vector of segments.
**Output values**: vector of x, y and angle.

## Node `simple_node`:
1. The surroundings are measured with the LiDAR, and this information transformed to line segments.
2. Search for the entrance, and update the given cart information to the initial robot frame.
3. Drive through the entrance.
4. Drive to the area of the cart.
5. Measure the surroundings again to find the cart.
6. Calculate the desired docking pose and an intermediate pose for positioning. The difference between the two poses is
only in the direction for accurate positioning.
7. Already calculate what objects can be used for positioning and what the desired distance to each of the objects is.
8. Drive to the intermediate position.
9. Drive closed loop to the desired position (so while checking the desired distance to each object).
10. Dock.
