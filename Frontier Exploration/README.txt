For this package its purpose is to listen to the Turtlebot's position through a TransformListener performing lookupTransform() and return that info to the console
While performing transform lookups the package sends hard coded goals to the Turtlebot to traverse to.

To utilize this package in the command line while the Turtlebot is running you must type each command in seperate terminals in the order shown
Since the goals are hard coded just use the .launch command as shown below
$ roslaunch project2 turtlebot3_navigation.launch
$ rosrun project2 TransformListners.py
$ rosrun project2 SimpleActionClient.py

The goals passed to the Turtlebot client are hard coded so if you wish to change them you must access the SimpleActionClient.py file adjust the [x,y,z] passed to pose.
Currently there are 2 goals passed to the Turtlebot client through Pose, each x y z value must be changed individually for each goal should you wish to change them.

With Task 2 the Detection.py script was added which launches a node for frontier detection, frontier publishing for vizualization with Markers.
These frontiers were found using OpenCV Countours but segmented by K-means AgglomerativeClustering with 6 clusters. 
K-Means was used to determine the cluster centroids which were then displayed using a unique marker.
To launch this file run the following command and usually the data will begin procesing but might need the robot to move initally to update the Rviz display.
$ rosrun project2 Detection.py
The amount of clustes to be found are hard coded in the Detection.py file and would have to be changed manually.
Frontiers and their markers are only updated when a change is detected in the OccupancyGrid.
Occasionally if the data is not being updated the console might output some errors for out of range checks this is of no issue to the script and the code will continue to run for updates.

For Task 3 the code was update to search the created centroids and determine which one is closest to the robot and mark that as its goal.
The goal centroid is then marked by a different color from the other centroids and frontiers.
Should a goal be unreachable the robot will eventually abandon the goal and Detection.py will check that if a goal is abandoned the whole frontier will be marked as obstacles.
    This is to prevent a new centoid being created in the unreachable area.
The now unreachable centroid position will be stored and checked against for future goals to see if new goals are too close to an unreachable area and find a new goal.
To launch this file run the following command:
$ rosrun project2 Detection.py
Some issues that may occur with exploration is the robot traveling to different areas than marked by the goal centroid.
Another one is that the robot will travese a large portion of the map to decide how it will reach the goal.
To fix the previous issue of index out of bounds errors displaying in the terminal the contours algorithm had to be adjusted and as a result the frontier detection is less accurate.
    Some possible frontiers are not marked by OpenCV contours and as such will not be accounted for in segmentation. These frontiers eventually get explored due to nearby frontiers being traversed.