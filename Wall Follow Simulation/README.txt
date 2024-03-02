For this package its purpose for Task 1 is to manually define a Q-Table that will have predetermined rewards to have it navigate a map with right wall following.
To run this file execute the following command
$ roslaunch project3 wallfollow.launch
$ rosrun project2 Follow.py

At this time the package Follow script will restart the simulation on launch and set the robot model's position to (-2,-2)
AFter getting its position set the robot will then begin laser scanning the area around it and being wall following based on its defined Q-Table
For now the robot's speeds will occasionally force it into a wall but given time it will push itself out and continue navigating.

With Task 2 this package has the option for q-table learning or testing.
When testing the script command should be used as follows:
$ rosrun project2 Follow.py output.txt
Where you specify the name of the learned q-table file to test on the robot. In this package output.txt is a learned q-table used in the Demo Video, this is the same file at this point as Complete.txt
When a test file is used that file is then written out to output.txt and creates its own blank graph.

When learning you use the same script command from task 1, to visualize learning from a percentage of correct actions of a greedy choice a graph.png file is created and updated every episode.
To view simply open the file created in your current directory and keep it open if you wish to see an update.
An output.txt file is updated every episode to write out the current learned Q-Table.