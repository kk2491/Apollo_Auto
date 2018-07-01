## Introduction

This guide describes how to add, build, and use the obstacle sim capability within Apollo to create multiple obstacles in Dreamview.

### Adding Obstacle Sim
The following are needed for obstacle sim to function in order to generate multiple objects:
1. In the main Apollo folder: **obstacle_sim_test.sh**
2. Obstacle sim directory provided here which needs to be copied into the **/modules/drivers** directoty.

### Building Obstacle Sim
To build the module, within Docker run bash obstacle_sim_test.sh build. The resultant files are contained within **modules/devel_isolated/**.

### Running Obstacle Sim
Note that you must have **roscore** running prior to running the module via the **bash scripts/bootstrap.sh** command in docker.

To run the module, first use the command **source modules/devel_isolated/setup.bash**. Then within the same console use the command **rosrun obstacle_sim obstacle_sim**. This will create the obstacles in Dreamview as per the data updated in CSV file.

In addition to the normal routing and planning options that must be active for the vehicle to operate in SimControl, you MUST also have turned on the **Prediction module**. This is due to the fact that the planner only takes in the prediction obstacle output, while the obstacle sim module produces a perception obstacle.

Note that at this point in time, the module does not exit cleanly. Therefore, once this command is run you must close that console window to stop the process.

### Creating New Obstacles
In the CSV file the parameters must be updated in the format as shown below separated by **;** (semicolon).

![](file:///H:\MD\CSV_File_Update.JPG)

Note that at this point of time there is no column names in the CSV file and data must be entered in the order as below

        location.set_x()  
        location.set_y()  
  	    location.set_z()  
  	    heading  
  	    length  
  	    width  
  	    height

To determine an appropriate position for the obstacles, you can utilize the x and y UTM values specified within the base_map.txt file. For example, choose an arbitrary point along a specific lane's center line in the base_map.txt, and use that as the location of the obstacle. If the obstacle is placed incorrectly, you can always go back and modify the location values and run the module again. At this point in time the module must be rebuilt (see above) each time these values are modified.

Future development will include CSV file with column names and dynamic obstacles.

