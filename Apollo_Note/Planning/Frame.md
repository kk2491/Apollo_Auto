### Frame Class

### Frame Class : 
Obstacle & Reference Line & Traffic Rule Fusion 


**Main Task :** 
To fuse the Predicted trajectory of the obstacles obtained by Prediction module + ego vehicle's Reference line provided by ReferencceLineProvider Class + Current road conditions (parking sign, crosswalk, speed bump etc)

Effect of obstacles and road conditions:

**a. Obstacle Impact**

Scenario 1: Obstacles behind Ego vehicle can be ignored
Scenario 2: Obstacles in front of Ego vehicle, decision must be taken accordingly (stop, overtake etc)

**b. Road condition Impact**

Scenario 1: Ego vehicle driving on reference line needs to be stopped if there is a forbidden region in front of it.
Scenario 2: Ego vehicle needs to stop if there is pedestrian crossing the Crosswalk, ignore if there is no one using the crosswalk.


**Major Task :**
Convert the road conditions into obstacle form.
Combine road conditions and obstacle traectory (from above step) to form a label indicating the obstacle exists.

In case of Ego vehicle, some obstacles can be ignored and some needs to be addressed.

**Doubt**
1. Obstacle information 
2. Ego vehicle refference line 
3. Labeling obstacle information accroding to traffic rules


#### 1. Obstacle Information Acquisition Strategy - Lagged Prediction

**Main Task :**

To obtain the obstacle trajectory estimated by the Prediction module and to perform post-processing.

Message format from Predictio module (File Name : prediction_obsttacle.proto)

```
message  Trajectory {
   optional  double  probability = 1 ;     // probability of this trajectory, the probability of the obstacle's trajectory motion plan 
  repeated  apollo.common.TrajectoryPoint  trajectory_point = 2 ;
}

message PredictionObstacle {
  optional apollo.perception.PerceptionObstacle perception_obstacle = 1;
  optional double timestamp = 2;  // GPS time in seconds
  // the length of the time for this prediction (e.g. 10s)
  optional double predicted_period = 3;
  // can have multiple trajectories per obstacle
  repeated Trajectory trajectory = 4;
}

message PredictionObstacles {
  // timestamp is included in header
  optional apollo.common.Header header = 1;
  // make prediction for multiple obstacles
  repeated PredictionObstacle prediction_obstacle = 2;
  // perception error code
  optional apollo.common.ErrorCode perception_error_code = 3;
  // start timestamp
  optional double start_timestamp = 4;
  // end timestamp
  optional double end_timestamp = 5;
}
```

Note:

Fetch obstacle information from Prediction module : ```prediction_obstacles.prediction_obstacle()```

Fetch the trajectories for obstacle : ```prediction_obstacle.trajectory()```

Fetch the published history messages in Adapter : ```const auto& prediction = *(AdapterManager::GetPrediction()) ```

Fetch recently released Prediction obstacles : ```prediction.GetLatestObserved()```

Apollo uses **Lagged Prediction** - more accurate obstacle prediction acquition method.
   
   Information released by Prediction module
   Data is also predicted using the obstacle trajectories in the historical information.






































