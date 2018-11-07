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


```
/// file in apollo/modules/planning/planning.cc
void Planning::RunOnce() {
  const uint32_t frame_num = AdapterManager::GetPlanning()->GetSeqNum() + 1;
  status = InitFrame(frame_num, stitching_trajectory.back(), start_timestamp, vehicle_state);
}

/// file in apollo/modules/planning/common/frame.cc
Status Frame::Init() {
  // prediction
  if (FLAGS_enable_prediction && AdapterManager::GetPrediction() && !AdapterManager::GetPrediction()->Empty()) {
    if (FLAGS_enable_lag_prediction && lag_predictor_) {      // 滞后预测策略，获取障碍物轨迹信息
      lag_predictor_->GetLaggedPrediction(&prediction_);
    } else {                                                   // Do not use the lag prediction strategy, directly take the obstacle information published by the latest Prediction module 
      . Prediction_ . CopyFrom ( AdapterManager::GetPrediction ()-> GetLatestObserved ());
    }
  }
  ...
}
```

Main steps of obtaining Obstacle trajectory using lag prediction strategy can be dvided into 

1. Most recently released data from Prediction module is added to **PredictionObstacles** container.
2. Post processing the released historical information

1. Most recently released data from Prediction module is added to **PredictionObstacles** container.

```
// / file in apollo/modules/planning/common/lag_prediction.cc 
void  LagPrediction::GetLaggedPrediction (PredictionObstacles* obstacles) const {
   // Step A. Last released obstacle trajectory prediction information processing 
  const  auto adc_position = AdapterManager:: GetLocalization ()-> GetLatestObserved (). pose (). position ();
   const  auto latest_prediction = (*prediction. begin ());         / / record the information published by the latest Prediction module 
  const  double timestamp = latest_prediction-> header () . timestamp_sec(); //The most recently published timestamp 
  std::unordered_set< int > protected_obstacles;
   for ( const  auto & obstacle : latest_prediction-> prediction_obstacle ()) {   // Get the last published data for each obstacle Motion track information 
    const  auto & perception = obstacle. perception_obstacle (); 
     double distance = common::util::DistanceXY (perception. position (), adc_position);
     if (perception. confidence () < FLAGS_perception_confidence_threshold &&   // obstacle confidence Must be greater than 0.5, the acquisition must be the vehicle VEHICLE class, otherwise it will not be processed
        perception.type() != PerceptionObstacle::VEHICLE) {
      continue;
    }
    If (distance < FLAGS_lag_prediction_protection_distance) {     // The distance between the obstacle and the vehicle is less than 30m, then the valid 
      protected_obstacles. insert (obstacle. perception_obstacle (). id ());
       // add protected obstacle 
      AddObstacleToPrediction ( 0.0 , obstacle, obstacles );
    }
  }
  ...
}
```

As seen in the above code snippet, the obstacle information needs to satisfy below 2 conditions to be valid:

a. **Obstacle Confidence** (obtained by Peception module CNN Segmentation) must be greater than 0.5 and the obstacle type is **Vehicle**

b. Distance between the obstacle and the ego vehicle should be less than 30m.

2. Post processing the released historical information

```
// / file in apollo/modules/planning/common/lag_prediction.cc 
void  LagPrediction::GetLaggedPrediction (PredictionObstacles* obstacles) const {
   // Step A Last released obstacle trajectory prediction information processing
  ...
  // Step B Historical obstacle trajectory prediction information processing published in the past 
  std::unordered_map< int , LagInfo> obstacle_lag_info;
   int  index = 0 ;   // data in begin() is the most recent data 
  for ( auto it = prediction. begin (); it != prediction. end (); ++it, ++ index ) {    // Process each published message 
    for ( const  auto & obstacle : (*it)-> prediction_obstacle ()) {                  / / Get the historical data of the release, the motion track information of each obstacle 
      const  auto & perception = obstacle.Perception_obstacle ();
       auto id = perception. id ();
       if (perception. confidence () < FLAGS_perception_confidence_threshold &&     //The obstacle confidence must be greater than 0.5, the acquisition must be the vehicle VEHICLE class, otherwise it will not be processed 
          . type () != PerceptionObstacle::VEHICLE) {
         continue ;
      }
      If (protected_obstacles. count (id) > 0 ) {           // If the obstacle appears in the last post, it is ignored, because only the latest obstacle information 
        continues ;   // don't need to count the Already added protected obstacle
      }
      auto& info = obstacle_lag_info[id];        
      ++info. count ;                           // Record the number of times an obstacle appears in all history information 
      if ((*it)-> header (). timestamp_sec () > info. last_observed_time ) {       // Save the last occurrence of the message because Only consider the latest obstacle information 
        info. last_observed_time = (*it)-> header (). timestamp_sec ();
        info.last_observed_seq = index;
        info.obstacle_ptr = &obstacle;
      }
    }
  }
  BOOL apply_lag = STD :: Distance (Prediction. the begin (), Prediction. End ())> = static_cast < int32_t > (min_appear_num_);
   for ( const  Auto & ITER: obstacle_lag_info) {
     IF (apply_lag && ITER. SECOND . COUNT < Min_appear_num_) {        // If the number of obstacles in the history information is less than min_appear_num_/3 times, the number of times is too small and can be ignored. 
      Continue ;
    }
    If (apply_lag && iter. second . last_observed_seq > max_disappear_num_) { //In the history information, if the obstacle was released too far, it can be ignored. 
      Continue ;
    }
    AddObstacleToPrediction(timestamp - iter.second.last_observed_time,
                            *(iter.second.obstacle_ptr), obstacles);
  }
}
```
### Need to check the above code snippet


Below are the 2 steps to determine whether the obstacle trajectory information is valid.

**Step 1:**
Record the number of occurances of each obstacle from historial release data (Why ??? - Obstacles that appear in the recent release are ignored, because they are not the latest data).

Below 2 conditions need to be met to make the obstacle information valid:

a. Obstacle confidence obtained from Perception module's CNN segmentation must be greater than 0.5 and the obstacle type is Vehicle.

b. The distance between the obstacle and the ego vehicle is less than 30m.


**Step 2:**
The obstacle information received from Step 1 needs to satisfy the below 2 condition to be valid.

a. Historical data in the message queue is greater than 3 (min_appear_num) and the number of occurances of each obstacle is greater than 3 (min_appear_num).

### What is this???
b. Historical data in the message queue is greater than 3 (min_appear_num), and the last release of the obstacle information is not more than 5 (max_disappear_num_) and the latest validity of the data needs to be guaranteed.



## 2. Setting relative position of Ego vehicle and obstacle-ReferenceLineInfo class initialization

From the previous step (Obstacle information acquisition strategy) the motion trajectory of the obstacle (future 5s) is obtained. 

From **ReferenceLineProvider** class, we get the ideal planning trajecctory of the Ego vehicle.

Next step is to add the obstacle trajectory information to the planned reference line (ReferenceLine) to determine at what point in time the ego vehicle can advance so that it doesn't collide with the obstacle trajectory.

This task is done in **Frame::Initi()** to complete the generation of ReferenceLineInfo class, which combines the information of the obstacle prediction trajectory and the ego vehicle's planning trajectory. And also the basic class of the final path planning.

```
// / file in apollo/modules/planning/common/frame.cc 
Status Frame::Init () {
   // Step A prediction, obstacle prediction trajectory information acquisition, using lag prediction strategy
  ...
  // Step B.1 Check the current time (relative_time=0.0s), whether the unmanned vehicle position and the obstacle position overlap (collision), and if so, you can exit 
  const  auto *collision_obstacle = FindCollisionObstacle ();    
   if (collision_obstacle) {
    AERROR << "Found collision with obstacle: " << collision_obstacle->Id();
    return Status(ErrorCode::PLANNING_ERROR, "Collision found with " + collision_obstacle->Id());
  }
  // Step B.2 If the current time does not conflict, check the position where the unmanned car can advance on the reference line in the future. ReferenceLineInfo generates 
  if (! CreateReferenceLineInfo ()) {    // 
    AERROR << " Failed to init reference line info " ;
     return  Status (ErrorCode::PLANNING_ERROR, " failed to init reference line info " );
  }
  return Status::OK();
}
```

Initialization of **ReferenceLineInfo** class is done by below 2 processes:

a. Instantiate the **ReferenceLineInfo** class according to Ego vehicle planning path ReferenceLine.  







































