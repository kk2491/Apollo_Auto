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


## 1. Obstacle Information Acquisition Strategy - Lagged Prediction

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

a. Instantiate the **ReferenceLineInfo** class according to Ego vehicle planning path ReferenceLine. (What is this: The number is consistent with with the ReferenceLine)

b. Initialize **ReferenceLineInfo::path_decision_** based on the obstacle trajectory.


a. Instantiate the **ReferenceLineInfo** class

```
// / file in apollo/modules/planning/common/frame.cc 
bool  Frame::CreateReferenceLineInfo () {
   // Step A Get the short-term planning path ReferenceLine of the unmanned vehicle from the ReferenceLineProvider and perform the shrink operation
  std::list<ReferenceLine> reference_lines;
  std::list<hdmap::RouteSegments> segments;
  if (!reference_line_provider_->GetReferenceLines(&reference_lines, &segments)) {  
    return false;
  }
  // Generate a corresponding ReferenceLineInfo for each ReferenceLine instantiated 
  reference_line_info_. clear ();
   auto ref_line_iter = reference_lines. begin ();
   auto segments_iter = segments. begin ();
   while (ref_line_iter != reference_lines. end ()) {
     if (segments_iter-> StopForDestination ()) {
      is_near_destination_ = true;
    }
    reference_line_info_.emplace_back(vehicle_state_, planning_start_point_,
                                      *ref_line_iter, *segments_iter);
    ++ref_line_iter;
    ++segments_iter;
  }
}

/// file in apollo/modules/planning/common/reference_line_info.cc
ReferenceLineInfo::ReferenceLineInfo(const common::VehicleState& vehicle_state,
                                     const TrajectoryPoint& adc_planning_point,
                                     const ReferenceLine& reference_line,
                                     const hdmap::RouteSegments& segments)
    : vehicle_state_(vehicle_state),
      adc_planning_point_(adc_planning_point),
      reference_line_(reference_line),
      lanes_(segments) {}

```

This extracts the short-term planning path of Ego vehicle from the ReferenceLineProvider.

Then generates a corresponding ReferenceLineInfo from a ReferenceLine&&segments, vehicle status and planning start point. 

ReferenceLineProvider : https://github.com/YannZyl/Apollo-Note/blob/master/docs/planning/reference_line_provider.md


b. Initialize **ReferenceLineInfo** class

```
// / file in apollo/modules/planning/common/frame.cc 
bool  Frame::CreateReferenceLineInfo () {
   // Step A Get the short-term planning path ReferenceLine of the unmanned vehicle from the ReferenceLineProvider and perform the shrink operation 
  ... / / Step B RerfenceLineInfo initializes bool has_valid_reference_line = false ;
   for ( auto &ref_info : reference_line_info_) {
     if (!ref_info. Init ( obstacles ())) {
       continue ;
    } else {
      has_valid_reference_line = true ;
    }
  } return
  
  
   has_valid_reference_line;
}
```

ReferenceLineInfo::Init(const std::vector<const Obstacle*>& obstacles) function does the following tasks:

- Check if the drone is on the reference line (What is drone here???)

	Bounding boxes start_s and end_s of Ego vehicle needs to be within the reference line [0, total_length]
	
- Check if the drone is too far from the reference line

	Ego vehicle's start_l and end_l needs to within [-kOutOfReferenceLineL, kOutOfReferenceLineL] interval, where kOutOfReferenceLineL = 10
	
	
- Add obstacle information to the ReferenceLineInfo class

	Another important task is to determine where the Ego vehicle can advance at a certain point in time as shown in the below figure.
	
	## Figure Here ##
	
This process is based on the trajectory of the obstacle (relative time point which gives the position of the obstacle moves to) combined with ideal path obtained by Ego vehicle (get low_t and high_t of the ego vehicle).
Lower bound of the travel distance = low_s - adc_start_s
Upper bound of the travel distance = high_s - adc_start_s

```
/// file in apollo/modules/planning/common/reference_line_info.cc
// AddObstacle is thread safe
PathObstacle* ReferenceLineInfo::AddObstacle(const Obstacle* obstacle) {
  // 封装成PathObstacle并加入PathDecision
  auto* path_obstacle = path_decision_.AddPathObstacle(PathObstacle(obstacle));
  ...
  // 计算障碍物框的start_s, end_s, start_l和end_l
  SLBoundary perception_sl;
  if (!reference_line_.GetSLBoundary(obstacle->PerceptionBoundingBox(), &perception_sl)) {
    return path_obstacle;
  } 
  path_obstacle-> SetPerceptionSlBoundary (perception_sl);
   // calculate whether obstacles have an effect on unmanned vehicles: no light obstacles satisfy the following conditions: 
  //     1. Obstacles are outside the ReferenceLine, ignore 
  //     2. Vehicles and obstacles 
  Are in the lane, but the obstacle is behind the unmanned vehicle, ignore if ( IsUnrelaventObstacle (path_obstacle)) {
     // ignore obstacles 
  } else {
     // construct the bounding box of the obstacle on the reference line 
    path_obstacle-> BuildReferenceLineStBoundary (reference_line_, Adc_sl_boundary_. start_s ()); 
  } return path_obstacle; 
} // / file in apollo/modules/planning/common/path_obstacle.cc
  

void PathObstacle::BuildReferenceLineStBoundary(const ReferenceLine& reference_line, const double adc_start_s) {

  if (obstacle_->IsStatic() || obstacle_->Trajectory().trajectory_point().empty()) {
    ...
  } else {
    if (BuildTrajectoryStBoundary(reference_line, adc_start_s, &reference_line_st_boundary_)) {
      ...
    } else {
      ADEBUG << "No st_boundary for obstacle " << id_;
    }
  }
}
```

Step 1:
First choose obstacle track point (2 points). Each point can construct the object_moving_box and the object_boundary in the above figure.

```
bool PathObstacle::BuildTrajectoryStBoundary(
    const ReferenceLine& reference_line, const double adc_start_s,
    StBoundary* const st_boundary) {
  for (int i = 1; i < trajectory_points.size(); ++i) {
    const auto& first_traj_point = trajectory_points[i - 1];
    const auto& second_traj_point = trajectory_points[i];
    const auto& first_point = first_traj_point.path_point();
    const auto& second_point = second_traj_point.path_point();

    double total_length = object_length + common::util::DistanceXY(first_point, second_point); //object_moving_box总长度

    common::math::Vec2d center((first_point.x() + second_point.x()) / 2.0,             // object_moving_box中心
                               (first_point.y() + second_point.y()) / 2.0);
    common::math::Box2d object_moving_box(center, first_point.theta(), total_length, object_width); 
    ... // calculate object_boundary, which is obtained by rotating a heading of object_moving_box, recording start_s, end_s, start_l and end_l if (!reference_line. GetApproximateSLBoundary (object_moving_box, start_s, end_s, &object_boundary) )) {  
       return false ; 
    } 
  } 
}
```

Step 2:

Determine the distance between the obstacle and Ego vehicle. The obstacle can be ignored if it is on both side of the reference line or it is behind the reference line.

    
```
// skip if object is entirely on one side of reference line. 
    constexpr  double  kSkipLDistanceFactor = 0.4 ;
     const  double skip_l_distance =
        (object_boundary.end_s() - object_boundary.start_s()) * kSkipLDistanceFactor + adc_width / 2.0 ; if (std::fmin (object_boundary.start_l(), object_boundary.end_l()) >    //The obstacle is on the left side of the reference line, then the unmanned vehicle can pass the obstacle directly, and the obstacle 
            skip_l_distance || std::fmax (object_boundary.start_l( ), object_boundary.end_l()) <    //The obstacle is on the right side of the reference line, then the unmanned vehicle can pass the obstacle directly, the obstacle can be ignored- 
            skip_l_distance) {
            
        
      Continue ; 
    } IF (object_boundary.end_s () < 0 ) {   // block behind the reference line, the obstacle can be ignored Continue ; 
    }

```

Step 3:

Calculate the upper and lower bounding bo of the low_t and high_t.

```
const double delta_t = second_traj_point.relative_time() - first_traj_point.relative_time(); // 0.1s
    double low_s = std::max(object_boundary.start_s() - adc_half_length, 0.0);
    bool has_low = false;
    double high_s = std::min(object_boundary.end_s() + adc_half_length, FLAGS_st_max_s);
    bool has_high = false;
    while (low_s + st_boundary_delta_s < high_s && !(has_low && has_high)) {
      if (!has_low) {   // 采用渐进逼近的方法，逐渐计算边界框的下界
        auto low_ref = reference_line.GetReferencePoint(low_s);
        has_low = object_moving_box.HasOverlap({low_ref, low_ref.heading(), adc_length, adc_width});
        low_s += st_boundary_delta_s;
      }
      if (!has_high) {  // 采用渐进逼近的方法，逐渐计算边界框的上界
        auto high_ref = reference_line.GetReferencePoint(high_s);
        has_high = object_moving_box.HasOverlap({high_ref, high_ref.heading(), adc_length, adc_width});
        high_s -= st_boundary_delta_s;
      }
    }
    if (has_low && has_high) {
      low_s -= st_boundary_delta_s;
      high_s += st_boundary_delta_s;
      double low_t = (first_traj_point.relative_time() +
           std::fabs((low_s - object_boundary.start_s()) / object_s_diff) * delta_t);
      polygon_points.emplace_back(    // 计算low_t时刻的上下界
          std::make_pair(STPoint{low_s - adc_start_s, low_t},
                         STPoint{high_s - adc_start_s, low_t}));
      double high_t =
          (first_traj_point.relative_time() +
           std::fabs((high_s - object_boundary.start_s()) / object_s_diff) * delta_t);
      if (high_t - low_t > 0.05) {
        polygon_points.emplace_back(  // 计算high_t时刻的上下界
            std::make_pair(STPoint{low_s - adc_start_s, high_t},
                           STPoint{high_s - adc_start_s, high_t}));
      }
    }
    
```

Step 4:
After calculating the upper and lower bounds of all the obstacle track segments, track according to time t.

```
 if (!polygon_points.empty()) {
    std::sort(polygon_points.begin(), polygon_points.end(),
              [](const std::pair<STPoint, STPoint>& a,
                 const std::pair<STPoint, STPoint>& b) {
                return a.first.t() < b.first.t();
              });
    auto last = std::unique(polygon_points.begin(), polygon_points.end(),
                            [](const std::pair<STPoint, STPoint>& a,
                               const std::pair<STPoint, STPoint>& b) {
                              return std::fabs(a.first.t() - b.first.t()) <
                                     kStBoundaryDeltaT;
                            });
    polygon_points.erase(last, polygon_points.end());
    if (polygon_points.size() > 2) {
      *st_boundary = StBoundary(polygon_points);
    }
  } else {
    return false;
  }
```

Summary :

Function :Ego vehicle Reference Line ReferenceLineof Initialization 

This gives the Ego vehicle's planning trajectory ReferenceLine and the Obstacle's predicted trajectory PredictionObstacles (Why Why Why ???)

This calculates the position s of the overlapping portion on Ego vehicle planning trajectory and the time t at which overlapping portion is driven. 



## 3. Label the obstacles according to Traffic rules

The position and the time point of the overlap of each obstacle on reference line have been estimated in the previous part.

These overlapping parts are the planning correction problems that Ego vehicle needs to consider to prevent traffic accidents. Since the obstacle movement may span multiple reference lines, it is necessary to consider each obstacle and decide whether it can be ignored or not. 

There are 11 types of traffic rules. Defined in ```modules/planning/conf/traffic_rule_config.pb.txt```.

```
1. After car situation processing--BACKSIDE_VEHICLE
2. Change lane situation handling--CHANGE_LANE
3. Crosswalk situation handling--CROSSWALK
4. Destination Processing - DESTINATION
5. Front car situation handling--FRONT_VEHICLE
6. Forbidden zone situation handling--KEEP_CLEAR
7. Find parking status - PULL_OVER
8. Reference line End Processing - REFERENCE_LINE_END
9. Rerouting query processing - REROUTING
10. Signal Condition Processing--SIGNAL_LIGHT
11. Parking situation processing--STOP_SIGN
```

```
/// file in apollo/modules/planning/planning.cc
void Planning::RunOnce() {
  const uint32_t frame_num = AdapterManager::GetPlanning()->GetSeqNum() + 1;
  status = InitFrame(frame_num, stitching_trajectory.back(), start_timestamp, vehicle_state);

  for (auto& ref_line_info : frame_->reference_line_info()) {
    TrafficDecider traffic_decider;
    traffic_decider.Init(traffic_rule_configs_);
    auto traffic_status = traffic_decider.Execute(frame_.get(), &ref_line_info);
    if (!traffic_status.ok() || !ref_line_info.IsDrivable()) {
      ref_line_info.SetDrivable(false);
      continue;
    }
  }
}

/// file in apollo/modules/planning/tasks/traffic_decider/traffic_decider.cc
Status TrafficDecider::Execute(Frame *frame, ReferenceLineInfo *reference_line_info) {
  for (const auto &rule_config : rule_configs_.config()) {   // 对于每条参考线进行障碍物的决策。
    if (!rule_config.enabled()) {
      continue;
    }
    auto rule = s_rule_factory.CreateObject(rule_config.rule_id(), rule_config);
    if (!rule) {
      continue;
    }
    rule->ApplyRule(frame, reference_line_info);
  }

  BuildPlanningTarget(reference_line_info);
  return Status::OK();
}
```

### 3.1 After car situation processing - BACKSIDE_VEHICLE

```
/// file in apollo/modules/planning/tasks/traffic_decider/backside_vehicle.cc
Status BacksideVehicle::ApplyRule(Frame* const, ReferenceLineInfo* const reference_line_info) {
  auto* path_decision = reference_line_info->path_decision();
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  if (reference_line_info->Lanes().IsOnSegment()) {  // The lane keeping reference line.
    MakeLaneKeepingObstacleDecision(adc_sl_boundary, path_decision);
  }
  return Status::OK (); 
} void BacksideVehicle::MakeLaneKeepingObstacleDecision ( const SLBoundary& adc_sl_boundary, PathDecision* path_decision) { 
  ObjectDecisionType ignore;    // From here you can see that the processing for the following car is mainly to ignore 
  ignore. mutable_ignore ();
   const double Adc_length_s = adc_sl_boundary. end_s () - adc_sl_boundary. start_s (); // calculate "car length"" for ( const auto * path_obstacle : path_decision-> path_obstacles (). Items ()) { // for each with reference line Overlapping obstacles for rule setting

  
   
    ...  
  }
}
```

This condition is being evaluated only for the reference line where the ego vehicle is located and the adjacent reference lines are not considered in this case.

Main decision here is to check whether the obstacle can be ignored. 

- Obstacle in front of the Ego vehicle is not considered here. This has been taken care in FRONT_VEHICLE.

```
if (path_obstacle->PerceptionSLBoundary().end_s() >= adc_sl_boundary.end_s()) {  // don't ignore such vehicles.
  continue;
}
```

- Ignore is there are no obstacle motion on the reference line
```
if (path_obstacle->reference_line_st_boundary().IsEmpty()) {
  path_decision->AddLongitudinalDecision("backside_vehicle/no-st-region", path_obstacle->Id(), ignore);
  path_decision->AddLateralDecision("backside_vehicle/no-st-region", path_obstacle->Id(), ignore);
  continue;
}
```

- Ignore the obstacle coming behind the Ego vehicle

```
// Ignore the car comes from back of ADC
if (path_obstacle->reference_line_st_boundary().min_s() < -adc_length_s) {  
  path_decision->AddLongitudinalDecision("backside_vehicle/st-min-s < adc", path_obstacle->Id(), ignore);
  path_decision->AddLateralDecision("backside_vehicle/st-min-s < adc", path_obstacle->Id(), ignore);
  continue;
}
```

From the code, if the min_s (that is the distance of obstacle track from the Ego vehicle) is less than half of Ego vehicle's length then the obstacle coming behind the Ego vehicle can be ignored temporarily.


- Ignore the obstacles that would not overtake in future

```
const double lane_boundary = config_.backside_vehicle().backside_lane_width();  // 4m
  if (path_obstacle->PerceptionSLBoundary().start_s() < adc_sl_boundary.end_s()) {
    if (path_obstacle->PerceptionSLBoundary().start_l() > lane_boundary ||
        path_obstacle->PerceptionSLBoundary().end_l() < -lane_boundary) {
      continue;
    }
    path_decision->AddLongitudinalDecision("backside_vehicle/sl < adc.end_s", path_obstacle->Id(), ignore);
    path_decision->AddLateralDecision("backside_vehicle/sl < adc.end_s", path_obstacle->Id(), ignore);
    continue;
  }
}
```

As shown in the code above, firstly it checks for the obstacles which are coming behind the Ego vehicle (At least no more than Ego vehicle). Secondly it calculates the lateral distance between the obstacle and ego vehicle, checks if it exceeds the threshold.
If it exceeds the threshold, then the obstacle can overtake the Ego vehicle. If it is less than the threshold it indicates that the obstacle would not overtake, it follows the Ego vehicle.

  
### 3.2 Change lane processing - CHANGE_LANE

In case of change lane, first step is to find obstacles 




### 3.1 After car situation processing - BACKSIDE_VEHICLE






















     




























































































