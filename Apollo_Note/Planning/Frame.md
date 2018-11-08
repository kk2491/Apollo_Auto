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

In case of change lane, first step is to find obstacles that are following the Ego vehicle and ones that are tyring to overtake the Ego vehicle.

**Following Ego vehicle ??????**

These obstacle trajectories are the lane change trajectories that effect the Ego vehicle. 
In next planning stage information such as location and speed of obstacles and Ego vehicles would be used. 
For each reference line, Apollo considers obstacles closest to the Ego vehicle that can effect the lane change and sets the obstacle to overtake the Ego vehicle. 

```
// / file in apollo/modules/planning/tasks/traffic_decider/change_lane.cc 
Status ChangeLane::ApplyRule (Frame* const frame, ReferenceLineInfo* const reference_line_info) {
   // If it is a straight street, you don't need to change lanes, then ignore 
  if (reference_line_info-> Lanes (). IsOnSegment ()) {
     return  Status::OK ();
  } // calculate warning obstacles & overtaking obstacles 
  guard_obstacles_. clear ();
  overtake_obstacles_. clear ();
   if (! FilterObstacles (reference_line_info )) {
     return
   Status(common::PLANNING_ERROR, "Failed to filter obstacles");
  }
  // 创建警示障碍物类
  if (config_.change_lane().enable_guard_obstacle() && !guard_obstacles_.empty()) {
    for (const auto path_obstacle : guard_obstacles_) {
      auto* guard_obstacle = frame->Find(path_obstacle->Id());
      if (guard_obstacle &&  CreateGuardObstacle(reference_line_info, guard_obstacle)) {
        AINFO << "Created guard obstacle: " << guard_obstacle->Id();
      }
    }
  }
  // 设置超车标志
  if (!overtake_obstacles_.empty()) {
    auto* path_decision = reference_line_info->path_decision();
    const auto& reference_line = reference_line_info->reference_line();
    for (const auto* path_obstacle : overtake_obstacles_) {
      auto overtake = CreateOvertakeDecision(reference_line, path_obstacle);
      path_decision->AddLongitudinalDecision(
          TrafficRuleConfig::RuleId_Name(Id()), path_obstacle->Id(), overtake);
    }
  }
  return Status::OK();
}

```

Step 1: Overtaking and Warning obstacle calculation

- Obstacle without Trajectory is ignored

```
if (!obstacle->HasTrajectory()) {
  continue;
}
```

- Obstacle in front of Ego vehicle is ignored and has no effect on lane change.

```
if (path_obstacle->PerceptionSLBoundary().start_s() > adc_sl_boundary.end_s()) {
  continue;
}
```

- Mark the obstacle as overtaking obstacle if it is within certain threshold distnace (10 m)

```
if (path_obstacle->PerceptionSLBoundary().end_s() <
        adc_sl_boundary.start_s() -
            std::max(config_.change_lane().min_overtake_distance(),   // min_overtake_distance: 10m
                     obstacle->Speed() * min_overtake_time)) {        // min_overtake_time: 2s
  overtake_obstacles_.push_back(path_obstacle);
}
```

- Ignore if the obstacle speed is very small (less than min_guard_speed) or the obstacele is not on the reference line at the end as they have no efffect on lane change. 

```
if (last_point.v() < config_.change_lane().min_guard_speed()) {
  continue;
}if (!reference_line.IsOnRoad(last_point.path_point())) {
  continue;
}
```

- The last planned point of the obstacle is on the reference line but it exceeds the threhold distance from Ego vehicle. It doesn't have effect on lane change.

```
SLPoint last_sl;
if (!reference_line.XYToSL(last_point.path_point(), &last_sl)) {
  continue;
}if (last_sl.s() < 0 || last_sl.s() > adc_sl_boundary.end_s() + kGuardForwardDistance) {
  continue;
}
```

2. Creating Warning Obstacles

Creating warning obstacle is actually predicting the trajectory of the obstacles in future. The code ```ChangeLane::CreateObstacle``` shows the prediction method of obstacle trajectories.

**Check this ..?????**
The predicted trajectory is spliced on the original trajectory (i.e prediction is performed again after the last trajectory point). 
The assumption of this prediction is that the obstacle is in the form of reference line.

Note:
Predicted Length
The obstacle prediction trajectory focusses on the ```config_.change_lane().guard_distance()``` of 100 m in front of Ego vehicle.

Obstacle Speed Assumption
Within this distance the obstacle speed is considered consistent with the last track point speed ```extend_v```, and would be verified as reference line advances.

Predicted Frequency
The distance between the 2 points is the length of the obstacle, so the relative time difference the 2 points : ```time_delta = kStepDistance / extend_v```


3. Creating obstacle overtaking labels

```
/// file in apollo/modules/planning/tasks/traffic_decider/change_lane.cc
ObjectDecisionType ChangeLane::CreateOvertakeDecision(
    const ReferenceLine& reference_line, const PathObstacle* path_obstacle) const {
  ObjectDecisionType overtake;
  overtake.mutable_overtake();
  const double speed = path_obstacle->obstacle()->Speed();
  double distance = std::max(speed * config_.change_lane().min_overtake_time(),  // 设置变道过程中，障碍物运动距离
                             CONFIG_. change_lane (). min_overtake_distance ()); 
  overtake. mutable_overtake () -> set_distance_s (Distance);   
   Double fence_s = path_obstacle-> PerceptionSLBoundary . () end_s () + Distance; 
   Auto . Point = reference_line GetReferencePoint (fence_s);                / / After setting the lane change, the position of the obstacle on the reference line is 
  overtake. mutable_overtake ()-> set_time_buffer (config_. change_lane (). min_overtake_time ()); // Set the minimum time required for the lane changeOvertake 
  . mutable_overtake ()-> set_distance_s (distance);                // Set the distance the obstacle advances during the lane changeover 
  . mutable_overtake ()-> set_fence_heading (point. heading ()); 
  overtake. mutable_overtake ()-> mutable_fence_point ( )-> set_x (point. x ()); // Set the coordinates of the obstacle after the lane change is completed 
  . mutable_overtake ()-> mutable_fence_point ()-> set_y (point. y ()); 
  overtake. mutable_overtake () ->mutable_fence_point()->set_z(0.0);
  return overtake;
}
```


### 3.3 Crosswalk conditioning - CROSSWALK

According to the **comity** rule, when the pedestrian or non-motor vehicle is far away the Ego vehicle can drive through the Crosswalk.
When someone passes by the Crosswalk, Ego vehicle must stop and let the pedestrian pass. 

```
// / file in apollo/modules/planning/tasks/traffic_decider/crosswalk.cc 
Status Crosswalk::ApplyRule (Frame* const frame, ReferenceLineInfo* const reference_line_info) {
   // Check if there is a crosswalk area 
  if (! FindCrosswalks (reference_line_info)) {
     return  the Status :: the OK ();
  } // do for each obstacle markers, the presence of the obstacle, the vehicle should stop or no direct passing MakeDecisions (Frame, reference_line_info);
   return the Status :: the OK ();
}

```

Step 1: Check that there are obstacles in each crosswalk area that require Ego vehicle to make stop.

- If the Ego vehicle has already passed a part of the crosswalk then ignore, no need to stop.

```
// skip crosswalk if master vehicle body already passes the stop line 
double stop_line_end_s = crosswalk_overlap->end_s;    
 if (adc_front_edge_s - stop_line_end_s > config_.crosswalk().min_pass_s_distance()) {   //The head passes a certain distance from the crosswalk, min_pass_s_distance: 1.0 
  Continue ;
}
```

Go through the obstacles (pedestrians and non-motor vehicles) in each crosswalk aread and expand the crosswalk area to improve safety.

- Ignore if the obstacle is not in extented crosswalk.

```
// expand crosswalk polygon
// note: crosswalk expanded area will include sideway area
Vec2d point(perception_obstacle.position().x(),
            perception_obstacle.position().y());const Polygon2d crosswalk_poly = crosswalk_ptr->polygon();
bool in_crosswalk = crosswalk_poly.IsPointIn(point);
const Polygon2d crosswalk_exp_poly = crosswalk_poly.ExpandByDistance(
         config_.crosswalk().expand_s_distance());bool in_expanded_crosswalk = crosswalk_exp_poly.IsPointIn(point);if (!in_expanded_crosswalk) {
  continue;
}

```

Calculate the lateral distance of the obstacle to the reference line ```obstacle_l_distance```, whether it is on and nearer to road. And whether the obstacle trajectory intersects the reference line ```is_path_cross```.

- If the lateral distance is greater than the loose distance and if the trajectory of the obstacle intersect then stop the Ego vehicle. Otherwise ego vehicle can be driven through the crosswalk as the lateral distance is relatively far. 

```
if (obstacle_l_distance >= config_.crosswalk().stop_loose_l_distance()) {  // stop_loose_l_distance: 5.0m
  // (1) when obstacle_l_distance is big enough(>= loose_l_distance),  
  //     STOP only if path crosses
  if (is_path_cross) {
    stop = true;
  }
}
```

- If the lateral distance is less than the compact distance and if the obstacle trajectory intersects with the reference line then stop the Ego vehicle. 

```
else if (obstacle_l_distance <= config_.crosswalk().stop_strick_l_distance()) { // stop_strick_l_distance: 4.0m
  // (2) when l_distance <= strick_l_distance + on_road(not on sideway),
  //     always STOP
  // (3) when l_distance <= strick_l_distance + not on_road(on sideway),
  //     STOP only if path crosses
  if (is_on_road || is_path_cross) {
    stop = true;
  }
} 

```

- Stop the Ego vehicle if the lateral distance is between the compact distance and loose distance.

```
else {
  // TODO(all)
  // (4) when l_distance is between loose_l and strick_l
  //     use history decision of this crosswalk to smooth unsteadiness
  stop = true;
}
```

If there is an obstacle that requires the Ego vehicle to stop, calculate the acceleration of the Ego vehicle and starts decelerating to stop the Ego vehicle. If the speed of the Ego vehicle can not be slowed down quickly, then pass through the crosswalk.

Formula for calculating the acceleration is 

```a = (v2 − u2 ) / 2s```

```( 0 - v2 ) = 2as ```

s = distance from the current stop to the obstacle. 

```util::GetADCStopDeceleration``` does this calculation.


Step 2: Build virtual wall obstacles and set parking labels for obstacles that effect the trajectory of the Ego vehicle.

A single obstacle is a small frame. Ego vehicle must keep a certain distance from the obstacle during the driving process. If the obstacle is in the center build a Virtual wall with a length of 0.1 and a width of lane to ensure safety. 

```
// create virtual stop wall
std::string virtual_obstacle_id =
      CROSSWALK_VO_ID_PREFIX + crosswalk_overlap->object_id;auto* obstacle = frame->CreateStopObstacle(
      reference_line_info, virtual_obstacle_id, crosswalk_overlap->start_s);if (!obstacle) {
  AERROR << "Failed to create obstacle[" << virtual_obstacle_id << "]";
  return -1;
}
PathObstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
if (!stop_wall) {
  AERROR <<

"Failed to create path_obstacle for: " << virtual_obstacle_id;
  return -1;
}
```

PredictionObstacle is packaged into Obstacle. Create an obstacle using box Box2d and add the parking signs to these virtual walls.

```
// build stop decision
const double stop_s =         // 计算停车位置的累计距离，stop_distance：1m，人行横道前1m处停车
      crosswalk_overlap->start_s - config_.crosswalk().stop_distance();auto stop_point = reference_line.GetReferencePoint(stop_s);
double stop_heading = reference_line.GetReferencePoint(stop_s).heading();
ObjectDecisionType stop;auto stop_decision = stop.mutable_stop();
stop_decision->set_reason_code(StopReasonCode::STOP_REASON_CROSSWALK);
stop_decision->set_distance_s(-config_.crosswalk().stop_distance());
stop_decision->set_stop_heading


(stop_heading);                  // Set the angle/direction of the parking point 
stop_decision-> mutable_stop_point ()->set_x(stop_point.x());     // Set the coordinates of the parking spot 
stop_decision-> mutable_stop_point ()->set_y(stop_point.y ()); 
stop_decision-> mutable_stop_point ()->set_z( 0.0 ); for ( auto pedestrian : pedestrians) { 
  stop_decision-> add_wait_for_obstacle (pedestrian);   // Set the obstacle id that causes the unmanned vehicle to stop 
} auto * path_decision = Reference_line_info-> path_decision (); 
path_decision-> AddLongitudinalDecision



(   
      TrafficRuleConfig::RuleId_Name(config_.rule_id()), stop_wall->Id(), stop);
      
```



### 3.4 Destination condition processing - DESTINATION

When arriving at the destination obstacles that cause the Ego vehicle to take actions are parking on side or finding a suitable parking spot (Small distance from the destination, no need to park). 

Step 1: Main decision logic

- Check if Ego vehicle is in PULL_OVER state, continue to keep the status.

```
auto* planning_state = GetPlanningStatus()->mutable_planning_state();
if (planning_state->has_pull_over() && planning_state->pull_over().in_pull_over()) {
  PullOver(nullptr);
  ADEBUG << "destination: continue PULL OVER";
  return 0;
}
```

- Check if the Ego vehicle needs to go to PULL_OVER status. This depends on the distance between the main reference line and the destination and whether the PULL_OVER is allowed. 

```
const auto& routing = AdapterManager::GetRoutingResponse()->GetLatestObserved();
const auto& routing_end = *routing.routing_request().waypoint().rbegin();
double dest_lane_s = std::max(       // stop_distance: 0.5，目的地0.5m前停车
      0.0, routing_end.s() - FLAGS_virtual_stop_wall_length -
      config_.destination().stop_distance()); 
common::PointENU dest_point;if (CheckPullOver(reference_line_info, routing_end.id(), dest_lane_s, &dest_point)) {
  PullOver(&dest_point);
} else {
  Stop(frame, reference_line_info, routing_end.id
(), dest_lane_s);
}
```

Step 2: CheckPullPver mechanism (Apollo doesn't enable PULL_OVER in destination)

- Return false if the PULL_OVER is disabled in the DESTINATION state.

```
if (!config_.destination().enable_pull_over()) {
  return false;
}
```

- Return false if the destination is not on the reference line.

```
const auto dest_lane = HDMapUtil::BaseMapPtr()->GetLaneById(hdmap::MakeMapId(lane_id));
const auto& reference_line = reference_line_info->reference_line();
// check dest OnRoad
double dest_lane_s = std::max(
    0.0, lane_s - FLAGS_virtual_stop_wall_length -
    config_.destination().stop_distance());
*dest_point = dest_lane->GetSmoothPoint(dest_lane_s);
if (!reference_line.IsOnRoad(*dest_point)) {
  return false;
}
```

- Return false if the Ego vehicle is far away from the destination.

```
// check dest within pull_over_plan_distance
common::SLPoint dest_sl;if (!reference_line.XYToSL({dest_point->x(), dest_point->y()}, &dest_sl)) {
  returnfalse;
}double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
double distance_to_dest = dest_sl.s() - adc_front_edge_s;
// pull_over_plan_distance: 55mif (distance_to_dest > config_.destination().pull_over_plan_distance()) {
  // to far, not sending pull-over yetreturnfalse;
}
 
```


Step 3: Obstacle PULL_OVER and STOP label setting

```
int Destination::PullOver(common::PointENU* const dest_point) {
  auto* planning_state = GetPlanningStatus()->mutable_planning_state();
  if (!planning_state->has_pull_over() || !planning_state->pull_over().in_pull_over()) {
    planning_state->clear_pull_over();
    auto pull_over = planning_state->mutable_pull_over();
    pull_over->set_in_pull_over(true);
    pull_over->set_reason(PullOverStatus::DESTINATION);
    pull_over->set_status_set_time(Clock::NowInSeconds());if (dest_point) {
      pull_over->mutable_inlane_dest_point()->set_x(dest_point->x());
      pull_over->mutable_inlane_dest_point()->set_y(dest_point->y());
    }
  }return0;
}
```

The stop label setting is same as the STOP in the CROSSWALK condition processing. A virtual wall is created and encapsulated into a new PathObstacle added to the PathDecision of the ReferenceLineInfo.



### 3.5 Front car situation handling - FRONT_VEHICLE

There are 2 types of obstacles affecting the decision of the Ego vehicle. 
a. Wait for the opportunity to overtake or follow the obstacle based on the obstacle information. 
b. Ego vehicle has to stop if the obstacle is static. 

a:
Follow the obstacle and wait for the opportunity to overtake. 

Overtaking defined here requires the Ego vehice to adjust the lateral distance and perform overtake. ** The direct driving through the obstacle in front is normal driving and obstacle can be ignored in this case (But WHY ?????**. 

There are 4 steps to complete the overtake behavior.

- Normal drive (DRIVE)
- Waiting for overtaking (WAIT)
- Overtaking (SIDEPASS)
- Normal driving (DRIVE)  

If the distance from Obstalce is too far then Ego vehice status would be in DRIVE. 
If the distance is relatively close, and overtaking condition is not met then Ego vehicle would follow the obstacle.  
If the distance is too close and the speed is small, then Ego vehicle would WAIT.
If the overtaking conditions are met, then Ego vehicle would go to OVERTAKE status.
Once the overtaking is completed, return to normal DRIVE status. 


Step 1:
Check condition overtaking - FrontVehicle::FindPassableObstacle

This checks for the obstacle that affect the normal driving of the Ego vehicle (ADC may need to overtake). Go through the PathObstacle in PathDecision and then check the following conditions. 

- Check if the obstacle is virtual or static. If so then depending on the condition Ego vehicle may have to stop. Overtaking strategy doesnt come here. 

```
if (path_obstacle->obstacle()->IsVirtual() || !path_obstacle->obstacle()->IsStatic()) {
  ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
        << "] VIRTUAL or NOT STATIC. SKIP";
  continue;
}
```


- Check the position of the obstacle and Ego vehicle. Obstacles can be ignored if they fall behind the ego vehicle. 

```
const auto& obstacle_sl = path_obstacle->PerceptionSLBoundary();
if (obstacle_sl.start_s() <= adc_sl_boundary.end_s()) {
  ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
         << "] behind ADC. SKIP";
  continue;
}
```

- Check the horizontal and vertical distance between the obstacle and Ego vehicle. If the logitudinal and vertical distance are too far then those obstacles can be ignored and Ego vehicle can stay in normal driving status. 

```
const double side_pass_s_threshold = config_.front_vehicle().side_pass_s_threshold(); // side_pass_s_threshold: 15m
if (obstacle_sl.start_s() - adc_sl_boundary.end_s() > side_pass_s_threshold) {
  continue;
}constdouble side_pass_l_threshold = config_.front_vehicle().side_pass_l_threshold(); // side_pass_l_threshold: 1mif (obstacle_sl.start_l() > side_pass_l_threshold || 
    obstacle_sl.end_l() < -side_pass_l_threshold) {continue;
}

```

```FrontVehicle::FindPassableObstacle``` return the first obstacle found to limit the driving of the Ego vehicle.

Step 2:
Set the signs for each stage of Overtaking ```FrontVehicle::ProcessSidePass```

- If the previous stage is in SidePassStatus::UNKNOWN state, set it to normal driving DRIVE. 

- If the previous state is in SidePassStatus::DRIVE and there is an obstacle blocking the path of Ego vehicle, then set the state WAIT. If no obstacles then continue normal driving DRIVE.

```
case SidePassStatus::DRIVE: {
  constexpr double kAdcStopSpeedThreshold = 0.1;  // unit: m/s
  const auto& adc_planning_point = reference_line_info->AdcPlanningPoint();
  if (!passable_obstacle_id.empty() &&            // 前方有障碍物则需要等待
      adc_planning_point.v() < kAdcStopSpeedThreshold) {
    sidepass_status->set_status(SidePassStatus::WAIT);
    sidepass_status->set_wait_start_time(Clock::NowInSeconds());
  }break;
}
```

- If the previous state is SidePassStatus::WAIT then based on the situation below status change would be done.

a. If there is no obstruction, set to Normal driving DRIVE.

b. If there is an obstacle in the front of Ego vehicle, start the WAIT time. If it exceeds the threshold find the left and right lanes for effective lanes overtaking. If no effective lanes present then stay in WAIT state.

```
Double wait_start_time = sidepass_status-> wait_start_time ();   
 double wait_time = Clock::NowInSeconds() - wait_start_time;   // calculate the waited time 

if (wait_time > config_.front_vehicle().side_pass_wait_time()) { // exceed the threshold, look for Other lanes are overtaking. Side_pass_wait_time:30s 
}
```

First query the HDMap to get the lane where the current ReferenceLine is located. If there is a lane, set the overtaking status. If there is no lane check the right lane. 
If the right lane is CITY_DRIVING (motorway, not non-motor vehicle lane or pedestrian street or parking) then proceed with overtaking with right lane.

```
if (enter_sidepass_mode) {
  sidepass_status->set_status(SidePassStatus::SIDEPASS);
  sidepass_status->set_pass_obstacle_id(passable_obstacle_id);
  sidepass_status->clear_wait_start_time();
  sidepass_status->set_pass_side(side);   // side知识左超车还是右超车。取值为ObjectSidePass::RIGHT或ObjectSidePass::LEFT
}
```

- If the previous state is SidePassStatus::SIDEPASS and if there is no obstacle present then set to Normal driving DRIVE. Else continue the state in overtaking process. (Why ?????)

```
case SidePassStatus::SIDEPASS: {
  if (passable_obstacle_id.empty()) {
    sidepass_status->set_status(SidePassStatus::DRIVE);
  }break;
}

```


2. Parking Treament (Stop the Ego vehicle)

- Check if the obstacle is stationary object. Ignore if it is virtual or dynamic obsacle, these will be taken care by Overtaking module.

```
if (path_obstacle->obstacle()->IsVirtual() || !path_obstacle->obstacle()->IsStatic()) {
  continue;
}
```

-  Get the position of obstacle wrt Ego vehicle, Ignore if the obstacle is behind the Ego vehicle. This will be taken care by Backside vehicle module.

```
const auto& obstacle_sl = path_obstacle->PerceptionSLBoundary();
if (obstacle_sl.end_s() <= adc_sl.start_s()) {
    continue;
}
```

- If the obstacle is marked in Overtaking module (which makes the Ego vehicle to overtake the obstacle) then there is no need to consider the obstacle at this time. 

```
// check SIDE_PASS decision
if (path_obstacle->LateralDecision().has_sidepass()) {
  continue;
}
```

- Build stop decision if the driveaway doesn't have enough width to allow the Ego vehicle to overtake the obstacle.

```
Double left_width = 0.0 ;
 double right_width = 0.0 ; 
reference_line.GetLaneWidth(obstacle_sl.start_s(), &left_width, &right_width); double left_driving_width = left_width - obstacle_sl.end_l() -            // calculate the left-side free distance of the obstacle 
                                config_.front_vehicle() .nudge_l_buffer(); double right_driving_width = right_width + obstacle_sl.start_l() -        // Calculate the free distance to the right of the obstacle. The + sign is because the left side of the lane line FLU coordinate system is the negative axis, and the right side is the positive axis 
                                 config_.front_vehicle( ).nudge_l_buffer(); if ((left_driving_width < adc_width && right_driving_width < adc_width) ||




        (obstacle_sl.start_l() <= 0.0 && obstacle_sl.end_l() >= 0.0)) {
  // build stop decision
  double stop_distance = path_obstacle->MinRadiusStopDistance(vehicle_param);
  const double stop_s = obstacle_sl.start_s() - stop_distance;
  auto stop_point = reference_line.GetReferencePoint(stop_s);
  double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;auto stop_decision = stop.mutable_stop();
  
  if (obstacle_type == PerceptionObstacle::UNKNOWN_MOVABLE ||
      obstacle_type == PerceptionObstacle::BICYCLE ||
      obstacle_type == PerceptionObstacle::VEHICLE) {
    stop_decision->set_reason_code(StopReasonCode::STOP_REASON_HEAD_VEHICLE);
  } else {
      stop_decision->set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);
  }
  stop_decision->set_distance_s(-stop_distance);
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);
  path_decision->AddLongitudinalDecision("front_vehicle", path_obstacle->Id(), stop);
}

```


### 3.6 Forbidden zone situation handling - KEEP_CLEAR

Forbidden zone is divided in to 2 categories
a. No stop zone
b. Intersection

The approach is to build a forbidden zone on reference line from start_s to end_s (start_s and end_s are the projection points of forbidden zone start_s and end_s on reference line). Width of no stop zone is the road width of the reference line.
 

1. Ignore if the Ego vehicle has already entered the no-stop zone or intersection. 

```
// check
const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
if (adc_front_edge_s - keep_clear_overlap->start_s >
      config_.keep_clear().min_pass_s_distance()) { // min_pass_s_distance：2.0m
  return false;
}
```

2. Create new keep clear zone obstacles and mark them as KEEP_CLEAR

```
// create virtual static obstacle
auto* obstacle = frame->CreateStaticObstacle(
  reference_line_info, virtual_obstacle_id, keep_clear_overlap->start_s,
  keep_clear_overlap->end_s);if (!obstacle) {
  returnfalse;
}auto* path_obstacle = reference_line_info->AddObstacle(obstacle);
if (!path_obstacle) {
  returnfalse;
}
path_obstacle->SetReferenceLineStBoundaryType(StBoundary::BoundaryType::KEEP_CLEAR);
```

Here is an additional supplement to the process of creating obstacles in the forbidden / KEEP_CLEAR zone, mainly to calculate the calibration box for obstacles in the forbidden zone. (namely center, length and width)

```

/// file in apollo/modules/planning/common/frame.cc
const Obstacle *Frame::CreateStaticObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id,
    const double obstacle_start_s,
    const double obstacle_end_s) {
  const auto &reference_line = reference_line_info->reference_line();
  // 计算禁停区障碍物start_xy，需要映射到ReferenceLine
  common::SLPoint sl_point;
  sl_point.set_s(obstacle_start_s);
  sl_point.set_l(0.0); 
  common::math::Vec2d obstacle_start_xy; if (!reference_line. SLToXY (sl_point, &obstacle_start_xy)) {
     return nullptr ; 
  } // calculate the forbidden zone obstacle end_xy, which needs to be mapped to ReferenceLine 
  sl_point. set_s (obstacle_end_s); 
  sl_point . set_l ( 0.0 ); 
  Common Math :: :: Vec2d obstacle_end_xy; IF (reference_line!. SLToXY {(sl_point, & obstacle_end_xy))
     return nullptr a ; 
  } // left and right obstacle width width calculating parking area, and the reference Line consistent double left_lane_width = 0.0 ;
  
   
  
   
  
  Double right_lane_width = 0.0 ;
   if (!reference_line. GetLaneWidth (obstacle_start_s, &left_lane_width, &right_lane_width)) {
     return  nullptr ; 
  } //The final calibration box for the obstacle in the forbidden zone 
  common::math::Box2d obstacle_box{ common::math LineSegment2d :: (obstacle_start_xy, obstacle_end_xy), 
      left_lane_width + right_lane_width}; // CreateStaticVirtualObstacle function is packaged into the parking area PathObstacle obstacle placed in PathDecision 
      return CreateStaticVirtualObstacle (obstacle_id, obstacle_box); 
}


```


### 3.7 Finding parking status - PULL_OVER

Finding a Parking spot is essential to perform the PULL_OVER. If the current state is PULL_OVER and it is already in parking location, then only the status needs to be updated.
If there is no parking location found, then it needs to be calculated. 
Once parking location is found build a parking area obstacle, then create PULL_OVER label for the obstacle.

```
Status PullOver::ApplyRule (Frame* const frame, ReferenceLineInfo* const reference_line_info) { 
  frame_ = frame; 
  reference_line_info_ = reference_line_info; if (! IsPullOver ()) {
     return Status::OK (); 
  } // Check if the time is PULL_OVER If it is PULL_OVER, then there is already a stop point stop_point if ( CheckPullOverComplete ()) {
     return Status::OK (); 
  } 
  common::PointENU stop_point; if ( GetPullOverStop (&stop_point) != 0 ) {    //
   
  
   
  Failed to get the parking location, the unmanned vehicle will stop at the stop lane 
    BuildInLaneStop (stop_point); 
    ADEBUG << " Could not find a safe pull over point. STOP in-lane " ; 
  } else {
     BuildPullOverStop (stop_point);            / / Get successful parking position, no pull over to park the vehicle in the lane from the finish 
  } return the Status :: the OK (); 
}

```

1. Get parking spot - ```GetPullOverStop()``` function

- If the parking location / point is already updated in the status information. Then it can be used directly. 

```
if (pull_over_status.has_start_point() && pull_over_status.has_stop_point()) {
    // reuse existing/previously-set stop point
    stop_point->set_x(pull_over_status.stop_point().x());
    stop_point->set_y(pull_over_status.stop_point().y());
}
```


- If it doesn't exist, then parking location needs to be calculated using ```FindPullOverStop``` function

**Why why why ?????** 

The parking location needs to be in front of the destination point ```PARKING_SPOT_LONGITUDINAL_BUFFER``` (default 1m) and ```buffer_to_boundary``` stop at the drive test (default 0.5m).
Ego vehicle is not allowed to PULL_OVER if the lane on the right side of the current lane is motorway (CITY_DRIVING). The method used by Apollo is **Sampling Detection**.
From the front to the end position, a parking condition is checked every kDistanceUnit (default 5m) and if it is satisfied, then Ego vehicle stops directly on lane.

```
int PullOver::FindPullOverStop(PointENU* stop_point) {
  const auto& reference_line = reference_line_info_->reference_line();
  const double adc_front_edge_s = reference_line_info_->AdcSlBoundary().end_s();double check_length = 0.0;
  double total_check_length = 0.0;
  double check_s = adc_front_edge_s;      // check_s为当前车辆车头的累计距离constexprdoublekDistanceUnit = 5.0;
  while (check_s < reference_line.

  

    Length() &&    // 在当前车道上，向前采样方式进行停车位置检索，前向检索距离不超过max_check_distance(默认60m)
      total_check_length < config_.pull_over().max_check_distance()) {
    check_s += kDistanceUnit;
    total_check_length += kDistanceUnit;
    ...
  }
}

```

Check the right lane of the point (check_s), if the right lane is CITY_DRIVING then ego vehicle can't be stopped without changing the lane. Ego vehicle needs to change the lane and continue the forward search. 

```
// check rightmost driving lane:
//   NONE/CITY_DRIVING/BIKING/SIDEWALK/PARKING
bool rightmost_driving_lane = true;
for (auto& neighbor_lane_id : lane->lane().right_neighbor_forward_lane_id()) {   // 
  const auto neighbor_lane = HDMapUtil::BaseMapPtr()->GetLaneById(neighbor_lane_id);
  ...constauto& lane_type = neighbor_lane->lane().type();
  if (lane_type == hdmap::Lane::CITY_DRIVING) {   //
    
    rightmost_driving_lane = false;
    break;
  }
}if (!rightmost_driving_lane) {
  check_length = 0.0;
  continue;
}

```

If the right lane is not a CITY_DRIVING then ego vehicle can perform PULL_OVER. 
Parking location needs to be in front of the destination point ```PARKING_SPOT_LONGITUDINAL_BUFFER``` (default 1m) and ```buffer_to_boundary``` stop.

The distance between the logitudinal direction and the parking point is based on front of the vehicle. 
**What is this Why Why Why ???**
The distance between the side and the parking point is taken as the reference from the minimum distance between the head of the lane and lane edge of the vehicle.

```
// all the lane checks have passed
check_length += kDistanceUnit;
if (check_length >= config_.pull_over().plan_distance()) {
  PointENU point;// check corresponding parking_spotif (FindPullOverStop(check_s, &point) != 0) {
    // parking_spot not valid/available
    check_length = 0.0;
    continue;
  }
  stop_point->set_x(point.x());
  stop_point->set_y(point.y());
  return0; 
}

```

2. If the parking location is found in previous step (1), then constuct a PathObstacle at the parking location and set the label as STOP. This is done by ```BuildPullOverStop``` function.

3. If the parking location is not found in (1), then force the Ego vehicle to make stop on lane. this is done by function ```BuildInLaneStop```.

- First look for historical data ```inlane_dest_point``` whether historical data allows parking on lane

- If ```inlane_dest_point``` not found, then search for parking location and PULL_OVER if there is parking spot available.

- If parking spot is not found, look for the used inlane point in ```inlane_adc_position_stop_point_```, then park the Ego vehicle.

- If the parking spot is still not found, then the Ego vehicle needs to make forced stop at the end of ```plan_distance``` on lane. Update ```inlane_adc_position_stop_point_```.


### 3.8 Reference Line End processing - REFERENCE_LINE_END

When the reference line ends, it is necessary to stop the Ego vehicle and re-route the routing query. In normal scenario, if the Reference line ends then there is no road ahead. And new routing needs to be updated to reach the destination. 

Construct a stop barrier before the end of the Reference line and set the label to STOP.

```
Status ReferenceLineEnd::ApplyRule(Frame* frame, ReferenceLineInfo* const reference_line_info) {
  const auto& reference_line = reference_line_info->reference_line();
  // 检查参考线剩余的长度，足够则可忽略这个情况，min_reference_line_remain_length：50m
  double remain_s = reference_line.Length() - reference_line_info->AdcSlBoundary().end_s();
  if (remain_s > config_.reference_line_end().min_reference_line_remain_length()) {
    return Status::OK();
  }//
   create avirtual stop wall at the end of reference line to stop the adc
  std::string virtual_obstacle_id =  REF_LINE_END_VO_ID_PREFIX + reference_line_info->Lanes().Id();
  double obstacle_start_s = reference_line.Length() - 2 * FLAGS_virtual_stop_wall_length; // 在参考线终点前，创建停止墙障碍物
  auto* obstacle = frame->CreateStopObstacle(reference_line_info, virtual_obstacle_id, obstacle_start_s);
  if (!obstacle) {
    return Status(common::PLANNING_ERROR, "Failed to create reference line end obstacle");
  }
  PathObstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    return Status(
        common::PLANNING_ERROR, "Failed to create path obstacle for reference line end obstacle");
  }// build stop decision，设置障碍物停止标签constdouble stop_line_s = obstacle_start_s - config_.reference_line_end().stop_distance();
  auto stop_point = reference_line.GetReferencePoint(stop_line_s);
  ObjectDecisionType stop;auto

  
   
   stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(StopReasonCode::STOP_REASON_DESTINATION);
  stop_decision->set_distance_s(-config_.reference_line_end().stop_distance());
  stop_decision->set_stop_heading(stop_point.heading());
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(TrafficRuleConfig::RuleId_Name(config_.rule_id()), stop_wall->Id(), stop);
  returnStatus::OK();
}

```


### 3.9 Rerouting Query Processing - REROUTING (Full of doubts!!!)

Based on the road condition, it can be divided into following scenarios.

- If the current reference line is straight and it is not a turn. Then there is no need to reroute and wait for new route.

- If the Ego vehicle is not on current reference line, then no rerouting required **Why ??????**

- If the current reference line can exit, no rerouting is required.

- If the current channel passage endpoint is not on reference line, no routing is required, waiting for a new route.

- If the end of the reference line is too far away from the Ego vehicle, no rerouting required.

- If the route query is performed last time, the time gap is less than the threshold, rerouting is not required and the new route is awaited.

- In other cases, manually initiate routing query requirements.  

**
a) ```Frame::Rerouting``` - Task done by the code is generate a new route from current position to the Destination. 
This function doesn't generate a new reference line because the reference line is generated by the ReferenceLineProvider class.

b) Rerouting would be of no use if Ego vehicle is re-routed but it is not on reference line.

c) It is better to wait for the ReferenceLineProvider to appply for rerouting and generate corresponding reference line. Therefore, the focus of 2, 3, 4 and so on is the lack of reference lines, not the positional deviation.**



### 3.10 Signal condition processing - SIGNAL_LIGHT

1. Check if there is a signal area under the current road conditions ```FindValidSignalLight```

```
signal_lights_from_path_.clear();
for (const hdmap::PathOverlap& signal_light : signal_lights) {
  if (signal_light.start_s + config_.signal_light().min_pass_s_distance() >
        reference_line_info->AdcSlBoundary().end_s()) {
    signal_lights_from_path_.push_back(signal_light);
  }
}
```

2. Access to information signals ```TrafficLight Perception``` using ```ReadSignals``` function.

```
const TrafficLightDetection& detection =
      AdapterManager::GetTrafficLightDetection()->GetLatestObserved();
for (int j = 0; j < detection.traffic_light_size(); j++) {
  const TrafficLight& signal = detection.traffic_light(j);
  detected_signals_[signal.id()] = &signal;
}
```


3. Take decision based on the current signal status using ```MakeDecisions``` function.

```
For ( auto & signal_light : signal_lights_from_path_) {
     // 1. If the signal light is red and the acceleration is not very large 
    // 2. If the signal light is unknown and the acceleration is not very large 
    // 3. If the signal light is yellow and the acceleration Not very big 
    / / In the above three cases, the car is parked, the parking tag is consistent with the front 
    if (( signal . color () == TrafficLight::RED && 
         stop_deceleration < config_. signal_light (). max_stop_deceleration ()) || 
        ( Signal . color () == TrafficLight::UNKNOWN && 
         stop_deceleration < config_. signal_light (). max_stop_deceleration ()) ||
        ( signal . color ( ) == TrafficLight::YELLOW && 
         stop_deceleration < config_. signal_light (). max_stop_deacceleration_yellow_light ())) {
       if ( BuildStopDecision (frame, reference_line_info, &signal_light)) { 
        has_stop = true ; 
        signal_debug-> set_is_stop_wall_created ( true ) ; 
      } 
    } // Set the intersection area, and whether there is power to pass, parking means no access. If (has_stop) { 
      reference_line_info-> SetJunctionRightOfWay (signal_light. start_s
    
    ,
                                                 false);  // not protected
    } else {
      reference_line_info->SetJunctionRightOfWay(signal_light.start_s, true);
      // is protected
    }
  }
  
```


### 3.11 Parking condition processing - STOP_SIGN

This can be divided into 2 parts : Find the nearest parking signal and decision processing.
Finding the next parking spot (stop sign) is done by function ```FindNextStopSign```. 
Next step is decision making is divided into following steps:

1. Get a list of waiting vehicles - done by function ```GetWatchVehicles```.

This function fetches the waiting vehicles in front of the Ego vehicle. Storage form is :
```  typedef std::unordered_map<std::string, std::vector<std::string>> StopSignLaneVehicles;
```

The first in the map ```string``` is the lane id, and the second ```vector<string>``` is the waiting vehicle ID in front of the Ego vehicle on this lane. The overall query is directly obtained in the ```PlanningStatus.stop_sign()``` (parking_state).
The first time it is empty, and the subsequent is not empty **WHY ?????**

```
int StopSign::GetWatchVehicles(const StopSignInfo& stop_sign_info,
                               StopSignLaneVehicles* watch_vehicles) {
  watch_vehicles->clear();
  StopSignStatus stop_sign_status = GetPlanningStatus()->stop_sign();
  // 遍历所有的车道
  for (int i = 0; i < stop_sign_status.lane_watch_vehicles_size(); ++i) {
    auto lane_watch_vehicles = stop_sign_status.lane_watch_vehicles(i);
    std::string associated_lane_id = lane_watch_vehicles.lane_id();
    std::string s;// 获取每个车道的等候车辆for (int j = 0; j < lane_watch_vehicles.watch_vehicles_size(); ++j) {
      std::string vehicle = lane_watch_vehicles.watch_vehicles(j);
      s = s.empty() ? vehicle : s + "," + vehicle;
      (*watch_vehicles)[associated_lane_id].push_back(vehicle);
    }
  }return0;
}

```

2. Check and update the parking status (stop status) ```PlanningStatis.stop_sign``` - Function used is ```ProcessStopStatus```.

The Parking process (stop) can be divided into 5 stages:
Normal Drive - DRIVE
Sttarting Stop - SOP
Waiting for buffering status - WAIT
Slowing forward - CREEP
Stop completed - DONE


- If the Ego vehicle is too far from the nearest Stop sign (parking area) then update the status as DRIVE.

```
// adjust status
double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
double stop_line_start_s = next_stop_sign_overlap_.start_s;
if (stop_line_start_s - adc_front_edge_s >  // max_valid_stop_distance: 3.5m
      config_.stop_sign().max_valid_stop_distance()) {
  stop_status_ = StopSignStatus::DRIVE;
}

```

- If the parking status (stop signal status) is normal driving DRIVE.
In this case if the Ego vehicle is too large or too far from the stop sign, then continue in the DRIVE status. Otherwise enter the stop state STOP, the status check ```CheckADCkStop``` completed. 


- If the parking status is starting to stop STOP.
In this case, if the waiting time from the start of the stop to the current time does not exceed the threshold stop_duration (default 1s), the STOP state continues to be maintained. Conversely, if the vehicle waiting in front is not empty, then it will enter the next stage WAIT buffer phase; if the vehicle ahead is empty, then you can directly enter the slow forward CREEP state or the parking completion state.

- If the parking status is waiting for buffering WAIT
In this case, if the waiting time does not exceed a threshold wait_timeout (default 8s) or there is a waiting vehicle ahead, continue to wait. On the contrary, you can enter the slow forward or stop state.

- If the parking status is slow forward CREEP
In this case, it is only necessary to check the distance between the front of the unmanned vehicle and the parking area. If it is greater than a certain value, it means that it can continue to move slowly, keep the state unchanged, and vice versa.

3. Update ahead waiting vehicle
a. The current state is DRIVE, then you need to add obstacles to the front waiting for the list of vehicles, because these obstacles will be waiting in front of the unmanned vehicles.

```
if (stop_status_ == StopSignStatus::DRIVE) {
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    // add to watch_vehicles if adc is still proceeding to stop sign
    AddWatchVehicle(*path_obstacle, &watch_vehicles);
  }
}
```


b. If the current state of the unmanned vehicle is waiting or stopping, delete part of the queue to wait for the vehicle - ```RemoveWatchVehicle``` function completed

In this case, if the obstacle has passed the parking area, delete it; otherwise continue to retain.

```
Double stop_line_end_s = over_lap_info-> lane_overlap_info ().end_s();
 double obstacle_end_s = obstacle_s + perception_obstacle.length() / 2 ;
 double distance_pass_stop_line = obstacle_end_s - stop_line_end_s; // If the obstacle has
 driven
 a certain distance through the parking area, the obstacle can be Things are removed from waiting for the vehicle. If (distance_pass_stop_line > config_.stop_sign().min_pass_s_distance() && !is_path_cross) { 
  erase = true ; 
} else {
   // passes associated lane (in junction) 
  if (!is_path_cross) { 
    erase = true ; 
  } 
} //
 check if obstacle stops
if (erase) {
  for (StopSignLaneVehicles::iterator it = watch_vehicles->begin();
         it != watch_vehicles->end(); ++it) {
    std::vector<std::string>& vehicles = it->second;
    vehicles.erase(std::remove(vehicles.begin(), vehicles.end(), obstacle_id), vehicles.end());
  }
}

```


- Recompose the remaining obstacles into a new waiting queue -- the ```ClearWatchVehiclefunction``` is complete.

```
for (Iterator IT StopSignLaneVehicles :: = watch_vehicles-> the begin (); 
       ! IT = watch_vehicles-> End ();
        / * NO INCREMENT * / ) { 
  STD :: Vector <STD :: String> = IT- & vehicle_ids> SECOND ;
   // Clean obstacles not in Current Perception 
  for ( Auto obstacle_it = vehicle_ids. the begin (); obstacle_it = vehicle_ids!. End ();) {
     // If the new queue no longer exists the obstacle, then directly to the obstacle Removes 
    if (obstacle_ids. count (*obstacle_it) == 0 ) from this lane {   
      obstacle_it = vehicle_ids.the ERASE (obstacle_it); 
    } the else { 
      ++ obstacle_it; 
    } 
  } IF (. vehicle_ids empty ()) {   // if this does not exist on the entire lane waiting vehicles, and direct these lanes delete 
    watch_vehicles-> the ERASE (IT ++ ); 
  } else { 
    ++it; 
  } 
}
```

  
- Update vehicle status PlanningStatus.stop_sign
This part is ```UpdateWatchVehicles``` done by the function , mainly to update the new waiting vehicle queue obtained in 3 to stop_sign.


```
int StopSign::UpdateWatchVehicles(StopSignLaneVehicles* watch_vehicles) {
  auto* stop_sign_status = GetPlanningStatus()->mutable_stop_sign();
  stop_sign_status->clear_lane_watch_vehicles();for (auto it = watch_vehicles->begin(); it != watch_vehicles->end(); ++it) {
    auto* lane_watch_vehicles = stop_sign_status->add_lane_watch_vehicles();
    lane_watch_vehicles->set_lane_id(it->first);
    std::string s;for (size_t

  
     i = 0; i < it->second.size(); ++i) {
      std::string vehicle = it->second[i];
      s = s.empty() ? vehicle : s + "," + vehicle;
      lane_watch_vehicles->add_watch_vehicles(vehicle);
    }
  }return0;
}

```

   
c. If the current vehicle status is a slow forward state CREEP

In this case, you can create a parking tag directly.

Finally, the impact of obstacles and road conditions on the decision-making of unmanned vehicles is divided into two categories, one is LongitudinalDecision, and the other is LateralDecision.

Vertical impact:

```
const STD unordered_map :: <ObjectDecisionType :: ObjectTagCase, int , athObstacle :: ObjectTagCaseHash> 
    PathObstacle s_longitudinal_decision_safety_sorter_ :: = { 
        {ObjectDecisionType :: kIgnore , 0 },       // ignored, priority 0 
        {ObjectDecisionType :: kOvertake , 100 },   / / Overtaking, priority 100 
        {ObjectDecisionType:: kFollow , 300 },     // Follow, priority 300 
        {ObjectDecisionType:: kYield , 400 },      // Deceleration, priority 400
        {ObjectDecisionType:: kStop , 500 }};      // Parking, priority 500
```

Lateral impact:

```
const STD unordered_map :: <ObjectDecisionType :: ObjectTagCase, int , PathObstacle :: ObjectTagCaseHash> 
    PathObstacle s_lateral_decision_safety_sorter_ :: = { 
        {ObjectDecisionType :: kIgnore , 0 },       // ignored, priority 0 
        {ObjectDecisionType :: kNudge , 100 },      / / fine-tuning, priority 100 
        {ObjectDecisionType:: kSidepass , 200 }}; // bypass, priority 200

```


What should I do when there is an obstacle to make an unmanned vehicle decision multiple times in 11 road conditions?

Longitudinal decision making, lhs is the first decision, rhs is the second decision, how to combine the two decisions


```
ObjectDecisionType PathObstacle::MergeLongitudinalDecision(
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }constauto lhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, lhs.object_tag_case());
  constauto rhs_val =
      
  
    FindOrDie (s_longitudinal_decision_safety_sorter_, rhs. object_tag_case ());
   if (lhs_val < rhs_val) {         // prioritize the decision with a higher priority 
    return rhs; 
  } else  if (lhs_val > rhs_val) {
     return lhs; 
  } else {
     if (lhs. has_ignore ()) {
       return rhs; 
    } else  if (lhs. has_stop ()) {     // If the priorities are the same, they are all parking, choose the decision to stop the parking distance, prevent the safety accident 
      return lhs. stop (). distance_s () < Rhs.STOP . () distance_s ? () lhs: rhs; 
    } the else  IF (lhs. has_yield ()) {    // If the same priority, are decelerating, select the deceleration distance small decisions, prevent accidents 
      return lhs. yield () . distance_s . () <rhs yield . () distance_s ? () lhs: rhs; 
    } the else  IF (lhs. has_follow ()) {   // If the same priority, are to follow, choose to follow a small distance from the decision-making to prevent security Accident 
      return lhs. follow (). distance_s () < rhs. follow (). distance_s () ? lhs : rhs; 
    }the else  IF (lhs. has_overtake ()) { // If the same priority, are overtaking, overtaking selected from the large decisions, prevent accidents 
      return lhs. overtake (). distance_s ()> rhs. overtake (). distance_s ( Lhs : rhs; 
    } else {
       DCHECK ( false ) << " Unknown decision " ; 
    } 
  } return lhs;   // stop compiler complaining 
}

```
  
Lateral consolidation, lhs is the first decision, rhs is the second decision, how to combine the two decisions

```
ObjectDecisionType PathObstacle::MergeLateralDecision(
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }constauto lhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, lhs.object_tag_case());
  constauto rhs_val =
      FindOrDie
  
    (s_lateral_decision_safety_sorter_, RHS. object_tag_case ());
   IF (lhs_val <rhs_val) {          // prefers the priority decision large        
    return RHS; 
  } the else  IF (lhs_val> rhs_val) {
     return LHS; 
  } the else {
     IF (LHS. has_ignore ( ) || lhs. has_sidepass ()) {
       return rhs; 
    } else  if (lhs. has_nudge ()) {                         // If the priorities are the same, they are fine-tuned, and the side-by-side fine-tuning decision is made to 
      return  std::fabs (lhs. Nudge().distance_l()) >
                     std::fabs(rhs.nudge().distance_l())
                 ? lhs
                 : rhs;
    }
  }return lhs;
}

```

