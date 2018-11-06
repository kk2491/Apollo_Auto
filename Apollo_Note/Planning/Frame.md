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
































