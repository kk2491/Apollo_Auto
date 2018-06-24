
Libraries included  :   modules/common/adapters/adapter_manager.h                    
                        modules/common/math/vec2d.h
                        modules/common/time/time.h
                        modules/common/util/file.h
                        modules/prediction/common/feature_output.h
                        modules/prediction/common/prediction_gflags.h
                        modules/prediction/common/prediction_map.h
                        modules/prediction/common/validation_checker.h
                        modules/prediction/container/container_manager.h
                        modules/prediction/container/obstacles/obstacles_container.h
                        modules/prediction/container/pose/pose_container.h
                        modules/prediction/evaluator/evaluator_manager.h
                        modules/prediction/predictor/predictor_manager.h
                        modules/prediction/proto/prediction_obstacle.pb.h


Function Definition for the functions in prediction.h

Class Prediction

Prediction::Name()
    Params   - None
    Returns  - FLAGS_prediction_module_name   -- Check where this is, might be a macro ?

Predition::Init()
{
    Params    - None
    Returns   - start_time - Calls Clock::NowInSeconds() - Which is there in modules/common/time/time.h (time.cc)
  

    prediction_conf_.Clear();    --- This is a prediction_conf proto class or message (Check where this is)
                             --- This clears the existing message/data
    
    if(! common::util::GetProto
}

                             
  

    
     
