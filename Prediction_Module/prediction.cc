
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
    
    //prediction_conf_ - Location - /modules/prediction/proto/prediction_conf.proto
    //adapter_conf_ - Location - /modules/common/adapters/proto/adapter_config.proto

    prediction_conf_.Clear();    --- This is a prediction_conf proto class or message (Check where this is)
                             --- This clears the existing message/data
    
    //Below condition GetProtoFromFile(string &File_Name, MessageType *message) returns Bool
    //GetProtoFromFile - Location - modules/common/util/file.h
    //This actually checks below 2 functions 
    //GetProtoFromASCIIFile - This checks if the file is openable and parsable
    //GetProtoFromBinaryFile - This checks if the file is openable and parsable
    
    if (GetProtoFromFile(FLAGS_prediction_conf_file, &prediction_conf_)    //Check the filename - FLAGS_prediction_conf_file
        Failed to load the file
    else
        File loaded - prediction_conf.ShortDebugString()
    
        
    adapeter_conf_.Clear();
    if (GetProtoFromFile(FLAGS_prediction_adapter_config_filename, &adapter_conf_)   //Check the filename - FLAGS_prediction_adapter_config_filename
        Unable to load adapter conf file
    else
        Adapter config file loaded into adapeter_conf_.ShortDebugString() 
        
        
        
        
    //Initialize all managers (Adapter, Container, Evaluator, Predictor)
    
    //Location - /modules/common/adapters/adapeter_config.proto
    //adapter_conf_ -- Object of AdapterManagerConfig
    //Adapter::Init -> This is in adapter_manager.cc 
    AdapterManager::Init(adapter_conf_);
        if adapter_config_.is_ros -> Create a new ros Nodehandle
        Go though the message or topics one by one and enable -- check where Enable**** function is there
    
    //Registers the containers
    //Check : Separate container will be created for each type ?  
    //Location - /modules/prediction/container/container_manager.cc
    //Init -> RegisterContainers() -> check condition -> RegisterContainer() -> CreateContainer()
    ContainerManager::instance()->Init(adapter_conf_)
    
    
    //Location - /modules/prediction/evaluator_manager.cc
    //Go through the obstacles one by one (message obstacle_conf)
        //Check if the obstacle type is defined (obstacle_conf.has_obstacle_type())
            //Message type is in /modules/perception/proto/perception_obstacle.proto
            //(unknown, unknown_movable, unknown_unmovable, pedestrian, bicycle, vehicle)
        //Check if the obstacle evaluator type is defined (obstacle_conf.has_evaluator_type())
            //(RNN, MLP, Cost)
        //Check if the obstacle has status and obstacle_status is OnLane
            //Determine whether it is vehicle, cyclist, pedestrian or unknown
    EvaluatorManager::instance()->Init(prediction_conf_);
        
        
    //Location - /modules/prediction/predictor/predictor_manager.cc
    //Go though the obstacles one by one (message obstacle_conf())
        //Check if the obstacle type is defined (obstacle_conf.has_obstacle_type())
        //Check if the obstacle has predictor type (obstacle_conf.has_predictor_type())
              //Lane_sequence, free_move, regional, move_sequence, empty, single_lane predictor
        //Check the obstacle_type() - Type of the obstacle
              //If vehicle - Check if it ON_LANE or OFF_LANE and return the predictor type
              //If bicycle - Check if it ON_LANE or OFF_LANE and return the predictor type
              //If pedestrian - Just return the predictor type
              //If unknown - Check if it ON_LANE or OFF_LANE and return the predictor type
    PredictorManager::instance()->Init(prediction_conf_);
        
        
    
    
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
      
    
    
      
                 
}

                             
  

    
     
