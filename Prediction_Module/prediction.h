Header file for prediction.cc

#define MODULES_PREDICTION_PREDICTION_H_  -- For this file

Libraries included :    Proto files from locationlization, perception, planning, prediction
                        modules/common/adapters/proto/adapter_config.pb.h
                        modules/common/proto/pnc_point.pb.h
                        modules/localization/proto/locationlization.pb.h
                        modules/perception/proto/perception_obstacle.pb.h
                        modules/planning/proto/planning.pb.h
                        modules/prediction/prediction_interface.h
                        modules/prediction/proto/prediction_conf.pb.h


Class - Prediction which inherits from PredictionInterface

public
    destructor()
    Name()        -     Get the name of the ros node
    Init()        -     Initialize the ros node
    Start()       -     Start the node
    Stop()        -     Stop the node
    RunOnce()     -     Function runs upon receiving a perception obstacle


private
    Status OnError()    -     
    OnLocalization      -       
    OnPlanning          -   




Private
    start_time = 0
    Create a object "prediction_conf_" of class PredictionConf
    Create a object "adapter_conf_" of class AdapterManagerConfig

check the prediction.cc for function definition
    

