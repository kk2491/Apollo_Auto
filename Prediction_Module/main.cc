Main program in Prediction module

Libraries   :   gflags/gflags.h
                modules/common/log.h
                ros.h
                prediction.h  - Header file of the predictin.cc
                
                
                /* This calls APOLLO_MAIN function which is in the file apollo_app.h as #define or macro and creates APP class
                and during execution below piece of code is executed
                
                modules/common/apollo_app.h
                                
                This is the int main function
                          it initiates InitGoogleLogging
                                       ParseCommandLineFlags
                                       Signal
                          Creates a object "apollo_app_" of class APP
                          
                          initiates ROS apollo_app_.Name()
                          
                          And does the ros Spin apollo_app_.Spin()
                        
                APOLLO_MAIN(apollo::prediction::Prediction)
                
                
