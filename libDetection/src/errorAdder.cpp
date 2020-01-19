#include "errorAdder.h"

ErrAdder::ErrAdder(){}
ErrAdder::~ErrAdder(){}

void ErrAdder::modelInit(){

    kf.g_measurement_matrix=(Mat_<float>(2, 4) <<   
            1,0,0,0,   
            0,1,0,0);  
    kf.g_measurement_noise_cov=(Mat_<float>(2, 2) <<   
            2000,0,   
            0,2000   
             );
    kf.g_process_noise_cov=(Mat_<float>(4,4) <<
            10000,0,0,0,
            0,10000,0,0,
	    0,0,10000,0,
	    0,0,0,10000);
    kf.init(4,2,0);
}

void ErrAdder::resetPostState(Mat statepost){
    kf.setStatepost(statepost);
}

Mat ErrAdder::predict(Mat measurement, float dt){
    kf.setSamplingtime(dt);
    kf.correct(measurement);
    return kf.predict();
}
