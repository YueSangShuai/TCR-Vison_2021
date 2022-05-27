//
// Created by rmtcr on 2022/5/27.
//

#ifndef RM_PREDICT_H
#define RM_PREDICT_H

#include <ceres/ceres.h>
class predict {
public:
    int tau;                //计算角度差的间隔帧数
    float tao;                  //观测时间
    predict();
private:
    ceres::Problem *problem;        //Ceres库待求解问题
    ceres::Solver::Options options;
};


#endif //RM_PREDICT_H
