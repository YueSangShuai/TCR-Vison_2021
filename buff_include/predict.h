//
// Created by rmtcr on 2022/5/27.
//

#ifndef RM_PREDICT_H
#define RM_PREDICT_H
#include<iostream>
#include <ceres/ceres.h>
#include "../header/General.h"
class predict {
public:
    int tau;                //计算角度差的间隔帧数
    float tao;                  //观测时间
    int samples;                //样本数量
    double phi;                 //拟合得到的相位
    double last_phi;//上一帧拟合的相位
    predict();

    double NiHe(double del_time,double del_angle,int tao,int sample);
    Point2f getPredictPoint(RM_BuffData buffs,double predictime,double passtime,int rotation);
    float pos_fun(float t,float phi);
private:
    ceres::Problem *problem;        //Ceres库待求解问题
    ceres::Solver::Options options;
};


#endif //RM_PREDICT_H
