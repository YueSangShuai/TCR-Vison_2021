#ifndef RM_PREDICT_H
#define RM_PREDICT_H
#include<iostream>
#include <ceres/ceres.h>
#include "../header/General.h"
class ekf {
private:
    double del_angle;       // 测量pitch角度
    double angle_speed;         // 测量yaw角度

public:
    Eigen::VectorXd x_;         //状态向量[锁定目标绝对pitch,锁定目标绝对yaw,v_pitch,v_yaw]
    //状态向量[x,y,v_x,v_y]
    Eigen::VectorXd x_p;
    // 有参数构造函数
    // P_in状态协方差矩阵    Q_in过程噪声矩阵    H_in测量矩阵    R_in测量噪声矩阵
    ekf(Eigen::MatrixXd P_in, Eigen::MatrixXd Q_in, Eigen::MatrixXd H_in, Eigen::MatrixXd R_in);
    // 传入状态矩阵,进行预测部分计算
    Eigen::MatrixXd Prediction(Eigen::MatrixXd _F);
    // 无参数，代表直接使用类内状态向量和状态矩阵相乘
    Eigen::VectorXd GetPrediction();
    // 有参数，代表与传入状态矩阵相乘
    Eigen::VectorXd GetPrediction(Eigen::MatrixXd _F);

    Eigen::MatrixXd F;                           // 状态转移矩阵

    Eigen::MatrixXd P;                          // 状态协方差矩阵
    Eigen::MatrixXd Q;                          // 过程噪声
    Eigen::MatrixXd H;                          // 测量矩阵
    Eigen::MatrixXd R;                          // 测量噪声矩阵

    Eigen::MatrixXd K;                          // 卡尔曼增益

    bool is_set_x = false;                     // 判断是否赋初值

    // 无参数构造函数
    ekf();
    // 状态向量初始化，赋初值
    void set_x(Eigen::VectorXd x, Eigen::MatrixXd _F);
    void set_x(Eigen::VectorXd x);
    // 状态更新
    void update(Eigen::VectorXd z, Eigen::MatrixXd _F);
    // 返回状态向量，获得预测值
    Eigen::VectorXd get_x();
    // void initialize();                                    //初始化
};



#endif //RM_PREDICT_H
