#include"../buff_include/BuffAngleSolver.h"
#include "../header/RemoteController.h"
#include"../DrawCurve/DraCurve.h"
#include "fstream"
#define SHOOT_DELAY_TIME 120                //发弹延迟
#define MIN_SAVE_CIRCLR_CENTER_NUMBER   3          //保留最小的圆心数量
#define MAX_SAVE_CIRCLR_CENTER_NUMBER   75          //保留最大的圆心数量
#define BUFF_R 156
#define RUN_FRAME_BLANK_NUMBER 7                                          //检测记录帧间隔
#define MIN_RUN_ANGLE 3
#define  G 9.98
//最小运动角度,判断当前大小符状态
Point3f CircleCenters[MAX_SAVE_CIRCLR_CENTER_NUMBER];           //圆心保留数组

static Point3f old_center;
Point3f old_Vector;
Point3f Vectors[MAX_SAVE_CIRCLR_CENTER_NUMBER];
float ChassisToBuff_Pitch[MAX_SAVE_CIRCLR_CENTER_NUMBER];
float ChassisToBuff_Yaw[MAX_SAVE_CIRCLR_CENTER_NUMBER];
Point2f last_point;
Point2f now_point;
/**********************角速度滤波**************************/
KF_two KF_pitch;
KF_two KF_yaw;
KF_two KF_angle;
float old_AngleSpeed;
static CarData old_carDatas;
bool isHaveSetBuffKF = false;
DrawCurve draw1;
double last_del_angle=0;
double now_del_angle=0;

bool is_angle_two=false;

BuffAngleSolver::BuffAngleSolver(){
    compute_pitch=0;
    compute_yaw=0;
}
void BuffAngleSolver::PinHole_solver(RM_BuffData& buffdata)
{
    double fx = caremaMatrix.at<double>(0, 0);
    double fy = caremaMatrix.at<double>(1, 1);
    double cx = caremaMatrix.at<double>(0, 2);
    double cy = caremaMatrix.at<double>(1, 2);
    Point2f pnt;
    vector<cv::Point2f> in;
    vector<cv::Point2f> out;
    in.push_back(buffdata.predict);
    //对像素点去畸变
    undistortPoints(in, out, caremaMatrix, distCoeffs, noArray(), caremaMatrix);
    pnt = out.front();
    //去畸变后的比值
    double rxNew = (pnt.x - cx) / fx;
    double ryNew = (pnt.y - cy) / fy;

    buffdata.yaw = atan(rxNew) / CV_PI * 180;
    buffdata.pitch = -atan(ryNew) / CV_PI * 180;
}
/**
 * @brief GetBuffForceResult            //得到大符预测点
 * @param BestArmor
 */
void BuffAngleSolver::GetBuffShootAngle(RM_BuffData* BestArmor,BuffStatus BuffShootStatus,CarData carDatas){
    float AngleSpeed=BestArmor[3].del_angle;
    float del_time=BestArmor[3].del_time;
    PinHole_solver(BestArmor[3]);
}

