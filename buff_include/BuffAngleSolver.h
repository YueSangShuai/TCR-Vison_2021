#ifndef BUFFANGLESOLVER_H
#define BUFFANGLESOLVER_H
#include"../header/General.h"
#include"../header/Filter.h"
#include "../buff_include/FindBuff.h"
class BuffAngleSolver{
private:
    double compute_pitch;
    double compute_yaw;
    void PinHole_solver(RM_BuffData& buffdata);
    //底盘usb
    cv::Mat caremaMatrix = (cv::Mat_<double>(3, 3) <<
            1281.2415311337063031,-0.3089685211182275,654.0926382032376978,
    0,1283.0023551375293209,510.1706263660812510,
    0,0,1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) <<  -0.0728917229903051, -0.1316730024507187, -0.0024528112888193, -0.0006391130756001 ,0.0);

public:
    BuffAngleSolver();                  //构造函数

    void GetBuffShootAngle(RM_BuffData * BestArmor,BuffStatus BuffShootStatus,CarData carDatas);

};

#endif // BUFFANGLESOLVER_H
