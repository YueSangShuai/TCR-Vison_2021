#ifndef BUFFANGLESOLVER_H
#define BUFFANGLESOLVER_H
#include"../header/General.h"
#include"../header/Filter.h"
#include "../buff_include/FindBuff.h"
class BuffAngleSolver{
private:
    float BuffWidth;
    float BuffHeight;
    float ChassisToPtz_x;
    float ChassisToPtz_y;
    float ChassisToPtz_z;
    float ChassisToPtz_Pitch;
    float ChassisToPtz_Yaw;
    float ChassisToPtz_Roll;

    void GetPoint2D( RM_BuffData & BestArmor,std::vector<cv::Point2f>&point2D);
    void GetPoint3D( RM_BuffData & BestArmor,std::vector<cv::Point3f>&point3D);
    void CountAngleXY(const std::vector<cv::Point2f>&point2D,const std::vector<cv::Point3f>&point3D, RM_BuffData & BestArmor);
    void GetBuffrAngle(RM_BuffData & BestArmor);
    float ShootAdjust(double x, double y, double z, float pitch, float yaw);
    Point3f GetShootPoistion(float angle,Point3f center,float r);
    void ChassisToPtz(RM_BuffData & BestArmor);
    Angle_t ComputeBuffShootTime(float tx, float ty, float distance,struct CarData CarDatas);
    double BuffKf(double del_angle,KF_two& Filter,double del_time);//根据二维点做预测
    Point2f getPredictPoint(Point2f circle_center_point,Point2f target_point,double predictangel);
    void PinHole_solver(RM_BuffData& buffdata)
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
