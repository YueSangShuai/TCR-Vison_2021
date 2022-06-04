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
    ChassisToPtz_x = 0;
    ChassisToPtz_y = 0;
    ChassisToPtz_z = 0;

    ChassisToPtz_Pitch = 0*PI/180;
    ChassisToPtz_Yaw = 0*PI/180;
    ChassisToPtz_Roll = 0*PI/180;

    BuffWidth = 23;
    BuffHeight = 12.7;
}

/**
 * @brief BuffAngleSolver::GetBuffrAngle    角度解算得到空间坐标
 * @param BestArmor
 */
void BuffAngleSolver::GetBuffrAngle( RM_BuffData &BestArmor){
    std::vector<cv::Point2f>point2D;
    std::vector<cv::Point3f>point3D;

    GetPoint2D(BestArmor,point2D);                                                                                                      //矩形转换为2d坐标
    GetPoint3D(BestArmor,point3D);                                                                                                     //矩形转换为3d坐标
    CountAngleXY(point2D,point3D,BestArmor);                                                                                         //解决pnp问题

    ChassisToPtz(BestArmor);                                                                                                                 //底盘转云台
}

/**
 * @brief GetBuffForceResult            //得到大符预测点
 * @param BestArmor
 */
void BuffAngleSolver::GetBuffShootAngle(RM_BuffData* BestArmor,BuffStatus BuffShootStatus,CarData carDatas){
    float AngleSpeed=BestArmor[3].del_angle;
    float del_time=BestArmor[3].del_time;

//    GetBuffrAngle(BestArmor[3]);

//    BestArmor[3].point[0]=Point2f ((BestArmor[3].predict.x-BestArmor[3].width/2),(BestArmor[3].predict.y+BestArmor[3].width/2));
//    BestArmor[3].point[3]=Point2f ((BestArmor[3].predict.x-BestArmor[3].width/2),(BestArmor[3].predict.y-BestArmor[3].width/2));
//    BestArmor[3].point[1]=Point2f ((BestArmor[3].predict.x+BestArmor[3].width/2),(BestArmor[3].predict.y+BestArmor[3].width/2));
//    BestArmor[3].point[2]=Point2f ((BestArmor[3].predict.x+BestArmor[3].width/2),(BestArmor[3].predict.y-BestArmor[3].width/2));

    //GetBuffrAngle(BestArmor[3]);
    PinHole_solver(BestArmor[3]);
}

/**
 * @brief BuffAngleSolver::GetShootPoistion
 * @param angle     当前位置相对圆弧度角
 * @param center
 * @param r
 * @return
 * @remark 传入角度必须为弧度
 */
Point3f BuffAngleSolver::GetShootPoistion(float angle, Point3f center, float r){
    return Point3f(center.x + r*sin(angle),center.y + r*cos(angle),center.z);
}

double BuffAngleSolver::BuffKf(double del_angle,KF_two& Filter,double del_time) {
    if(!Filter.is_set_x){
        //第一次设置滤波
        Eigen::VectorXd x(2,1);
        x<<del_angle,0;
        Filter.set_x(x);
    }else{
        //连续滤波
//        ofstream erro_file;
//        erro_file.open("../erro.txt",ios::out | ios::app);
        Eigen::VectorXd x(2,1);
        x<<del_angle,del_angle/del_time;
        Eigen::MatrixXd F(2,2);
        F<<     1, del_time,
                0, 1;

        Filter.Prediction(F);
        Filter.update(x,F);
    }
    return Filter.x_(0);
}
/**
 * @brief BuffAngleSolver::ShootAdjust              依据计算所得法向量得旋转角,进行坐标旋转得到正对坐标
 * @param x
 * @param y
 * @param z
 * @param pitch
 * @param yaw
 * @return
 */
float BuffAngleSolver::ShootAdjust(double x, double y, double z, float pitch, float yaw){
    //绕pitch轴旋转，即为绕x轴旋转
    Eigen::MatrixXd r_Pitch(3,3);
    r_Pitch<<cos(pitch) , 0 , -sin(pitch),
                        0 , 1 , 0,
                        sin(pitch) , 0 , cos(pitch);
    //绕yaw轴旋转，即为绕y轴旋转
    Eigen::MatrixXd r_Yaw(3,3);
    r_Yaw<<cos(yaw) , sin(yaw) , 0,
                     -sin(yaw) , cos(yaw) , 0 ,
                     0 , 0 , 1;

    Eigen::VectorXd original(3,1);          //按z，x，y传入，即变化对应到左手坐标系
    original<<z,x,y;
    original = r_Pitch*original;
    original = r_Yaw*original;
    x = original(1);
    y = original(2);
    z = original(0);
}

/**
 * @brief BuffAngleSolver::GetPoint2D       存入2d点
 * @param BestArmor
 * @param point2D
 */
void BuffAngleSolver::GetPoint2D( RM_BuffData & BestArmor,std::vector<cv::Point2f>&point2D){

    cv::Point2f lu,ld,ru,rd,ce,cice;        //right_down right_up left_up left_down

    lu = BestArmor.point[0];
    ld = BestArmor.point[3];
    ru = BestArmor.point[1];
    rd = BestArmor.point[2];
//     cout<<"lu:"<<lu<<endl;
    point2D.clear();///先清空再存入
    point2D.push_back(lu);
    point2D.push_back(ru);
    point2D.push_back(rd);
    point2D.push_back(ld);
}

//矩形转换为3d坐标                                                                                                                                                                                             3
void BuffAngleSolver::GetPoint3D( RM_BuffData & BestArmor,std::vector<cv::Point3f>&point3D)
{
    float fHalfX=0;
    float fHalfY=0;

    fHalfX=BuffWidth/2.0;
    fHalfY=BuffHeight/2.0;

    point3D.push_back(cv::Point3f(-fHalfX,-fHalfY,0.0));
    point3D.push_back(cv::Point3f(fHalfX,-fHalfY,0.0));
    point3D.push_back(cv::Point3f(fHalfX,fHalfY,0.0));
    point3D.push_back(cv::Point3f(-fHalfX,fHalfY,0.0));

}

//pnp转换
void BuffAngleSolver::CountAngleXY(const std::vector<cv::Point2f>&point2D,const std::vector<cv::Point3f>&point3D, RM_BuffData & BestArmor){
    cv::Mat rvecs;//=cv::Mat::zeros(3,1,CV_64FC1);
    cv::Mat tvecs;//=cv::Mat::zeros(3,1,CV_64FC1);

//     cv::solvePnP(point3D,point2D,caremaMatrix,distCoeffs,rvecs,tvecs,false,SOLVEPNP_UPNP);
    solvePnP(point3D,point2D,caremaMatrix,distCoeffs,rvecs,tvecs);
    double tx = tvecs.ptr<double>(0)[0];
    double ty = -tvecs.ptr<double>(0)[1];
    double tz = tvecs.ptr<double>(0)[2];

    BestArmor.tx = tx;
    BestArmor.ty = ty;
    BestArmor.tz = tz;


    BestArmor.pitch = atan2( BestArmor.ty, BestArmor.tz)*180/PI;
    BestArmor.yaw = atan2( BestArmor.tx, BestArmor.tz)*180/PI;
}

/**
 * @brief BuffAngleSolver::ChassisToPtz             摄像头旋转至云台
 * @param BestArmor
 */
void BuffAngleSolver::ChassisToPtz(RM_BuffData &BestArmor){
    //绕roll轴旋转，即为绕z轴旋转
    Eigen::MatrixXd r_Roll(3,3);
    r_Roll<<1 , 0 , 0,
                    0 , cos(ChassisToPtz_Roll) , sin(ChassisToPtz_Roll),
                    0 , -sin(ChassisToPtz_Roll) , cos(ChassisToPtz_Roll);
    //绕pitch轴旋转，即为绕x轴旋转
    Eigen::MatrixXd r_Pitch(3,3);
    r_Pitch<<cos(ChassisToPtz_Pitch) , 0 , -sin(ChassisToPtz_Pitch),
                        0 , 1 , 0,
                        sin(ChassisToPtz_Pitch) , 0 , cos(ChassisToPtz_Pitch);
    //绕yaw轴旋转，即为绕y轴旋转
    Eigen::MatrixXd r_Yaw(3,3);
    r_Yaw<<cos(ChassisToPtz_Yaw) , sin(ChassisToPtz_Yaw) , 0,
                     -sin(ChassisToPtz_Yaw) , cos(ChassisToPtz_Yaw) , 0 ,
                     0 , 0 , 1;

    Eigen::VectorXd original(3,1);          //按z，x，y传入，即变化对应到左手坐标系
    original<<BestArmor.tz,BestArmor.tx,BestArmor.ty;

    //平移变换
    Eigen::VectorXd translation(3,1);
//    cout<<"未偏移前:x:"<<change[1]<<"   y:"<<change[2]<<"   z:"<<change[0]<<endl;
    translation<<ChassisToPtz_z,ChassisToPtz_x,ChassisToPtz_y;
    original = original + translation;

    Eigen::VectorXd change(3,1);
    //旋转变换
    change =  r_Roll * original;
    change = r_Pitch*change;
    change = r_Yaw*change;

    BestArmor.tx = change(1);
    BestArmor.ty = change(2);
    BestArmor.tz = change(0);
}

Angle_t BuffAngleSolver::ComputeBuffShootTime(float tx, float ty, float distance,struct CarData CarDatas){

    //单位转换
    tx /= 100.0;
    ty /= 100.0;
    float tz = distance/100.0;
//    cout<<"缓冲计算tx:"<<tx<<"  ty:"<<ty<<" tz:"<<distance<<endl;
    float speed = CarDatas.ShootSpeed;
//    speed=20;
    double a = -0.5*G*(pow(tz,2)+pow(tx,2));
    double b = sqrt((pow(tz,2)+pow(tx,2)))*pow(speed,2);
    double c = -0.5*G*(pow(tz,2)+pow(tx,2)) - pow(speed,2)*ty;
    //判别式
    double Discriminant = pow(a,2)+pow(b,2)-4*a*c;
//    cout<<"判别式:"<<Discriminant<<"   a:"<<a<<"   b:"<<b<<"   c:"<<c<<endl;
    Angle_t ShootBuff = {0,0,0,0};
    if(Discriminant<0)return ShootBuff;
    double angle_tan_1 = atan((-b + sqrt(Discriminant))/(2*a))*180/PI;
    double angle_tan_2 = atan((-b - sqrt(Discriminant))/(2*a))*180/PI;
//    double angle_tan = ty/b;
//    double real_angle = fabs(angle_tan - angle_tan_1)<fabs(angle_tan - angle_tan_2)?angle_tan_1:angle_tan_2;
    //角度取舍,并转换为相对角度
    if(fabs(angle_tan_1)<=fabs(angle_tan_2)&&fabs(angle_tan_1)<45){
        ShootBuff.pitch = angle_tan_1 - CarDatas.pitch;
    }else if(fabs(angle_tan_2)<45){
        ShootBuff.pitch = angle_tan_2 - CarDatas.pitch;
    }else{      //都不符合要求
        cout<<"计算解不符合实际"<<endl;
        return ShootBuff;
    }

//    real_angle = atan(real_angle)*180/PI;
//    ShootBuff.pitch = real_angle;
    ShootBuff.yaw = atan2(tx,tz)*180/CV_PI;
    cout<<"缓冲计算tx:"<<tx<<"  ty:"<<ty<<" tz:"<<tz<<"yaw"<<ShootBuff.yaw<<endl;
    //处理当前视角为后方时
//    if(distance<0){
//        if(ShootBuff.pitch>=0){
//            ShootBuff.pitch += -180;
//        }else{
//            ShootBuff.pitch += 180;
//        }
//        if(ShootBuff.yaw>=0){
//            ShootBuff.yaw += -180;
//        }else{
//            ShootBuff.yaw += 180;
//        }
//    }
//    ShootBuff.t = b/(speed*cos(ShootBuff.pitch*PI/180))*1000;
    ShootBuff.t = tz/(speed*cos(ShootBuff.pitch*PI/180))*1000;
    return ShootBuff;
}
Point2f BuffAngleSolver::getPredictPoint(Point2f circle_center_point,Point2f target_point,double predictangel) {
    int x1, x2, y1, y2;
    x1 = circle_center_point.x * 100;
    x2 = target_point.x * 100;
    y1 = circle_center_point.y * 100;
    y2 = target_point.y * 100;
    Point2f predict_point;
    predict_point.x = static_cast<int>(
            (x1 + (x2 - x1) * cos(-predictangel * 180 / PI) - (y1 - y2) * sin(-predictangel * 180 / PI)) / 100);
    predict_point.y = static_cast<int>(
            (y1 - (x2 - x1) * sin(-predictangel * 180 / PI) - (y1 - y2) * cos(-predictangel * 180 / PI)) / 100);
    return predict_point;
}

