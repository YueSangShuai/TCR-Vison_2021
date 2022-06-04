#ifndef FINDBUFF_H
#define FINDBUFF_H
#include"../header/General.h"
class FindBuff{
private:
    void PreDelBuff(Mat Src, Mat &dst,int color);
    vector<RotatedRect> FindBestBuff(Mat Src,Mat & dst);
    RotatedRect GetShootBuff(vector<RotatedRect> box_buffs,Mat Src);
    double getCenterAngle(Point2f circle_center,Point2f center);
    Point2f getPredict(Point2f circle_center_point,Point2f target_point,double predictangel);
    Point2f circle_center;
    bool is_rotation=true;
    double upbuchagn(double x){
        double a0 =      0.4932 ;
        double a1 =     -0.3288 ;
        double b1 =     0.03847 ;
        double w =       5.079;
        double temp= a0 + a1*cos(x*w) + b1*sin(x*w);
        return temp;
    }
    double downbuchagn(double x){
        double a0 =      0.7271  ;
        double a1 =     0.07821 ;
        double b1 =      0.2114  ;
        double a2 =    -0.08488  ;
        double b2 =     0.01165 ;
        double w =        6.78 ;
        double temp= a0 + a1*cos(x*w) + b1*sin(x*w) +
                     a2*cos(2*x*w) + b2*sin(2*x*w);
        return  temp;
    }
public:
    RM_BuffData* BuffModeSwitch(Mat Src,int color);
};

#endif // FINDBUFF_H
