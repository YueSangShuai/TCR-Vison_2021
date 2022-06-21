#ifndef FINDBUFF_H
#define FINDBUFF_H
#include"../header/General.h"
#include "../header/Filter.h"
class FindBuff{
private:
    void PreDelBuff(Mat Src, Mat &dst,int color);
    vector<RotatedRect> FindBestBuff(Mat Src,Mat & dst);
    RotatedRect GetShootBuff(vector<RotatedRect> box_buffs,Mat Src);
    double getCenterAngle(Point2f circle_center,Point2f center);
    Point2f getPredict(Point2f circle_center_point,Point2f target_point,double predictangel);
    Point2f circle_center;
    void KF_angle(double angle,KF_two& Filter);
public:
    RM_BuffData* BuffModeSwitch(Mat Src,int color);
};

#endif // FINDBUFF_H
