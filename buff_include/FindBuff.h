#ifndef FINDBUFF_H
#define FINDBUFF_H
#include"../header/General.h"
class FindBuff{
private:
    void PreDelBuff(Mat Src, Mat &dst,int color);
    vector<RotatedRect> FindBestBuff(Mat Src,Mat & dst);
    RotatedRect GetShootBuff(vector<RotatedRect> box_buffs,Mat Src);
    double getCenterAngle(Point2f circle_center,Point2f center);
    Point2f circle_center;

public:
    RM_BuffData* BuffModeSwitch(Mat Src,int color);
    Point2f getCircleCenter(){
        return circle_center;
    }
};

#endif // FINDBUFF_H
