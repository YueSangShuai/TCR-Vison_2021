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
    double calStartTimeInPeriod(float angle_diff_of_interval, double time_interval)
    {
        double amplitude = 0.785;
        double angular_frequency = 1.884;
        double initial_phase = 0;
        double const_number = 1.305;
        double temp_formulation_1 = angular_frequency * (angle_diff_of_interval - const_number * time_interval) / (2 * amplitude);
        double temp_formulation_2 = sin(angular_frequency * time_interval / 2);
        double time_begin = (asin(temp_formulation_1 / temp_formulation_2) - initial_phase) / angular_frequency - time_interval / 2;
        return time_begin;
    }
    double calPredictAngleByPeriod(double time_begin, double time_interval)
    {
        double angular_frequency = 1.884;
        double amplitude = 0.785;
        double initial_phase = 0;
        double const_number = 1.305;
        float angular_frequency_half = angular_frequency / 2;
        //积分计算角度（弧度）
        double rotate_angle = amplitude / angular_frequency_half * sin( angular_frequency_half * (2 * time_begin + time_interval) + initial_phase )
                              * sin( angular_frequency_half * time_interval ) + const_number * time_interval;
        return rotate_angle;
    }
public:
    RM_BuffData* BuffModeSwitch(Mat Src,int color);
};

#endif // FINDBUFF_H
