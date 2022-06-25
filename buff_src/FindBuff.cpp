#include"../buff_include/FindBuff.h"
#include "../DrawCurve/DraCurve.h"
#include "../header/General.h"
#include <fstream>   // 对文件输入输出
#include <iostream>  //对屏幕上输入输出
#include <iostream>
#include <string.h>
#include <stdlib.h>
/********************大符调试********************************/
#define PI 3.14159
#define BUFF_W_RATIO_H 1.9              //最终大符内轮廓长宽比
#define BUFF_AREA_RATIO 500.0       //最终大符内轮廓面积与图片面积像素比
#define BUFFER_BUFF_BOX 4             //大符内轮廓存储缓冲数量
#define BUFF_CIRCLE_BOX    3             //圆形计算所需个数,应比总数量少1,最后一位为当前识别目标
#define BUFF_MIN_DISTANCE 0             //两次记录最短间距
#define G 9.80665
DrawCurve draw;
int rotationbool=0;
int rotationNum=0;
/*************************************************************/
RM_BuffData BuffBox[BUFFER_BUFF_BOX];       //存储最近几帧的大符信息
int BuffNum = 0;

double blueDecay=0.3;
uint8_t dilateKernelSize=3;
uint8_t binaryThreshold=90;
int image_count=0;
double count_del_angle=0;
double count_del_time=0;

double now_angle=0;
double last_angle=0;
double now_speed=0;
double last_speed=0;

KF_two anglefilter;
bool isfind=false;
/**
 *
 *
 * @brief FindBuff::BuffModeSwitch
 * @param Src
 * @return 返回大符识别矩形
 * @remark 大符识别接口,传入当前帧图像,进行图像处理和寻找目标,返回最终待击打矩形,搭配相应滤光片使用使图像更稳定
 */
RM_BuffData* FindBuff::BuffModeSwitch(Mat Src,int color){
    Mat dst;
    RM_BuffData Buff;
    PreDelBuff(Src,dst,color);
    vector<RotatedRect> BuffClump = FindBestBuff(Src,dst);
    if(BuffClump.size()<=0){
        cout<<"当前大符未识别到目标";
        this->circle_center=Point2f (0,0);
        return (RM_BuffData*)-1;
    }

    RotatedRect BuffObject  = GetShootBuff(BuffClump,Src);
    Buff.point[0] = Point2f(BuffObject.center.x - BuffObject.size.width/2,BuffObject.center.y - BuffObject.size.height/2);
    Buff.point[1] = Point2f(BuffObject.center.x + BuffObject.size.width/2,BuffObject.center.y - BuffObject.size.height/2);
    Buff.point[2] = Point2f(BuffObject.center.x + BuffObject.size.width/2,BuffObject.center.y + BuffObject.size.height/2);

    Buff.point[3] = Point2f(BuffObject.center.x - BuffObject.size.width/2,BuffObject.center.y + BuffObject.size.height/2);
    Buff.circle_center=this->circle_center;
    Buff.box.center=BuffObject.center;
    Buff.normalizedCenter=Point2f((Buff.box.center.x-Buff.circle_center.x),(Buff.box.center.y-Buff.circle_center.y));
    Buff.armoranle= myArctan(Buff.normalizedCenter);
    Buff.box = BuffObject;
    Buff.image_count=image_count;
    Buff.timestamp=(double)cvGetTickCount()/(cvGetTickFrequency()*1000000);

    image_count++;
    if(image_count==maxImage)image_count=0;
    float del_angle= GetAngle(Buff.circle_center,Buff.box.center,BuffBox[3].box.center)*(PI/180);
    float  del_time=Buff.timestamp-BuffBox[3].timestamp;

    last_speed=now_speed;
    last_angle=now_angle;
    now_angle=del_angle;



    double erro=now_speed-last_speed;
    double predict_angle=del_angle;
    KF_angle(predict_angle,anglefilter);
    predict_angle=anglefilter.x_(0);

    if(isnan(predict_angle)){
        anglefilter.is_set_x=false;
    }

    if(erro>0){
        predict_angle*=1.2;
    }else{
        predict_angle*=0.9;
    }
    Buff.predict= getPredict(Buff.circle_center,Buff.box.center,predict_angle);
    if(isfind==false) Buff.predict= Buff.box.center;
    isfind=false;
    //draw.InsertData(del_angle,predict_angle,"value","predict","boxing");

    if(del_angle<0.001){
        Buff.predict= getPredict(Buff.circle_center,Buff.box.center,0);
    }
    if(del_angle>.05){
        Buff.predict= getPredict(Buff.circle_center,Buff.box.center,0);
    }

    if(Buff.image_count<maxImage-1){
        count_del_angle+=del_angle;
        count_del_time+=del_time;
    }else if(Buff.image_count==maxImage-1){
        Buff.del_angle=count_del_angle;
        Buff.del_time=count_del_time;
        count_del_angle=0;
        count_del_time=0;
    }
    //存入数组,进入分析
    if(BuffNum == 0){
        BuffNum++;
        //初始化数组内容
        for(int i = 0;i<BUFFER_BUFF_BOX;i++){
            BuffBox[i] = Buff;
        }
    }
    else{
        //已初始化,进入连续计算
        int index = -1;
        float max_distance = 0;
        //寻找最远的序号
        for(int i = 0;i<3;i++){
            float distance = GetDistance(BuffObject.center,BuffBox[i].box.center);
            if(distance>BUFF_MIN_DISTANCE&&distance>max_distance){
                index = i;
                max_distance = distance;
            }
        }
        if(index!=-1&&GetDistance(BuffObject.center,BuffBox[(index+1)%BUFF_CIRCLE_BOX].box.center)>BUFF_MIN_DISTANCE
           &&GetDistance(BuffObject.center,BuffBox[(index+2)%BUFF_CIRCLE_BOX].box.center)>BUFF_MIN_DISTANCE){
            //符合条件存入数组
            BuffBox[index] = Buff;
            BuffNum++;                  //指向下一个位置
        }

        BuffBox[BUFFER_BUFF_BOX-1] = Buff;
    }

    for(int t = 0;t<3;t++){
        for(int i = 0;i<4;i++){
            line(Src,BuffBox[t].point[(i+1)%4],BuffBox[t].point[i%4],Scalar(255,0,0),1,4);
        }
    }
    if(BuffNum<3)
        return  (RM_BuffData*)-1;
    return BuffBox;
}

/**
 * @brief FindBuff::PreDelBuff
 * @param Src
 * @param dst
 * @return
 */
void FindBuff::PreDelBuff(Mat Src, Mat &dst,int color){
    GaussianBlur(Src, Src, Size(3, 3), 0);
    Mat channels[3],mid,bin;
    split(Src,channels);
    //衰减蓝色通道
    for(int i=0;i<Src.cols*Src.rows;i++)
    {
        channels[0].data[i]*=(1-blueDecay);
    }
    //红通道-蓝通道
    subtract(channels[2],channels[0],mid);
    //imshow("mid",mid);
    threshold(mid,bin,binaryThreshold,255,THRESH_BINARY);
    Mat element = getStructuringElement(MORPH_ELLIPSE,Point(dilateKernelSize,dilateKernelSize));
    dilate(bin,mid,element);
    Mat kernel = getStructuringElement(MORPH_ELLIPSE,Point(4,4));
    morphologyEx(mid,bin,MORPH_CLOSE,kernel);
    dst=bin;
#ifdef IMSHOW_IMAGE
    imshow("dst",bin);
#endif
}

/**
 * @brief FindBuff::FindBestBuff
 * @param Src
 * @param color
 * @return
 */
vector<RotatedRect> FindBuff::FindBestBuff(Mat Src,Mat & dst) {
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<RotatedRect> box_buffs;
    //寻找全部轮廓,用于计算内轮廓数量
    findContours(dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    if (contours.size() > 2) {
        vector<Point> possibleCenter;
        for (uint i = 0; i < contours.size(); i++) {
            int sub = hierarchy[i][2];
            if (sub != -1)//有子轮廓
            {
                if(hierarchy[sub][0]==-1)//没有兄弟轮廓
                {
                    auto area = contourArea(contours[sub]);//轮廓面积
                    auto rect = minAreaRect(contours[sub]);//轮廓外接矩形
                    auto mmp = rect.size;
                    float aspectRatio = mmp.height / mmp.width;
                    float areaRatio = area / (rect.size.width * rect.size.height);//面积比，用来衡量轮廓与矩形的相似度
                    if (aspectRatio > 1)
                        aspectRatio = 1 / aspectRatio;
                    //qDebug()<<"面积:"<<area<<",长宽比:"<<aspectRatio<<",面积比:"<<areaRatio<<endl;
                    //TODO:确定实际装甲板面积、长宽比、面积占比

                    if (area > 100&& aspectRatio < 0.8 && areaRatio > 0.5) {
                        ellipse(Src, rect, Scalar(0,255,0), 5, CV_AA);
                        box_buffs.push_back(rect);
                    }
                }
            }
                else{
                Point2f center;
                float radius;
                minEnclosingCircle(contours[i], center, radius);
                //circle(Src,center,CV_AA,Scalar(255,0,0),8);
                //半径需要具体调试
                if (radius < 10 && radius > 3) {
                    possibleCenter.push_back(center);
                    //circle(Src,center,CV_AA,Scalar(255,0,0),8);
                }
            }
        }
        for(int i=0;i<box_buffs.size();i++){
            float x2,x3,x4,y2,y3,y4;
            //获取目标装甲板的四个角点
            Point2f vectors[4];
            box_buffs[i].points(vectors);
            float dd1= getPointsDistance(vectors[0],vectors[1]);
            float dd2= getPointsDistance(vectors[1],vectors[2]);
            if(dd1<dd2)
            {
                x2=box_buffs[i].center.x-vectors[0].x;
                y2=box_buffs[i].center.y-vectors[0].y;
                x3=box_buffs[i].center.x-vectors[1].x;
                y3=box_buffs[i].center.y-vectors[1].y;
                dd2=dd1;
            }
            else
            {
                x2=box_buffs[i].center.x-vectors[1].x;
                y2=box_buffs[i].center.y-vectors[1].y;
                x3=box_buffs[i].center.x-vectors[2].x;
                y3=box_buffs[i].center.y-vectors[2].y;
                dd1=dd2;
            }
            x4=x2-x3;y4=y2-y3;
            //遍历所有可能的中心R点
            for (Point2f p:possibleCenter)
            {
                //计算待选中心和装甲板中心的角度
                float x1=box_buffs[i].center.x-p.x;
                float y1=box_buffs[i].center.y-p.y;
                float angle=0;
                float d1 = getPointsDistance(p,box_buffs[i].center);
                //计算与目标装甲板中心的夹角
                angle=acos((x4*x1+y1*y4)/dd1/d1)*57.3;

                int minRadius=50;
                int maxRadius=88;
                circle(Src,box_buffs[i].center,minRadius,Scalar(255,0,0),2);
                circle(Src,box_buffs[i].center,maxRadius,Scalar(255,0,0),2);
                //根据半径范围和与短边（锤子柄）的角度筛选出中心R
                if(d1>minRadius && d1 < maxRadius && (angle<10||angle>170))
                {
                    Mat debug=src.clone();
                    circle_center=p;
                    isfind=true;
                    circle(Src,p,CV_AA,Scalar(255,0,0),2);

                    break;
                }
            }
        }
    }
#ifdef IMSHOW_IMAGE

    imshow("绘制ing", Src);
#endif
    return box_buffs;

}
/**
 * @brief FindBuff::GetShootBuff
 * @param box_buffs
 * @param Src
 * @return
 */
RotatedRect FindBuff::GetShootBuff(vector<RotatedRect> box_buffs,Mat Src){
    if(box_buffs.size() == 1)
        return box_buffs[0];
    //为最终得到的大符内轮廓打分
    int *grade = (int *)malloc(box_buffs.size()*sizeof(int));
    memset(grade, 0, box_buffs.size()*sizeof(int));
    for(int i = 0;i<box_buffs.size();i++){
        *(grade+i) = 100*(1 - fabs(box_buffs[i].size.height/box_buffs[i].size.width - BUFF_W_RATIO_H)/BUFF_W_RATIO_H);
        //cout<<fabs((Src.cols*Src.rows)/box_buffs[i].size.area())<<endl;
        *(grade+i) += 100*(1 - fabs((Src.cols*Src.rows)/box_buffs[i].size.area() - BUFF_AREA_RATIO)/BUFF_AREA_RATIO);
    }
    int max_grade = *grade;
    int max_xuhao = 0;
    for(int t = 1;t<box_buffs.size();t++){
        if(*(grade+t)>max_grade){
            max_grade = *(grade+t);
            max_xuhao = t;
        }
    }
    free(grade);
    return box_buffs[max_xuhao];
}
/**
 * @brief FindBuff::getCenterAngle
 * @param circle_center
 * @param center
 * @return
 */
double FindBuff::getCenterAngle(Point2f circle_center, Point2f center) {
        double angle_recent;
        double angle_recent_2PI;
        angle_recent = atan2((center.y - circle_center.y), (center.x - circle_center.x));
        // 以x为原点，向右，向上为正轴，逆时针求解angle_recent_2PI
        if(angle_recent > 0 && angle_recent < PI)
            angle_recent_2PI = 2 * PI - angle_recent;
        else
            angle_recent_2PI = -angle_recent;
return angle_recent_2PI;
}
Point2f FindBuff::getPredict (Point2f circle_center_point,Point2f target_point,double predictangel) {
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
void FindBuff::KF_angle(double angle,KF_two& Filter) {
    if(!Filter.is_set_x){
        //第一次设置滤波
        Eigen::VectorXd x(1,1);
        x<<angle;
        Filter.set_x(x);
    }else{
        Eigen::VectorXd x(1,1);
        x<<angle;
        Eigen::MatrixXd F(1,1);
        F<<   1;
        Filter.Prediction(F);
        Filter.update(x,F);
    }
}

