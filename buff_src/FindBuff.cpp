#include"../buff_include/FindBuff.h"
#include "../DrawCurve/DraCurve.h"
#include "../header/General.h"
#include <fstream>   // 对文件输入输出
#include <iostream>  //对屏幕上输入输出

/********************大符调试********************************/
#define PI 3.14159
#define BUFF_W_RATIO_H 1.9              //最终大符内轮廓长宽比
#define BUFF_AREA_RATIO 500.0       //最终大符内轮廓面积与图片面积像素比
#define BUFFER_BUFF_BOX 4             //大符内轮廓存储缓冲数量
#define BUFF_CIRCLE_BOX    3             //圆形计算所需个数,应比总数量少1,最后一位为当前识别目标
#define BUFF_MIN_DISTANCE 0             //两次记录最短间距

#define G 9.80665
DrawCurve draw;
/*************************************************************/
RM_BuffData BuffBox[BUFFER_BUFF_BOX];       //存储最近几帧的大符信息
int BuffNum = 0;
double blueDecay=0.25;
uint8_t blue_dilateKernelSize=5;
uint8_t red_dilateKernelSize=5;
uint8_t binaryThreshold=100;
uint8_t rRadius=10;
int image_count=0;
double count_del_time=0;
double count_del_angle=0;
/**
 * @brief FindBuff::BuffModeSwitch
 * @param Src
 * @return 返回大符识别矩形
 * @remark 大符识别接口,传入当前帧图像,进行图像处理和寻找目标,返回最终待击打矩形,搭配相应滤光片使用使图像更稳定
 */
RM_BuffData* FindBuff::BuffModeSwitch(Mat Src,int color){
    Mat dst;
    RM_BuffData Buff;
    PreDelBuff(Src,dst,color);
    //imshow("Src",Src);
//    Mat temp;
//    temp= rgbtohsv(Src,_color);
//    vector<RotatedRect> BuffClump = FindBestBuff(Src,temp);
    vector<RotatedRect> BuffClump = FindBestBuff(Src,dst);
//    if(BuffClump.size()!=0){
//        Point2f center=BuffClump[0].center;
//        cout<<"center:"<<center;
//        circle(Src,center,2,Scalar(0,255,0));
//    }


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
    Buff.timestamp=(float)cvGetTickCount();

    image_count++;
    if(image_count==30)image_count=0;
//    double rotation= BuffBox[3].armoranle-BuffBox[2].armoranle;
////    if(rotation<0){
////        this->is_rotation=true;
////        cout<<"顺时针"<<endl;
////    }else{
////        this->is_rotation=false;
////        cout<<"逆时针"<<endl;
////    }
//    cout<<"rotation"<<rotation<<endl;
    float del_angle= GetAngle(Buff.circle_center,BuffBox[3].box.center,BuffBox[2].box.center)/(180/PI);
    float del_time=(Buff.timestamp-BuffBox[3].timestamp)/(cvGetTickFrequency()*1000000);

    ofstream angle_file;
    ofstream kf_file;
    angle_file.open("/home/rmtcr/RM/angle.txt", ios::out | ios::app);
    kf_file.open("/home/rmtcr/RM/kf.txt", ios::out | ios::app);
//    out_txt_file << del_angle<<endl;
    if(Buff.image_count<29){
        count_del_angle+=del_angle;
        count_del_time+=del_time;
    }else if(Buff.image_count==29){
        Buff.del_time=count_del_time;
        Buff.del_angle=count_del_angle;
        count_del_angle=0;
        count_del_time=0;
    }

    //draw.InsertData(BuffBox[3].armoranle-BuffBox[2].armoranle);
//    if(del_angle>0.5){
//        cout<<"del_angle"<<del_angle;
//        double anglespeed=del_angle/(BuffBox[3].timestamp-BuffBox[2].timestamp);
//        cout<<"angle_speed="<<anglespeed<<endl;
//        draw.InsertData(anglespeed);
//    }
    //存入数组,进入分析
    if(BuffNum == 0){
        BuffNum++;
        //初始化数组内容
        for(int i = 0;i<BUFFER_BUFF_BOX;i++){
            BuffBox[i] = Buff;
        }
    }else{
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
    //circle(Src,circle_center,CV_AA,Scalar(255,0,0),5);
    //cout<<"del_time"<<(BuffBox[3].timestamp-BuffBox[2].timestamp)/(cvGetTickFrequency()*1000)<<endl;
    //draw.InsertData(del_time);
    return BuffBox;
}

/**
 * @brief FindBuff::PreDelBuff
 * @param Src
 * @param dst
 * @return
 */
void FindBuff::PreDelBuff(Mat Src, Mat &dst,int color){
double t = (double)cvGetTickCount();            //计时
//    cvtColor(Src,dst,CV_RGB2GRAY);
//    threshold(dst,dst,40,255,CV_THRESH_BINARY);
//    cv::Mat gray_element=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(7,7));
//    dilate(dst,dst,gray_element);
//    erode(dst,dst,gray_element);
////    cv::Mat gray_element2=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
////    dilate(dst,dst,gray_element);
//    imshow("灰度二值化",dst);
////    vector<Mat> spli;
//    Mat hsv;
//    Mat mask;
//    cvtColor(Src,hsv,CV_RGB2HSV);
////    split(hsv,spli);
//    inRange(hsv, Scalar(0, 10, 46), Scalar(180, 60, 255), mask);
//    dilate(mask,mask,gray_element);
////    imshow("mask",mask);
//    dst = dst - mask;
//    dilate(dst,dst,gray_element);
//    Mat element=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
//    erode(dst,dst,element);

if(color==2)
{
Mat channels[3],mid,bin;
split(Src,channels);
//衰减蓝色通道
for(int i=0;i<Src.cols*Src.rows;i++)
{
channels[2].data[i]*=(1-blueDecay);
}
//红通道-蓝通道
subtract(channels[2],channels[0],mid);
threshold(mid,bin,binaryThreshold,255,THRESH_BINARY);
Mat element = getStructuringElement(MORPH_ELLIPSE,Point(red_dilateKernelSize,red_dilateKernelSize));
dilate(bin,mid,element);
Mat kernel = getStructuringElement(MORPH_ELLIPSE,Point(7,7));
morphologyEx(mid,bin,MORPH_CLOSE,kernel);
dst=bin;
imshow("Src",dst);
t=(double)cvGetTickCount()-t;
t = t/(cvGetTickFrequency()*1000);                                //t2为一帧的运行时间,也是单位时间
printf("used time is %gms\n",t);
//    imshow("分割",dst);
}
else if(color==1){
Mat channels[3],mid,bin;
split(Src,channels);
//衰减蓝色通道
for(int i=0;i<Src.cols*Src.rows;i++)
{
channels[0].data[i]*=(1-blueDecay);
}
//蓝通道-红通道
subtract(channels[0],channels[2],mid);
threshold(mid,bin,binaryThreshold,255,THRESH_BINARY);
Mat element = getStructuringElement(MORPH_ELLIPSE,Point(blue_dilateKernelSize,blue_dilateKernelSize));
dilate(bin,mid,element);
Mat kernel = getStructuringElement(MORPH_ELLIPSE,Point(7,7));
morphologyEx(mid,bin,MORPH_CLOSE,kernel);
dst=bin;
imshow("Src",dst);
t=(double)cvGetTickCount()-t;
t = t/(cvGetTickFrequency()*1000);                                //t2为一帧的运行时间,也是单位时间
printf("used time is %gms\n",t);
//    imshow("分割",dst);
}


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
    bool success = false;                               //记录是否成功找到符合要求扇叶

//    for (size_t i = 0; i < hierarchy.size(); i++) {
//        if (hierarchy[i][2] == -1)continue;
//        if (contours[i].size() < 6)continue;
//        RotatedRect box = fitEllipse(contours[i]);
//
//        float shanye_bili = box.size.height / box.size.width;
//        float shanye_area = box.size.height * box.size.height;
//
////        if (shanye_bili > 2.9 || (shanye_bili < 1.7 && shanye_bili > 1.5) || shanye_bili < 1.1 ||
////            box.size.area() < 500)
////            continue;
//        ellipse(Src, box, Scalar(255, 0, 255), 5, CV_AA);
//        int *nei_lunkuo = (int *) malloc(contours.size() * sizeof(int));
//        memset(nei_lunkuo, 0, contours.size() * sizeof(int));
//        //判断第一个内轮廓是否符合标准
//        if (contours[hierarchy[i][2]].size() >= 6) {
//            RotatedRect first_box = fitEllipse(contours[hierarchy[i][2]]);
//
////            cout<<"首个扇叶bili:"<<(float)first_box.size.height/(float)first_box.size.width<<endl;
////            cout<<"首个面积比1:"<<box.size.area()/first_box.size.area()<<endl;
////            cout<<"首个轮廓面积："<<(Src.cols*Src.rows)/box.size.area()<<endl;
//            if (box.size.area() / first_box.size.area() < 10)
//                *(nei_lunkuo + hierarchy[i][2]) = 1;
//        }
//        int j = hierarchy[i][2];
//        while (hierarchy[j][0] != -1) {
//            if (contours[hierarchy[j][0]].size() < 6) {
//                j = hierarchy[j][0];
//                continue;
//            }
//            RotatedRect box2 = fitEllipse(contours[hierarchy[j][0]]);
//            if (box.size.area() / box2.size.area() > 10) {
//                j = hierarchy[j][0];
//                continue;
//            }
//            *(nei_lunkuo + hierarchy[j][0]) = 1;
//            j = hierarchy[j][0];
//        }
//        int z = hierarchy[i][2];
//        while (hierarchy[z][1] != -1) {
//            if (contours[hierarchy[z][1]].size() < 6) {
//                z = hierarchy[z][1];
//                continue;
//            }
//            RotatedRect box2 = fitEllipse(contours[hierarchy[j][0]]);
//            if (box.size.area() / box2.size.area() > 10) {
//                z = hierarchy[z][1];
//                continue;
//            }
//            if (box.size.area() / box2.size.area() > 7)continue;
//            *(nei_lunkuo + hierarchy[z][1]) = 1;
//            z = hierarchy[z][1];
//        }
//        int num = 0;
//        for (int t = 0; t < contours.size(); t++) {
//            if (*(nei_lunkuo + t) == 1) {
//                num++;
//            }
//        }
// //       cout << "内轮廓数量:" << num << endl;
//        if (num == 1) {
//            success = true;
//            if (contours[hierarchy[i][2]].size() < 6)continue;
//            RotatedRect box_buff = fitEllipse(contours[hierarchy[i][2]]);
//            if (box_buff.angle < 5 || box_buff.angle > 175) {
//                if (box.angle > 5 && box.angle < 175)continue;
//            } else {
//                if (!((tan(box_buff.angle * PI / 180) * tan(box.angle * PI / 180) + 1) < 0.3 ||
//                      (box_buff.angle - box.angle) < 5))
//                    continue;
//            }
////            cout<<"轮廓面积最终比："<<(Src.cols*Src.rows)/box_buff.size.area()<<endl;
//            box_buffs.push_back(box_buff);
//            ellipse(Src, box_buff, Scalar(255, 0, 0), 5, CV_AA);
//        }
//        free(nei_lunkuo);
////        if(num == 2){
////            if(contours[hierarchy[i][2]].size()<6)continue;
////            RotatedRect box = fitEllipse(contours[hierarchy[i][2]]);
//
////            ellipse(dst, box, Scalar(0,255,0), 5, CV_AA);
////            if(hierarchy[hierarchy[i][2]][0] != -1){
////                if(contours[hierarchy[hierarchy[i][2]][0]].size()<6)continue;
////                RotatedRect box = fitEllipse(contours[hierarchy[hierarchy[i][2]][0]]);
//
////                ellipse(dst, box, Scalar(0,0,255), 5, CV_AA);
////            }
////        }
//
//    }
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
                    if (area > 100&& aspectRatio < 0.76 && areaRatio > 0.6 ) {
                        ellipse(dst, rect, Scalar(0,255,0), 5, CV_AA);
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
                if (radius < 20 && radius > 5) {
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
                int maxRadius=200;
                //根据半径范围和与短边（锤子柄）的角度筛选出中心R
                if(d1>minRadius && d1 < maxRadius )
                {
                    Mat debug=src.clone();
                    circle_center=p;
                    //circle(Src,p,CV_AA,Scalar(255,0,0),8);
                    //imshow("绘制ing",Src);
                    break;
                }
            }
        }
    }

//    cout<<"完成"<<endl;
    imshow("绘制ing", Src);
//    //保存录像
//    outputVideo<<dst;
//    if(!success)
//        waitKey();
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

