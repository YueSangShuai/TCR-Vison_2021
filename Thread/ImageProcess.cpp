/*
	* Date: 2022.3.24
	* Details: 图像获取和处理控制
	* Remarks: 利用多线程异步处理图像的获取与处理步骤，提高cpu的利用率，提高程序运行速度，图像的获取与处理控制采用生产者和消费者的方法
*/
#include "opencv2/highgui/highgui.hpp"
#include "../header/ImageProcess.h"
#include "../DaHengCamera/DaHengCamera.h"
#include<sys/timeb.h>
#include "../buff_include/FindBuff.h"
#include "../buff_include/BuffAngleSolver.h"

#include<opencv2/highgui.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/opencv.hpp>

// 模式设置
#define USING_CAMERA			// 使用大恒相机获取图像
// #define USING_VIDEO			// 使用视频进行调试
// #define DEBUG_ARMOR			// 利用装甲板视频进行调试
// #define DEBUG_BUFF			// 利用能量机关视频进行调试
#define ANGLESOLVER		        // 是否进行角度解算

#define BUFFER 5		// 线程间相机采集最高超过处理的帧数
ImageDate Image[BUFFER];
static volatile unsigned int proIdx = 0;    // 生产id
static volatile unsigned int consIdx = 0;   // 消费id
double run_time = 0;        // 记录时间
bool is_first_set=false;
// 初始化数据
// int targetNum = 2;			// 设置需要击打的装甲板目标数字
// Color ENEMYCOLOR = RED;	// 设置敌方战车灯条颜色
double fps;
bool USE_BUFFER = false;
ImageGetMode last_model = DAHENG;


FindBuff BuffDetector;
BuffAngleSolver BuffAngleData;


// 导入装甲板识别类
ArmorDetector detector;
// 导入角度解算类
#ifdef ANGLESOLVER
AngleSolver angleSolver;
#endif

// 无参数构造函数
ImageProcess::ImageProcess() {
    proIdx = 0;
    consIdx = 0;
}

// 使用大恒相机获取图像进行调试
DaHengCamera CameraTure;

// 获取图像函数
void ImageProcess::ImageProducter() {
#ifdef DEBUG_ARMOR		// 利用装甲板视频进行调试
    VideoCapture cap(Armor_videopath);
#endif
#ifdef DEBUG_BUFF		// 利用能量机关视频进行调试
    VideoCapture cap(Buff_videopath);
#endif

#ifdef USING_CAMERA

    // 链接设备
    CameraTure.StartDevice(1);
    // 设置分辨率(设置分辨率要在摄像头开始采集之前)
    CameraTure.SetResolution(2,2);
    CameraTure.SetExposureTime(0);
    // 设置曝光值
    CameraTure.SetExposureTime(GXExpTimeValue);
    // 设置曝光增益
    CameraTure.SetGAIN(0,GXGain);
    // 关闭自动白平衡
    CameraTure.Set_BALANCE_AUTO(1);
    // 控制设备开采
    CameraTure.SetStreamOn();

#endif // USING_CAMERA

    // 生产部分循环
    do {
        while (proIdx - consIdx >= BUFFER);
        // 挂资源锁
        reciveRes.lock();
        // 解锁
        reciveRes.unlock();
        cout << "进入生产!" << endl;
        ImageDate Src;
        // 使用收数资源
        // 挂资源锁
        reciveRes.lock();
        Src.ReciveStm32.BeginToNowTime = getStm32.BeginToNowTime;
        Src.ReciveStm32.pitch = getStm32.pitch;
        Src.ReciveStm32.yaw = getStm32.yaw;
        Src.ReciveStm32.ShootSpeed = getStm32.ShootSpeed;
        Src.mode = getStm32.model_;
        Src.ReciveStm32.c=getStm32.c;
        cout<<"颜色为:"<<Src.ReciveStm32.c<<endl;
        if (Src.mode == last_model && USE_BUFFER == false)
        {
            USE_BUFFER = false;
        }
        else if (Src.mode == last_model && USE_BUFFER == true)
        {
            USE_BUFFER = true;
        }
        else if (Src.mode != last_model && USE_BUFFER == false)
        {
            USE_BUFFER = true;
        }
        else if (Src.mode != last_model && USE_BUFFER == true)
        {
            USE_BUFFER = false;
        }
        cout << "得到收数时间" << getStm32.BeginToNowTime << endl;
        cout<< "收数pitch:" << Src.ReciveStm32.pitch << "    yaw:" << Src.ReciveStm32.yaw << "   射速:" << getStm32.ShootSpeed << endl << endl;
        cout<<"颜色:"<<Src.ReciveStm32.c<<endl;
        // 解锁
        reciveRes.unlock();

#ifdef DEBUG_ARMOR		// 利用装甲板视频进行调试
        cap >> Src.SrcImage;
        Src.mode = VIDEO;
#endif
#ifdef DEBUG_BUFF		// 利用能量机关视频进行调试
        cap >> Src.SrcImage;
        Src.mode = BUFF;
#endif

#ifdef USING_CAMERA
        // 使用大恒相机获取图像
        CameraTure.GetMat(Src.SrcImage);
        if (Src.SrcImage.cols == 0) {
            cout << "丢帧!" << endl;
            continue;
        }
        // Src.mode = DAHENG;
#endif // USING_CAMERA
        Image[proIdx % BUFFER] = Src;       // 存入图像

        proIdx++;
        cout << "生产完成: " << proIdx - 1 << endl;

    } while (bool_Run);

#ifdef DEBUG_ARMOR
    cap.release();
#endif
#ifdef DEBUG_BUFF
    cap.release();
#endif
}

// 消费图像函数
void ImageProcess::ImageConsumer() {
    // 导入数字识别模板
    detector.loadSVM("../123svm.xml");

#ifdef USING_CAMERA
    #ifdef ANGLESOLVER
    // 使用相机
    // 设置基本参数
    angleSolver.setCameraParam("camera_params.xml", 1);
    angleSolver.setArmorSize(SMALL_ARMOR, 135, 55);
    angleSolver.setArmorSize(BIG_ARMOR, 230, 55);
#endif
#endif

    double t, t1;
    do {
        while (consIdx >= proIdx);
        t = static_cast<double>(getTickCount());     // 记录时间
        int ImageIndex = proIdx - 1;              // 获取最新图像的index
	    ArmorDate ArmDate;

//        if (USE_BUFFER == false)
//            Image[(ImageIndex) % BUFFER].mode = DAHENG;
//        else
            Image[ImageIndex%BUFFER].mode=BUFF;
        if (Image[(ImageIndex) % BUFFER].mode == DAHENG) {        // 自瞄(非BUFF模式)
            // 设置敌方战车灯条颜色
            detector.setEnemyColor(ENEMYCOLOR);
            if (Image[(ImageIndex) % BUFFER].mode == VIDEO) {   // 读取视频模式
                Image[(ImageIndex) % BUFFER].SrcImage.copyTo(src);      // 将获取到的图像copy to src中
                if (src.empty())
                    break;
                detector.setImg(src);       // 装甲板识别图像预处理
                ArmDate = detector.run(src);          // 装甲板识别综合函数
            }
            else if (Image[(ImageIndex) % BUFFER].mode == DAHENG) {     // 读取大恒相机模式
                Image[(ImageIndex) % BUFFER].SrcImage.copyTo(src);      // 将获取到的图像copy to src中
                if (src.empty())
                    break;
                detector.setImg(src);       // 装甲板识别图像预处理
                ArmDate = detector.run(src);          // 装甲板识别综合函数
            }

            // 获取装甲板识别后的相关数据 用于后续操作以及DEBUG信息输出
            // vector<Point2f> contourPoints;
            // ArmorType type;
            // Point2f centerPoint;
            // ArmorBox BestArmor;
            // pattern Car_model;
            // if (detector.isFoundArmor())
                // detector.getTargetInfo(BestArmor, contourPoints, centerPoint, type, Car_model);

            // targetNum = 2;
            // 设置目标装甲板数字
            // detector.setTargetNum(targetNum);

            // 角度解算
#ifdef ANGLESOLVER
            double yaw = 0, pitch = 0, distance = 0;
	    // 输出击打状态
	    if (ArmDate.Car_model == 0)
		cout << "Car_model =================== FirstFind" << endl;
	    else if (ArmDate.Car_model == 1)
		cout << "Car_model =================== Shoot" << endl;
	    else if (ArmDate.Car_model == 2)
		cout << "Car_model =================== Stop" << endl;
	    else if (ArmDate.Car_model == 3)
		cout << "Car_model =================== buffering" << endl;
	    // 输出装甲板的类型
	    cout << "Armor_type========================" << ArmDate.type << endl;
            if (ArmDate.Car_model == buffering) {       // 缓冲模式
                // 缓冲状态击打
                // angleSolver.BufferSetFilter(ArmDate.BestArmor, Image[(ImageIndex) % BUFFER].ReciveStm32, pitch, yaw);
		pitch = 0;
		yaw = 0;
            }
            else if (ArmDate.Car_model == FirstFind || ArmDate.Car_model == Shoot) {        // 首次识别模式，连续射击模式
                // 正常射击状态
                angleSolver.getAngle(ArmDate.Car_model, ArmDate.BestArmor, Image[(ImageIndex) % BUFFER].ReciveStm32, ArmDate.contourPoints, ArmDate.centerPoint, ArmDate.type, yaw, pitch, distance);
            }

            // 发送数据给端口，为发送串口数据给下位机做准备
            ArmorToData(ArmDate.Car_model, pitch, yaw);

            //********************** DEGUG **********************//
            //装甲板检测识别调试参数是否输出
            //param:
            //		1.showSrcImg_ON,		  是否展示原图
            //		2.bool showSrcBinary_ON,  是否展示二值图
            //		3.bool showLights_ON,	  是否展示灯条图
            //		4.bool showArmors_ON,	  是否展示装甲板图
            //		5.bool textLights_ON,	  是否输出灯条信息
            //		6.bool textArmors_ON,	  是否输出装甲板信息
            //		7.bool textScores_ON	  是否输出打击度信息
            //					   1  2  3  4  5  6  7
            detector.showDebugInfo(1, 1, 1, 1, 0, 0, 0);
#endif // ANGLESOLVER

        }
        else if (Image[(ImageIndex) % BUFFER].mode == BUFF){          // 能量机关(BUFF模式)
            float buff_pitch,buff_yaw;
            RM_BuffData *Buffs = BuffDetector.BuffModeSwitch(Image[(ImageIndex) % BUFFER].SrcImage,Image[(ImageIndex) % BUFFER].ReciveStm32.c);
            if (Buffs != (RM_BuffData *) -1)
            {
                BuffAngleData.GetBuffShootAngle(Buffs, BUFF_CONTINUE_SHOOT,
                                                Image[(ImageIndex) % BUFFER].ReciveStm32);
                for (int i = 0; i < 4; i++) {
                    line(Image[(ImageIndex) % BUFFER].SrcImage, Buffs[3].point[i % 4], Buffs[3].point[(i + 1) % 4],
                         Scalar(0, 0, 255), 4, 4);
                }
                //circle(Image[(ImageIndex) % BUFFER].SrcImage,Buffs[3].circle_center,CV_AA,Scalar(255,0,0),3);
                //上-pith 右+yaw
                buff_pitch=-Buffs[3].pitch+9.7;
                buff_yaw=Buffs[3].yaw+14.7;

                circle(Image[(ImageIndex) % BUFFER].SrcImage,Buffs[3].predict,CV_AA,Scalar(255,0,255),3);
                char test[100];
                sprintf(test, "angle:%0.4f", Buffs[3].armoranle*57.3);
                cv::putText(Image[(ImageIndex) % BUFFER].SrcImage, test, cv::Point(10, 20),
                            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);
                sprintf(test, "yaw:%0.4f ", buff_yaw);
                cv::putText(Image[(ImageIndex) % BUFFER].SrcImage, test, cv::Point(10, 40),
                            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);
                sprintf(test, "pitch:%0.4f ", buff_pitch);
                cv::putText(Image[(ImageIndex) % BUFFER].SrcImage, test,
                            cv::Point(Image[(ImageIndex) % BUFFER].SrcImage.cols / 3, 40), cv::FONT_HERSHEY_SIMPLEX,
                            1, cv::Scalar(255, 255, 255), 1, 8);
                //cv::namedWindow("绘制",WINDOW_FULLSCREEN);
#ifdef IMSHOW_IMAGE
                imshow("绘制",Image[(ImageIndex)%BUFFER].SrcImage);
#endif
                ArmorToData(Shoot,buff_pitch,buff_yaw);
            }
            else
            {
                buff_pitch=0;
                buff_yaw=0;
                ArmorToData(Shoot,buff_pitch,buff_yaw);
            }

        }
//        else
//            continue;

        // FPS
//        t1 = static_cast<double>((getTickCount() - t) / getTickFrequency());
//        printf("Armor Detecting FPS: %f\n", 1 / t1);

#ifdef DEBUG_MODE
        char chKey = cv::waitKey(1);       // 修改waitKey获取的值可以控制视频读取的速率
        switch (chKey) {
            case 'b':           // 输入b/B可将敌方战车颜色设置为蓝色
            case 'B':
                ENEMYCOLOR = BLUE;
                break;
            case 'r':           // 输入r/R可将敌方战车颜色设置为红色
            case 'R':
                ENEMYCOLOR = RED;
                break;
            case 'q':           // 输入q/Q可退出程序循环
            case 'Q':
            case 27:
                bool_Run = false;
                break;
            default:
                break;
        }
#endif


        consIdx = ImageIndex + 1;
        cout << "消费完成" << endl;
    } while (bool_Run);
}

// 保存参数函数
void ImageProcess::SaveData() {
    // 文件读取
    // OpenCV中可以使用FileStorage对xml,yml等文件进行读写操作
    FileStorage fs_param(paramFileName, FileStorage::WRITE);    // 进行写操作
    if (!fs_param.isOpened()) {
        cout << "参数文件打开失败!" << endl;
        exit(1);
    }
}
