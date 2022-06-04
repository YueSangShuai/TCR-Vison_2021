//
// Created by rmtcr on 2022/5/27.
//

#ifndef RM_DRACURVE_H
#define RM_DRACURVE_H
#include"../header/General.h"

class DrawCurve{
public:
    void ClearSaveData();
    void InsertData(float Data);
    void InsertData(float Data1,float Data2,string s1,string s2,string window_name);
};

#endif // DRAWCURVE_H

