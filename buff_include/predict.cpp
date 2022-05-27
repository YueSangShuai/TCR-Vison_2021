//
// Created by rmtcr on 2022/5/27.
//

#include "../buff_src/predict.h"
//非线性拟合代价函数
struct CURVE_FITTING_COST
{
    //构造函数
    CURVE_FITTING_COST ( double x, double y,float tau) : _tau(tau),_x ( x ), _y ( y ) {}
    // 残差的计算
    float _tau;
    template <typename T>
    bool operator() (
            const T* const phi,       // 待拟合参数，有1维
            T* residual ) const     // 残差
    {
        auto value = 1.305*_tau + 0.41666666667*(-ceres::cos(phi[0]+(1.884*_x)) + ceres::cos(phi[0]+1.884*(-_tau+_x)));
//        auto value = 1.305*_tau+0.4166666667*(-2)* sin(-1.884*_tau/2)*ceres::sin(phi[0]+1.884*_x+(1.884*_tau/2));
        residual[0] = T(_y) - value;
        return true;
    }
    const double _x, _y;    // x,y数据
};
predict::predict(){
    options.linear_solver_type = ceres::DENSE_QR;   // 增量方程如何求解
    options.minimizer_progress_to_stdout = false;   // 不输出到控制台
    options.num_threads=3;

}