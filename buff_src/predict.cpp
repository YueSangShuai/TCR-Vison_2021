//
// Created by rmtcr on 2022/5/27.
//

#include "../buff_include/predict.h"
static int is_count=50;
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
double predict::NiHe(double del_time,double del_angle,int tao,int sample) {
    double _phi[1]={0.0};
    problem->AddResidualBlock (     // 向问题中添加误差项
            // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 1> (
                    //以开始拟合的一帧作为时间轴原点，其余帧与其计算相对时间
                    new CURVE_FITTING_COST (del_time,(double)del_angle,tao)
            ),
            nullptr,            // 核函数，这里不使用，为空
            _phi                 // 待估计参数
    );
    if(sample>is_count){
        ceres::Solver::Summary summary;                // 优化信息
        ceres::Solve ( options,problem, &summary );  // 开始优化
        phi=_phi[0];
        last_phi=phi;
    }else{
        std::cout<<"未能信任这份结果"<<std::endl;
    }
    delete problem;
}
float predict::pos_fun(float t,float phi)
{
    return (1.305 * t) - (0.416666666666667 * cos(1.884 *t + phi ));
}
Point2f predict::getPredictPoint(double predictime) {

}