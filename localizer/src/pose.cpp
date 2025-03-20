#include "localizer/pose.hpp"

// デフォルトコンストラクタ
Pose::Pose()
{

}

// コンストラクタ
Pose::Pose(const double x, const double y, const double yaw)
{
    x_ = x;
    y_ = y;
    yaw_ = yaw;
}

// 代入演算子
Pose& Pose::operator =(const Pose& pose)
{
    this->x_ = pose.x_;
    this->y_ = pose.y_;
    this->yaw_ = pose.yaw_;
    return *this;
}

// 複合代入演算子/=
Pose& Pose::operator /=(const double a)
{
    if (a != 0)
    {
        this->x_ /= a;
        this->y_ /= a;
        this->yaw_ /= a;
    }
    return *this;
}

// setter
void Pose::set(const double x, const double y, const double yaw)
{
   this->x_ = x;
   this->y_ = y;
   this->yaw_ = yaw; 
}

// パーティクルの移動
// ノイズを加えて，移動させる
void Pose::move(double length, double direction, double rotation, const double fw_noise, const double rot_noise)
{
    
}

// 適切な角度(-M_PI ~ M_PI)に変更
void Pose::normalize_angle()
{
    double angle = 0.0;
    while (angle < -M_PI && M_PI < angle) { // 適切な範囲外のとき
        if (angle < -M_PI) {
            angle += 2 * M_PI;
        } else 
            angle -= 2 * M_PI;
    }
}