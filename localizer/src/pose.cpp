
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

// パーティクルの移動(位置と向きの更新)
// ノイズを加えて，移動させる
void Pose::move(Pose &pose, double length, double direction, double rotation, const double fw_noise, const double rot_noise) // lengthは直進距離
{
    // ノイズを加えるc
    double noisy_length = length + fw_noise * std::sqrt(std::abs(length));
    double noisy_rotation = rotation + rot_noise * std::sqrt(std::abs(rotation));

    // 移動させる
    pose.x_ += noisy_length * std::cos(pose.yaw_ + direction); // 進行方向は現在の向き+移動方向
    pose.y_ += noisy_length * std::sin(pose.yaw_ + direction);
    pose.yaw_ += noisy_rotation; // この関数は他のcppでも使えるstatic関数(クラスをクラス名::で指定して使える)なためx_等のメンバ変数を使えないしthis->x_もできない、引数にPose追加

    // 角度を適切化
    normalize_angle(pose.yaw_); // この関数内で使えるようにstatic関数に合わせた
}

// 適切な角度(-M_PI ~ M_PI)に変更
void Pose::normalize_angle(double &angle)
{
    while (angle < -M_PI && M_PI < angle) { // 適切な範囲外のとき
        if (angle < -M_PI) {
            angle += 2 * M_PI;
        } else 
            angle -= 2 * M_PI;
    }
}