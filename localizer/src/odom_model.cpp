#include "localizer/odom_model.hpp"

// デフォルトコンストラクタ
OdomModel::OdomModel() {}

// コンストラクタ、クラスの変数を初期化するもの
OdomModel::OdomModel(const double ff, const double fr, const double rf, const double rr)
    : engine_(seed_gen_()), std_norm_dist_(0.0, 1.0), fw_dev_(0.0), rot_dev_(0.0)
{
    fw_var_per_fw_ = ff * ff; // ffの分散、前進方向に走行中の前進方向の分散
    fw_var_per_rot_ = fr * fr; // frの分散、前進方向に走行中の回転方向の分散
    rot_var_per_fw_ = rf * rf; // rfの分散、回転方向に走行中の前進方向の分散
    rot_var_per_rot_ = rr * rr; // rrの分散、回転方向に走行中の回転方向の分散

    fw_dev_ = ff;
    rot_dev_ = rr; // 要らない?
}

// 代入演算子
OdomModel& OdomModel::operator =(const OdomModel& model)
{
    this->fw_var_per_fw_ = model.fw_var_per_fw_;
    this->fw_var_per_rot_ = model.fw_var_per_rot_;
    this->rot_var_per_fw_ = model.rot_var_per_fw_ ;
    this->rot_var_per_rot_ = model.rot_var_per_rot_;

    this->fw_dev_ = model.fw_dev_;
    this->rot_dev_ = model.rot_dev_;

    return *this;
}

// localizer.cppでも使用
// 並進，回転に関する標準偏差の設定
void OdomModel::set_dev(const double length, const double angle)
{
    fw_dev_ = sqrt(fw_var_per_fw_ * length * length + fw_var_per_rot_ * angle * angle); // 前進方向に走行中
    rot_dev_ = sqrt(rot_var_per_fw_ * length * length + rot_var_per_rot_ * angle * angle); // 回転方向に走行中
}

// localizer.cppでも使用
// 直進に関するノイズ（fw_dev_）の取得
double OdomModel::get_fw_noise()
{
    return fw_dev_ * std_norm_dist_(engine_); // 正規分布からサンプリング 
}

// localizer.cppでも使用
// 回転に関するノイズ（rot_dev_）の取得
double OdomModel::get_rot_noise()
{
    return rot_dev_ * std_norm_dist_(engine_); // 正規分布からサンプリング
}