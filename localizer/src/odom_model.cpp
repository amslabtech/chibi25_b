#include "localizer/odom_model.hpp"

// デフォルトコンストラクタ
OdomModel::OdomModel() {}

// コンストラクタ、クラスの変数を初期化するもの
OdomModel::OdomModel(const double ff, const double fr, const double rf, const double rr)
    : engine_(seed_gen_()), std_norm_dist_(0.0, 1.0), fw_dev_(0.0), rot_dev_(0.0)
{
    fw_var_per_fw_ = ff; // ffの分散、前進方向に走行中の前進方向の分散
    fw_var_per_rot_ = fr; // frの分散、前進方向に走行中の回転方向の分散
    rot_var_per_fw_ = rf; // rfの分散、回転方向に走行中の前進方向の分散
    rot_var_per_rot_ = rr; // rrの分散、回転方向に走行中の回転方向の分散
}

// 代入演算子
OdomModel& OdomModel::operator =(const OdomModel& model)
{
    this->fw_var_per_fw_ = model.fw_var_per_fw_;
    this->fw_var_per_rot_ = model.fw_var_per_rot_;
    this->rot_var_per_fw_ = model.rot_var_per_fw_ ;
    this->rot_var_per_rot_ = model.rot_var_per_rot_;
    return *this;
}

// 並進，回転に関する標準偏差の設定
void OdomModel::set_dev(const double length, const double angle)
{
    fw_dev_ = sqrt(fw_var_per_fw_ * length * length + fw_var_per_rot_ * angle * angle); // 前進方向に走行中
    rot_dev_ = sqrt(rot_var_per_fw_ * length * length + rot_var_per_rot_ * angle * angle); // 回転方向に走行中
}

// 直進に関するノイズ（fw_dev_）の取得→localizer.cppで使用
double OdomModel::get_fw_noise()
{
    return fw_dev_ * std_norm_dist_(engine_); // 正規分布からサンプリング 
}

// 回転に関するノイズ（rot_dev_）の取得→localizer.cppで使用
double OdomModel::get_rot_noise()
{



}


// コンストラクタ
OdomModel::OdomModel(const double ff, const double fr, const double rf, const double rr)
    : ff_(ff), fr_(fr), rf_(rf), rr_(rr), engine_(seed_gen_()), std_norm_dist_(0.0, 1.0)
{
    fw_var_per_fw_ = ff_;  // 直進のばらつき
    fw_var_per_rot_ = fr_; // 回転による直進のばらつき
    rot_var_per_fw_ = rf_; // 直進による回転のばらつき
    rot_var_per_rot_ = rr_; // 回転のばらつき
}

// 代入演算子
OdomModel& OdomModel::operator =(const OdomModel& model)
{
    this->ff_ = model.ff_;
    this->fr_ = model.fr_;
    this->rf_ = model.rf_;
    this->rr_ = model.rr_;
    return *this;
}

// 並進，回転に関する標準偏差の設定
void OdomModel::set_dev(const double length, const double angle)
{
    fw_dev_ = std::sqrt(fw_var_per_fw_ * length * length + fw_var_per_rot_ * angle * angle);
    rot_dev_ = std::sqrt(rot_var_per_fw_ * length * length + rot_var_per_rot_ * angle * angle);
}

// 直進に関するノイズ（fw_dev_）の取得
double OdomModel::get_fw_noise()
{
    return fw_dev_ * std_norm_dist_(engine_); // 正規分布からサンプリング
}

// 回転に関するノイズ（rot_dev_）の取得
double OdomModel::get_rot_noise()
{

    return rot_dev_ * std_norm_dist_(engine_); // 正規分布からサンプリング
}