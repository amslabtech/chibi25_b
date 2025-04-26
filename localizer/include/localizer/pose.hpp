#ifndef POSE_HPP
#define POSE_HPP

#include <cmath>

class Pose
{
    public:
        Pose(); // デフォルトコンストラクタ
        Pose(const double x, const double y, const double yaw); // コンストラクタ
        Pose& operator =(const Pose& pose); // 代入演算子
        Pose& operator /=(const double a);  // 複合代入演算子/=

        // accessor
        void set(const double x, const double y, const double yaw);
        double x()   const { return x_; } // x_, y_, yaw_はprivateなのでアクセサ関数を使う、変数名.x()等
        double y()   const { return y_; } // 返り値がconstで値を返すだけの代入が出来ないのでその時は変数名.set(x, y, yaw);使う
        double yaw() const { return yaw_; }

        // パーティクルの移動
        void move(double length, double direction, double rotation, const double fw_noise, const double rot_noise);

        // 適切な角度(-M_PI ~ M_PI)に変更
        static void normalize_angle(double &angle);

    private:
        double x_;   // [m]
        double y_;   // [m]
        double yaw_; // [rad]
};

#endif