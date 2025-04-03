#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "localizer/pose.hpp"

class Particle
{
    public:
        Particle(); // デフォルトコンストラクタ
        Particle(const double x, const double y, const double yaw, const double weight); // コンストラクタ
        Particle& operator =(const Particle& p); // 代入演算子

        // accessor
        void set_weight(const double weight);   // 重みのセット、Particleの変数名.set_weight(引数)で使える
        double weight() const { return weight_; } // 値を返すだけの関数(右辺)だから、重みの値を変え直接代入するとき(*=)はset_weight()を使うか、double& にして参照を返すと.weight()を直接変更できる

        // 尤度関数、他のcppでも使うためクラス::で指定して使用できるstatic関数を用いてこの関数内の関数もstaticにする
        static double likelihood(const nav_msgs::msg::OccupancyGrid& map, const sensor_msgs::msg::LaserScan& laser,
            const double sensor_noise_ratio, const int laser_step, const std::vector<double>& ignore_angle_range_list);

        // --- メンバ変数 ---
        Pose pose_;
        Pose& getPose() { return pose_; } // 返り値がconstでないからconst Particleからはこの関数を呼び出せない

    private:
        // 柱か判断
        static bool is_ignore_angle(double angle, const std::vector<double>& ignore_angle_range_list);

        // 壁までの距離を算出
        static double calc_dist_to_wall(double x, double y, const double laser_angle, const nav_msgs::msg::OccupancyGrid& map,
            const double laser_range, const double sensor_noise_ratio);

        // 座標からグリッドのインデックスを返す
        static int  xy_to_grid_index(const double x, const double y, const nav_msgs::msg::MapMetaData& map_info);

        // マップ内か判断
        static bool in_map(const int grid_index, const int map_data_size);

        // 確率密度関数（正規分布）
        static double norm_pdf(const double x, const double mean, const double stddev);

        // --- メンバ変数 ---
        double weight_; // [-]
};

#endif
