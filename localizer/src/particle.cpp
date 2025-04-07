#include "localizer/particle.hpp"

// デフォルトコンストラクタ
Particle::Particle() : pose_(0.0, 0.0, 0.0)
{
    //記述不要？
}

// コンストラクタ
Particle::Particle(const double x, const double y, const double yaw, const double weight) : pose_(x, y, yaw)//ここで宣言と代入を同時に行う
{
    weight_ = weight;
}

// 代入演算子
Particle& Particle::operator =(const Particle& p)
{
    if(this != &p)//自身への代入を防ぐ
    {
        pose_ = p.pose_;
        weight_ = p.weight_;
    }
    return *this;//自身を返すことで連続代入が可能に
}

// setter
void Particle::set_weight(double weight)
{
    weight_ = weight;
}

// localizer.cppでも使用
// 尤度関数
// センサ情報からパーティクルの姿勢を尤度を算出
double Particle::likelihood(const nav_msgs::msg::OccupancyGrid& map, const sensor_msgs::msg::LaserScan& laser, // この&はポインタではなく普通に参照
    const double sensor_noise_ratio, const int laser_step, const std::vector<double>& ignore_angle_range_list)
{
    double L = 1.0; // 尤度
    // センサ情報からパーティクルの姿勢を評価
    for(double i = 0; i < laser.angle_max - laser.angle_min; i += laser.angle_increment) // laser_step使わなくていいのか?
    {
        if(is_ignore_angle(i,ignore_angle_range_list)) // 柱有り
        {
            L *= 1.0; // そのまま
        }
        else // 柱無し
        {
            // pose_とlaserの角度を使って壁までの距離を推定calc_dist_to_wall
            double estimated_dist_ = calc_dist_to_wall(pose_.x(), pose_.y(), pose_.yaw()+(laser.angle_max-laser.angle_min), map, laser.angle_max-laser.angle_min, sensor_noise_ratio); // これはparticle.cppなのでそのhppにPose pose_が記載されておりこの変数をPoseとして使える

            // 推定値を実測値と比較して尤度算出norm_pdf
            L *= norm_pdf(estimated_dist_,laser.angle_max-laser.angle_min,sensor_noise_ratio);
        }
    }
    return L;
}

// 柱がある範囲か判定
bool Particle::is_ignore_angle(double angle, const std::vector<double>& ignore_angle_range_list)
{
    for (int i = 0; i < ignore_angle_range_list.size(); i += 2)
    {
        double start_angle = ignore_angle_range_list[i];
        double end_angle = ignore_angle_range_list[i + 1];

        // 角度が柱の範囲内にある場合は無視すべき角度
        if (angle >= start_angle && angle <= end_angle)
        {
            return true;
        }
        else
        {
            return false; // 角度が柱の範囲外
        }
    }
}
// ?
// 左右対称であるので，0 < angle < 1, 2 < angle < 3のように設定しても良い．
// リストのサイズを変更しないのであれば，for文を使わずにif文を使ったほうが簡単かも

// 与えられた座標と角度の方向にある壁までの距離を算出
// マップデータが100の場合，距離を返す
// マップデータが-1（未知）の場合，マップ範囲外の場合はsearch_limit * 2.0を返す
// いずれでもない場合は，search_limit * 5.0を返す
double Particle::calc_dist_to_wall(double x, double y, const double laser_angle, const nav_msgs::msg::OccupancyGrid& map,
        const double laser_range, const double sensor_noise_ratio) // laser_angleはレーザの現在の向き角度、laser_rangeはレーザの角度範囲
{
    // 探索のステップサイズ
    const double search_step = map.info.resolution;
    // 最大探索距離
    const double search_limit = laser_range;

    const int width = map.info.width;
    const int height = map.info.height;
    const double resolution = map.info.resolution;
    const double origin_x = map.info.origin.position.x;
    const double origin_y = map.info.origin.position.y;

    // 探索
    for(double dist=0.0; dist<search_limit; dist+=search_step)
    {
        double new_x = x + dist * cos(laser_angle);
        double new_y = y + dist * sin(laser_angle);
        int map_data_size = width*height;
        
        int grid_x = static_cast<int>((x-origin_x)/resolution);//grid座標に変換
        int grid_y = static_cast<int>((y-origin_y)/resolution);//grid座標に変換
        int grid_index=xy_to_grid_index(grid_x,grid_y,map.info);
        int cell_value=map.data[grid_index];
        if(!in_map(grid_index,map_data_size))
        {
            return search_limit*2.0;
        }
        else 
        {
            if (cell_value == 100)
            {
                return dist;  // 壁に衝突した距離を返す
            }
            else if (cell_value == -1)
            {
                return search_limit * 2.0;  // 未知領域
            }
            else
            {
                return search_limit * sensor_noise_ratio * 5.0;
            }
        }
    }
}

// 座標からグリッドのインデックスを返す
int Particle::xy_to_grid_index(const double x, const double y, const nav_msgs::msg::MapMetaData& map_info)
{
    int width = map_info.width;
    
    int grid_index = y * width + x;
    return grid_index;
}

// マップ内か判定
bool Particle::in_map(const int grid_index, const int map_data_size)
{
    if(grid_index > 0 && grid_index < map_data_size)
    {
        return true;
    }

    else 
    {
        return false;
    }
}

// 確率密度関数（正規分布）
double Particle::norm_pdf(const double x, const double mean, const double stddev)
{
    return exp(-1 * (x-mean) * (x-mean) / (2 * stddev * stddev)) / sqrt(2.0 * M_PI * stddev * stddev); // 1 / sqrt(2πσ²) * exp(-(x-μ)² / (2σ²))
}