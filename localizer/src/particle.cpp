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
    double L = 0.0; // 尤度(掛け算だったら1.0)
    // センサ情報からパーティクルの姿勢を評価
    // printf("before_for\n");
    for(int i = 0; i < laser.ranges.size(); i++) // i+=laser_stepにすれば間引きのif重ねず済んだ
    // for(double i = laser.angle_min; i <= laser.angle_max; i++) // iは配列のインデックスなためdoubleじゃバグる
    {
        int check_laser = 1; // for文で回されるチェックするレーザの本数
        if (check_laser % laser_step != 0) // そこから間引き
        {
            // if(is_ignore_angle(i,ignore_angle_range_list)) // 柱有り、掛け算のときはこのif文付ける
            // {
            //     L *= 1.0; // そのまま
            //     // printf("L_wall=%lf\n", L);
            // }
            
            if(!is_ignore_angle(i,ignore_angle_range_list)) // 柱なし
            // else // 柱無し、掛け算のとき
            {
                // pose_とlaserの角度を使って壁までの距離を推定calc_dist_to_wall
                double theta = pose_.yaw() + laser.angle_min + i * laser.angle_increment;
                // double theta = pose_.yaw()+(laser.angle_max-laser.angle_min);
                // double theta = pose_.yaw();
                double estimated_dist_ = calc_dist_to_wall(pose_.x(), pose_.y(), theta, map, laser.angle_max-laser.angle_min, sensor_noise_ratio); //  地図視点の壁からの距離
                // double estimated_dist_ = calc_dist_to_wall(pose_.x(), pose_.y(), theta, map, laser.ranges[i], sensor_noise_ratio);
                // printf("estimated_dist=%lf\n", estimated_dist_);

                // 推定値を実測値と比較して尤度算出norm_pdf
                // double distance_error = std::abs(estimated_dist_ - laser.ranges[i]); // ！実測値との誤差で計算したい
                // printf("distance_error=%lf\n", distance_error);
                // L *= norm_pdf(distance_error,laser.angle_max-laser.angle_min,sensor_noise_ratio);
                // L *= norm_pdf(distance_error,0.0,sensor_noise_ratio);
                L += norm_pdf(estimated_dist_,laser.ranges[i],sensor_noise_ratio); // 足し算
                // L += norm_pdf(estimated_dist_,laser.ranges[i],laser.ranges[i]*sensor_noise_ratio); // 足し算
                // printf("L_nonwall=%lf\n", L);
            }
        }
        check_laser++;
    }
    // printf("L=%lf\n", L);

    // 安全対策
    if (L < 1.0) {
        L = 1.0; // 最小尤度
    }
    return L;
}

// 柱がある範囲か判定
bool Particle::is_ignore_angle(double angle, const std::vector<double>& ignore_angle_range_list)
{
    // printf("before_for\n");
    double start_angle = 0.0;
    double end_angle = 0.0;

    for (int i = 0; i < ignore_angle_range_list.size(); i += 2)
    {
        // printf("before_ifelse\n");
        start_angle = ignore_angle_range_list[i]; // 下限
        end_angle = ignore_angle_range_list[i + 1]; // 上限
        // printf("start_angle=%lf\n", start_angle);
        // printf("end_angle=%lf\n", end_angle);

        // 角度が柱の範囲内にある場合は無視すべき角度
        if (angle >= start_angle && angle <= end_angle)
        {
            // printf("true\n");
            return true;
        }
        // else
        // {
        //     // printf("false\n");
        //     return false; // 角度が柱の範囲外
        // }
    }
    return false; // 角度が柱の範囲内にないことを全ての角度リストについてfor文で確認したらfalse返す
}
// 左右対称であるので，0 < angle < 1, 2 < angle < 3のように設定しても良い．
// リストのサイズを変更しないのであれば，for文を使わずにif文を使ったほうが簡単かも

// 与えられた座標と角度の方向にある壁までの距離を算出
// マップデータが100の場合，距離を返す
// マップデータが-1（未知）の場合，マップ範囲外の場合はsearch_limit * 2.0を返す
// いずれでもない場合は，search_limit * 5.0を返す
double Particle::calc_dist_to_wall(double x, double y, const double laser_angle, const nav_msgs::msg::OccupancyGrid& map,
        const double laser_range, const double sensor_noise_ratio) // x,yはパーティクルの現在位置、laser_angleはレーザの現在の向き角度、laser_rangeはレーザの角度範囲
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
        
        int grid_x = static_cast<int>((new_x-origin_x)/resolution);//grid座標に変換
        int grid_y = static_cast<int>((new_y-origin_y)/resolution);//grid座標に変換
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
        }
    }
    return search_limit * sensor_noise_ratio * 5.0;
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
double Particle::norm_pdf(const double x, const double mean, const double stddev) // 予測値、観測値、ノイズの大きさ
{
    return exp((-1) * (x-mean) * (x-mean) / (2 * stddev * stddev)) / sqrt(2.0 * M_PI * stddev * stddev); // 1 / sqrt(2πσ²) * exp(-(x-μ)² / (2σ²))
}