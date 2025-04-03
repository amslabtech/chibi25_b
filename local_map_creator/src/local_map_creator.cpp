#include "local_map_creator/local_map_creator.hpp"

// コンストラクタ
LocalMapCreator::LocalMapCreator() : Node("local_map_creater")
{
    // パラメータの取得(hz, map_size, map_reso)
    this->get_parameter("hz", hz_);
    this->get_parameter("map_size", map_size_);
    this->get_parameter("map_reso", map_reso_);

    // Subscriberの設定
    // <subscriber名> = this->create_subscription<<msg型>>("<topic名>", rclcpp::QoS(<確保するtopicサイズ>).reliable(), std::bind(&<class名>::<コールバック関数名>, this, std::placeholders::_1));
    // std::bindを使ってsubするコールバック関数，std::placeholdersを使ってその関数内での引数を指定する
    // std::placeholdersで指定する引数は大体1番目のもの（コールバック関数の引数が1つであるため）
    sub_obs_poses_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/o_points", rclcpp::QoS(1).reliable(), std::bind(&LocalMapCreator::obs_poses_callback, this, std::placeholders::_1));

    // Publisherの設定
    // <publisher名> = this->create_publisher<<msg型>>("<topic名>", rclcpp::QoS(<確保するtopicサイズ>).reliable());
    pub_local_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/l_map", rclcpp::QoS(1).reliable());

    // --- 基本設定 ---
    // マップの基本情報(local_map_)を設定する（header, info, data）
    //   header:フレームIDとタイムスタンプ
    local_map_.header.stamp = this->now();
    local_map_.header.frame_id = "map";

    //   info(width, height, position.x, position.y):マップの解析度、サイズ、原点位置など
    local_map_.info.width = static_cast<int>(map_size_ / map_reso_); // グリッド数で表す
    local_map_.info.resolution = map_reso_;
    local_map_.info.height = static_cast<int>(map_size_ / map_reso_); // グリッド数で表す
    local_map_.info.origin.position.x = -map_size_ / 2.0; // .info.origin.positionはローカルマップの原点でマップの左下の座標を表す
    local_map_.info.origin.position.y = -map_size_ / 2.0;
    local_map_.info.origin.position.z = 0.0;
    local_map_.info.origin.orientation.x = 0.0;
    local_map_.info.origin.orientation.y = 0.0;
    local_map_.info.origin.orientation.z = 0.0;
    local_map_.info.origin.orientation.w = 1.0;

    //   data:実際のマップデータ(local_map_.data[]の0:空き, 100:障害物, -1:未知)
    local_map_.data.resize(local_map_.info.width * local_map_.info.height, -1); // ☆マップの初期化とは違いマップのサイズを確保し、全て未知-1に
}

// obs_posesのコールバック関数
void LocalMapCreator::obs_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    obs_poses_ = *msg;
    flag_obs_poses_ = true;
}

// 周期処理の実行間隔を取得する
int LocalMapCreator::getFreq()
{
    return hz_; // ループ周波数の数値を返す
}

// 障害物情報が更新された場合、マップを更新する
void LocalMapCreator::process()
{
    if (flag_obs_poses_) {
        update_map();
        flag_obs_poses_ = false; // またmsg取得したときコールバックでtrue判定できるように戻しておく
    }
}

// 障害物の情報をもとにローカルマップを更新する
void LocalMapCreator::update_map()
{
    // マップを初期化する
    init_map();

    // 障害物の位置を考慮してマップを更新する
    for (const auto &pose : obs_poses_.poses) // 障害物の位置情報
    {
        int index = xy_to_grid_index(pose.position.x, pose.position.y); // (x, y)座標をグリッド数で示す
        if (index >= 0 && index < static_cast<int>(local_map_.data.size()))
        {
            local_map_.data[index] = 100; // 障害物を示す値
        }
    }

    local_map_.header.stamp = this->now();

    // 更新したマップをpublishする
    // <publisher名>->publish(<変数名>);
    pub_local_map_->publish(local_map_);
}

// マップの初期化(すべて「未知」にする)
void LocalMapCreator::init_map()
{
    std::fill(local_map_.data.begin(), local_map_.data.end(), -1); // ☆サイズは決まっているから新しく確保せずに内容だけをリセットする
}

// マップ内の場合、trueを返す
bool LocalMapCreator::in_map(const double dist, const double angle)
{
    // 指定された距離と角度がマップの範囲内か判定する
    double x = dist * cos(angle);
    double y = dist * sin(angle);
    if (x >= -map_size_ / 2.0 && x <= map_size_ / 2.0 && y >= -map_size_ / 2.0 && y <= map_size_ / 2.0) {
        return true;
    }
}

// 距離と角度からグリッドのインデックスを返す
int LocalMapCreator::get_grid_index(const double dist, const double angle){
    if (in_map(dist, angle)) {
        double x = dist * cos(angle);
        double y = dist * sin(angle);
        return xy_to_grid_index(x, y);
    }
}

// 座標からグリッドのインデックスを返す
int LocalMapCreator::xy_to_grid_index(const double x, const double y){
    int grid_x = static_cast<int>((x + map_size_ / 2.0) / map_reso_);
    int gridc_y = static_cast<int>((y + map_size_ / 2.0) / map_reso_);
    return grid_y * local_map_.info.width + grid_x;
}