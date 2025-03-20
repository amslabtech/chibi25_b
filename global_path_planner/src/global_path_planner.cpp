#include "global_path_planner/global_path_planner.hpp"

using namespace std::chrono_literals;

// デフォルトコンストラクタ
// パラメータの宣言と取得
Astar::Astar() : Node("teamb_path_planner"), clock_(RCL_ROS_TIME)
{
    // ###### パラメータの宣言 ######
    robot_radius_ = this->declare_parameter<double>("robot_radius",0.3);
    //  = this->declare_parameter<double>("origin_y");


    // ###### パラメータの取得 ######


    // ###### global_path_とcurrent_node_のframe_id設定 ######


    // dataサイズの確保
    global_path_.poses.reserve(2000);


    // ####### Subscriber #######
    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable(),std::bind(&Astar::map_callback,this,std::placeholders::_1));

    // ###### Publisher ######
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/g_path",rclcpp::QoS(1).reliable());
    pub_node_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/pub_node_point",rclcpp::QoS(1).reliable());
    pub_current_path_ = this->create_publisher<nav_msgs::msg::Path>("/g_path",rclcpp::QoS(1).reliable());
    pub_new_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_new",rclcpp::QoS(1).reliable());
}

// mapのコールバック関数
// msgを受け取り，map_に代入，その情報をそれぞれ取得
// process()を実行
void Astar::map_callback(const nav_msgs::msg::OccupancyGrid msg)  //マップの読み込み
{
    map_ = *msg;
    //https://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
    //ChatGPTでは-1(未知)、0(空き)、100(障害物)を示す

    resolution_ = map_.info.resolution;//単位格子当たりの大きさを格納
    width_ = map_.info.width;//マップの幅
    height_ = map_.info.height;//マップの高さ
    RCLCPP_INFO(this->get_logger(), "Grid size: %f meters per cell", resolution_);
}

// マップ全体の障害物を拡張処理（new_map_をpublishする）
void Astar::obs_expander()
{
    if (!map_received_) {
        RCLCPP_WARN(this->get_logger(), "Map not received yet, cannot expand obstacles.");
        return;
    }

    // 新しいマップデータ（拡張後）を作成
    new_map_ = map_;  // 元のマップをコピー
    std::vector<int8_t> new_map_data = map_.data;  // 1Dのデータをコピー

    margin_length = static_cast<int>(std::ceil(robot_radius_ / resolution_));  // セル単位の拡張範囲

    for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; x++) {
            index = y * width_ + x;
            if (map_grid_[y][x] == 100) {  // 障害物セルを拡張
                obs_expand(index, new_map_data);
            }
        }
    }

    // 新しいデータを `new_map_` に適用
    new_map_.data = new_map_data;

    // 拡張後のマップをPublish
    pub_new_map_->publish(new_map_);
    RCLCPP_INFO(this->get_logger(), "Published expanded obstacle map.");
}

// 指定されたインデックスの障害物を拡張（margin_length分）
void Astar::obs_expand(const int index,std::vector<int8_t> new_map_data)
{
    int x = index % width_;
    int y = index / width_;

    for (int dy = -margin_length; dy <= margin_length; dy++) {
        for (int dx = -margin_length; dx <= margin_length; dx++) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {  // 範囲チェック
                int n_index = ny * width_ + nx;
                if (new_map_data[n_index] != 100) {
                    new_map_data[n_index] = 50;  // 拡張エリア（50 = 半障害物）
                }
            }
        }
    }
}

// ヒューリスティック関数の計算
double Astar::make_heuristic(const Node_ node)
{

    int vertical = 0;//縦マス
    int side = 0;//横マス
    double heuristic = 0.0;//ヒューリスティック関数
    vertical = abs(node.y -goal_node_.y);//目標とゴールまでの差
    side = abs(node.x - goal_node_.x); //目標とゴールまでの差
    return heuristic = sqrt(vertical * vertical + side *side);
    //ピタゴラスの定理を用いて障害物除いた距離を算出

}

// スタートとゴールの取得（mからグリッド単位への変換も行う）
Node_ Astar::set_way_point(int phase)
{
    double grid_x,grid_y = 0.0;

    grid_x = node.x / resolution_;
    grid_y = node.y / resolution_;

    node.x = grid_x;
    node.y = grid_y;

    if(phase == 0){//スタート地点の格納にはphase=0
        start_node_ = node;//スタート地点格納
    }
    if(phase == 1){//ゴール地点の格納にはphase=1
        goal_node_ = node;//目標地点格納
    }

}

// ノードをたどり，waypoint間のパスを作成．その後グローバルパスに追加
// 参考：push_back(...) https://cpprefjp.github.io/reference/vector/vector/push_back.html
void Astar::create_path(Node_ node)
{
    nav_msgs::msg::Path partial_path;
    partial_path.poses.push_back(node_to_pose(node));

    // ###### パスの作成 ######


    // ###### パスの追加 ######

}

// ノード座標（グリッド）をgeometry_msgs::msg::PoseStamped（m単位のワールド座標系）に変換
geometry_msgs::msg::PoseStamped Astar::node_to_pose(const Node_ node)
{
    double world_x = 0.0,world_y = 0.0;//ワールド座標系格納用
    world_x = node.x * resolution_;//m単位に変換
    world_y = node.y * resolution_;//m単位に変換

    // PoseStampedの作成
    geometry_msgs::msg::PoseStamped pose_stamped;
    
    // ヘッダー情報（ROSのタイムスタンプやフレームIDが必要な場合は適宜設定）
    pose_stamped.header.frame_id = "map"; // マップ座標系（適宜変更）
    pose_stamped.header.stamp = rclcpp::Clock().now(); // 現在の時間を設定
    
    // 位置情報の設定
    pose_stamped.pose.position.x = world_x;
    pose_stamped.pose.position.y = world_y;
    pose_stamped.pose.position.z = 0.0; // 2DなのでZは0

    // 姿勢（オリエンテーション）はとりあえず無回転のまま（単位クォータニオンを設定）
    pose_stamped.pose.orientation.w = 1.0;
    pose_stamped.pose.orientation.x = 0.0;
    pose_stamped.pose.orientation.y = 0.0;
    pose_stamped.pose.orientation.z = 0.0;

    return pose_stamped;
}


// openリスト内で最もf値が小さいノードを取得する関数
Node_ Astar::select_min_f()
{
    double smallest_f = 0.0;//整数でも対応できるように変数設定　autoは？
    smallest_f = std::min_element(open_list_.begin(),open_list_.end(), [](const Node_& a, const Node_& b){
        return a.f < b.f;
    });//min_elementは比較によって最小値を見つけ出す関数
}

// スタートノードの場合，trueを返す
bool Astar::check_start(const Node_ node)
{
    return node = start_node_;
}

// ゴールノードの場合，trueを返す
bool Astar::check_goal(const Node_ node)
{
    return node = goal_node_;
}

// 2つが同じノードである場合，trueを返す
bool Astar::check_same_node(const Node_ n1, const Node_ n2)
{
    // if(n1.x == n2.x && n1.y == n2.y){
    //     return true;//x座標とy座標が同じであればtrue
    // }else{
    //     return false;
    // }

    return n1 = n2;
}

// 指定したリストに指定のノードが含まれるか検索
//（含まれる場合はインデックス番号を返し，含まれない場合は-1を返す）
int Astar::check_list(const Node_ target_node, std::vector<Node_>& set)
{
    auto result = std::find_if(list.begin(),list.end(), [](const Node_& n){
        return check_same_node(n,node);//同じノードを探す
        //autoで大丈夫？それからインデックス番号って？
    });

    if(result == list.end()){
        return -1;
    }
    //下にも同じような関数が一つあるけどどう違うの？
}

// list1から指定されたノードを探し，リスト1から削除してリスト2に移動する関数
void Astar::swap_node(const Node_ node, std::vector<Node_>& list1, std::vector<Node_>& list2)
{
    auto result = search_node_from_list(node,list1);//autoだけ心配

    list2.push_back(*result);//見つけたノードをlist2の末尾に追加
    list1.erase(result);//list2から指定のノードを削除   

}

// 指定のノードが障害物である場合，trueを返す
bool Astar::check_obs(const Node_ node)
{
    // (x, y) の座標を 1次元インデックスに変換
    index = node.y * width + node.x;

    // インデックスが範囲外でないかチェック
    if (index < 0 || index >= static_cast<int>(new_map_.data.size())) {
        return true;  // 範囲外なら障害物とみなす
    }

    // OccupancyGridのdata配列から値を取得
    int8_t cell_value = new_map_.data[index];//int8_tは符号付整数型

    // 100（障害物）なら true を返す
    return (cell_value == 100);

}

// 隣接ノードを基にOpenリスト・Closeリストを更新
// 隣接ノードを計算し，障害物を避けつつ，リスト内のノードを適切に追加・更新
// 複数の変数への代入はstd::tie(...)を使用すると便利 https://minus9d.hatenablog.com/entry/2015/05/24/133253
void Astar::update_list(const Node_ node)
{
    // 隣接ノードを宣言
    std::vector<Node_> neighbor_nodes;

    // ###### 隣接ノード ######


    
}

// 現在のノードを基に隣接ノードを作成
void Astar::create_neighbor_nodes(const Node_ node, std::vector<Node_>&  neighbor_nodes)
{
    // 動作モデルの作成
    std::vector<Motion_> motion_list;

    // ###### 動作モデルの作成 ######

    // ###### 隣接ノードの作成 ######

}

// 動作モデルを作成（前後左右，斜めの8方向）
void Astar::get_motion(std::vector<Motion_>& list)
{
    list.push_back(motion( 1, 0, 1.0));// 前
    list.push_back(motion(-1, 0, 1.0));// 後
    list.push_back(motion( 0,-1, 1.0));// 左 
    list.push_back(motion( 0, 1, 1.0));// 右
    list.push_back(motion( 1, 1, 1.414));// 右前
    list.push_back(motion(-1, 1, 1.414));// 右後
    list.push_back(motion( 1,-1, 1.414));// 左前
    list.push_back(motion(-1,-1, 1.414));// 左後
}

// 与えられたdx, dy, costを基にモーション（移動）を作成
// 隣接したグリッドに移動しない場合はエラーメッセージを出力して終了
Motion_ Astar::motion(const int dx,const int dy,const int cost)
{
    dx = abs(goal_node_.x - start_node_.y);//スタートからゴールまでのx座標移動距離
    dy = abs(goal_node_.y - start_node_.y);//スタートからゴールまでのy座標移動距離

    //斜め移動を考慮しなくてよいのか
    // int difference = 0;//差分を設定
    // difference = abs(dx - dy);//差を求めることによって斜め移動ができるか確認
    
    cost = dx + dy;//単純に加算してコストを求める

}

// 現在のノードと与えられたモーションを基に隣接ノードを計算し，その隣接ノードのf値と親ノードを更新して返す
Node_ Astar::get_neighbor_node(const Node_ node, const Motion_ motion)
{

}

// 指定されたノードがOpenリストまたはCloseリストに含まれているかを調べ，結果をインデックスとともに返す
// 1はOpenリスト，2はCloseリストにノードが含まれていることを示す
// -1はどちらのリストにもノードが含まれていないことを示す
std::tuple<int, int> Astar::search_node(const Node_ node)
{

}


// 親ノードかの確認し、親ノードであればtrue
bool Astar::check_parent(const int index, const Node_ node)
{
    return index = node.parent_y * width_ + node.parent_x;
    //インデックスが親ノードである場合
}



// 指定リスト内のノード検索
// 同一のノードが見つかればそのインデックスを返す
// 見つからなければ-1を返す
int Astar::search_node_from_list(const Node_ node, std::vector<Node_>& list)
{
    auto result = std::find_if(list.begin(),list.end(), [](const Node_& n){
        return check_same_node(n,node);//同じノードを探す　autoだけ心配
    });

    if(result == list.end()){
        return -1;
    }
}


// ［デバック用］指定されたノードの位置をRvizに表示
// test_show_がtrueの場合，ノードの座標をワールド座標系に変換し
// そのノードの情報をRvizにパブリッシュ
void Astar::show_node_point(const Node_ node)
{

}

// ［デバック用］指定されたパスをRvizに表示
// test_show_がtrueの場合，パスのフレームIDを"map"に設定し
// パス情報をRvizにパブリッシュ
void Astar::show_path(nav_msgs::msg::Path& current_path)
{

}

// 実行時間を表示（スタート時間beginを予め設定する）
void Astar::show_exe_time()
{
    RCLCPP_INFO_STREAM(get_logger(), "Duration = " << std::fixed << std::setprecision(2) << clock_.now().seconds() - begin_.seconds() << "s");
}



// 経路計画を行う関数
// 目的地までの経路をA*アルゴリズムを用いて計算し，グローバルパスを作成
// 各フェーズ（ウェイポイント間）について，OpenリストとCloseリストを操作しながら経路を探索
void Astar::planning()
{
    begin_ = clock_.now();
    const int total_phase = way_points_x_.size();

    // ###### ウェイポイント間の経路探索 ######


    show_exe_time();
    RCLCPP_INFO_STREAM(get_logger(), "COMPLITE ASTAR PROGLAM");
    exit(0);
}


// map_callback()関数で実行する関数
// A*アルゴリズムを実行する前にマップのロードをチェック
// マップが読み込まれた後に壁判定と経路計画を実行
void Astar::process()
{
    RCLCPP_INFO_STREAM(get_logger(), "process is starting...");

    if(!map_checker_){
    RCLCPP_INFO_STREAM(get_logger(), "NOW LOADING...");
    }else
    {
        RCLCPP_INFO_STREAM(get_logger(), "NOW LOADED MAP");
        obs_expander(); // 壁の拡張
        planning(); // グローバルパスの作成
    }

}
