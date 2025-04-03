#include "global_path_planner/global_path_planner.hpp"

using namespace std::chrono_literals;

// デフォルトコンストラクタ
// パラメータの宣言と取得
Astar::Astar() : Node("teamb_path_planner"), clock_(RCL_ROS_TIME)
{
    // ###### パラメータの宣言 ######
    robot_radius_ = this->declare_parameter<double>("robot_radius",0.3);
    // 経由地（waypoints）をパラメータとして宣言
    std::vector<std::vector<double>> waypoints = this->declare_parameter<std::vector<std::vector<double>>>(
        "waypoints",
        {{}, {}, {}} // デフォルトの経由地
    );

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
    
    // マップの原点を取得//ここ違う
    current_node_.point.x = map_.info.origin.position.x;
    current_node_.point.y = map_.info.origin.position.y;
    current_node_.point.z = 0.0;
    
    
    RCLCPP_INFO(this->get_logger(), "Grid size: %f meters per cell", resolution_);
    RCLCPP_INFO(this->get_logger(), "Grid size: %f meters per cell", width_);
    RCLCPP_INFO(this->get_logger(), "Grid size: %f meters per cell", height_);

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
    int y = index / width_ + 1;

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
    if(phase == 0){//スタート地点の格納にはphase=0
        double grid_x = 0.0,grid_y = 0.0,grid_parent_x = 0.0,grid_parent_y = 0.0;

        grid_x = current_node_.point.x / resolution_;
        grid_y = current_node_.point.y / resolution_;
        grid_parent_x = current_node_.point.x / resolution_;
        grid_parent_y = current_node_.point.y / resolution_;

        //スタート地点格納
        start_node_.x = grid_x;
        start_node_.y = grid_y;
        start_node_.parent_x = grid_parent_x;
        start_node_.parent_y = grid_parent_y;
        // 毎回更新していくのか区間ごとに変化させていくのか？
        // なわけねえだろ後者や！
        const auto& wp : waypoints;
        Node_ waypoint(wp[0], wp[1]); // 1つの経由地を取得

        if (path.empty()) {
            RCLCPP_WARN(this->get_logger(), "経由地 (%f, %f) までの経路が見つかりませんでした。", wp[0], wp[1]);
            return;
        }

        // すでにある最後のノードを重複しないように除外
        if (!final_path.empty()) {
            path.erase(path.begin());
        }
        final_path.insert(final_path.end(), path.begin(), path.end());

        start = waypoint; // 次の経由地のために更新

        // waypointとしてgoal_node_を格納
    }
    // if(phase == 1){//最終ゴール地点の格納にはphase=1
        
    // }

}

// ノードをたどり，waypoint間のパスを作成．その後グローバルパスに追加
// 参考：push_back(...) https://cpprefjp.github.io/reference/vector/vector/push_back.html
void Astar::create_path(Node_ node)
{
    nav_msgs::msg::Path partial_path;
    partial_path.poses.push_back(node_to_pose(node));

    // ###### パスの作成 ######
    while (node->parent != nullptr) {  // 親が存在する限り遡る
        node = node->parent; // 親ノードを取得
        partial_path.poses.push_back(node_to_pose(*node));
    }

    // 逆順になっているので順番を反転
    std::reverse(partial_path.poses.begin(), partial_path.poses.end());

    // ###### パスの追加 ######
    global_path_.poses.insert(global_path_.poses.end(), partial_path.poses.begin(), partial_path.poses.end());
}


// ノード座標（グリッド）をgeometry_msgs::msg::PoseStamped（m単位のワールド座標系）に変換
geometry_msgs::msg::PoseStamped Astar::node_to_pose(const Node_ node)
{
    double world_x = 0.0,world_y = 0.0;//ワールド座標系格納用
    world_x = node.x * resolution_;//m単位に変換
    world_y = node.y * resolution_;//m単位に変換

    geometry_msgs::msg::PoseStamped world_node;
    //ワールド座標系のノードを作成
    
    // ヘッダー情報（ROSのタイムスタンプやフレームIDが必要な場合は適宜設定）
    world_node.header.frame_id = "map"; // マップ座標系（適宜変更）
    world_node.header.stamp = rclcpp::Clock().now(); // 現在の時間を設定
    
    // 位置情報の設定
    world_node.pose.position.x = world_x;
    world_node.pose.position.y = world_y;
    world_node.pose.position.z = 0.0; // 2DなのでZは0

    // 姿勢（オリエンテーション）はとりあえず無回転のまま（単位クォータニオンを設定）
    world_node.pose.orientation.w = 1.0;
    world_node.pose.orientation.x = 0.0;
    world_node.pose.orientation.y = 0.0;
    world_node.pose.orientation.z = 0.0;

    return world_node;
}


// openリスト内で最もf値が小さいノードを取得する関数
Node_ Astar::select_min_f()
{
    auto smallest_f = std::min_element(open_list_.begin(),open_list_.end(), [](const Node_& a, const Node_& b){
        return a.f < b.f;
    });//min_elementは比較によって最小値を見つけ出す関数
    return *smallest_f;
}

// スタートノードの場合，trueを返す
bool Astar::check_start(const Node_ node)
{
    return check_same_node(node,start_node_);
}

// ゴールノードの場合，trueを返す
bool Astar::check_goal(const Node_ node)
{
    return check_same_node(node,goal_node_);
}

// 2つが同じノードである場合，trueを返す
bool Astar::check_same_node(const Node_ n1, const Node_ n2)
{
    return (n1.x == n2.x && n1.y == n2.y);
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
    index = node.y * width_ + node.x;

    // インデックスが範囲外でないかチェック
    if (index < 0 || index >= static_cast<int>(new_map_.data.size())) {
        return true;  // 範囲外なら障害物とみなす
    }

    // OccupancyGridのdata配列から値を取得
    int8_t cell_value = new_map_.data[index];//int8_tは符号付整数型

    // 100（障害物）なら true を返す
    return (cell_value >= 50);

}

// 隣接ノードを基にOpenリスト・Closeリストを更新
// 隣接ノードを計算し，障害物を避けつつ，リスト内のノードを適切に追加・更新
// 複数の変数への代入はstd::tie(...)を使用すると便利 https://minus9d.hatenablog.com/entry/2015/05/24/133253
void Astar::update_list(Node_ node)
{
    // 隣接ノードを宣言
    std::vector<Node_> neighbor_nodes;
    create_neighbor_nodes(node,neighbor_nodes);
    //隣接ノードを用意する

    // ###### 隣接ノード ######

    for (auto& neighbor : neighbor_nodes){
        
        //close_list__の中身と同じものが入っていたらclose_list_へ
        // 障害物であったらclose_list_へ
        if (close_list_(neighbor)||check_obs(neighbor)){
            swap_node(neighbor,open_list_,close_list__);   
        }
        //open_listに格納
        open_list_.push(neighbor);
    }
    //現在のノードをclose_list_に移動
    swap_node(node,open_list_,close_list_);
    //open_listに格納したf値のなかで最も小さいものを選択
        // open_list_ 内の f値が最小のノードを取得し、新しい current_node_ に更新
    if (!open_list_.empty()) {
        node = select_min_f();
    } else {
        throw std::runtime_error("Error: open_list_ is empty, no next node available.");
    }
}

// 現在のノードを基に隣接ノードを作成
void Astar::create_neighbor_nodes(const Node_ node, std::vector<Node_>&  neighbor_nodes,double pre_g_value)
{
    // 動作モデルの作成
    std::vector<Motion_> motion_list;

    // ###### 動作モデルの作成 ######
    get_motion(motion_list);

    // ###### 隣接ノードの作成 ######
    for (const auto& motion : motion_list) {
        Node_ neighbor = get_neighbor_node(node,motion,pre_g_value);
        neighbor.h = make_heuristic(node);
        neighbor_nodes.push_back(neighbor);
        // if (is_valid_node(neighbor)) {
        //     neighbor_nodes.push_back(neighbor);
        // }
    }
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
    if (abs(dx) > 1 || abs(dy) > 1) {
        std::cerr << "Error: Invalid motion (" << dx << ", " << dy << ")" << std::endl;
        exit(EXIT_FAILURE);
    }//エラー文
    return Motion_{dx, dy, cost};
}

// 現在のノードと与えられたモーションを基に隣接ノードを計算し，その隣接ノードのf値と親ノードを更新して返す
Node_ Astar::get_neighbor_node(const Node_ node, const Motion_ motion)
{
    Node_ neighbor;
    neighbor.x = node.x + motion.dx;
    neighbor.y = node.y + motion.dy;
    neighbor.parent_x = node.x;
    neighbor.parent_y = node.y;
    neighbor.g += motion.cost;
    return neighbor;
}

// 指定されたノードがOpenリストまたはCloseリストに含まれているかを調べ，結果をインデックスとともに返す
// 1はOpenリスト，2はCloseリストにノードが含まれていることを示す
// -1はどちらのリストにもノードが含まれていないことを示す
std::tuple<int, int> Astar::search_node(const Node_ node)
{
    //Openリストの検索
    int index = 0;
    for (const auto& n : open_list_) {
        search_node_from_list(node,open_list_);
    }

    // Closeリストの検索
    index = 0;
    for (const auto& n : close_list__) {
        search_node_from_list(node,close_list__);
    }
        // どちらにも存在しない場合
    return std::make_tuple(-1, -1);
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
        if(check_same_node(n,node) == true){
            return node.y * width + node.x;
        }//同じノードを探す　autoだけ心配
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
    if(!test_show_) return;
    //ワールド座標系に変換
    geometry_msgs::msg::PointStamped node_check = node_to_pose(node);
    //座標返還後のノードを格納
    pub_node_point_ -> publish(node_check);
    //node_checkにパブリッシュ
}


// ［デバック用］指定されたパスをRvizに表示
// test_show_がtrueの場合，パスのフレームIDを"map"に設定し
// パス情報をRvizにパブリッシュ
void Astar::show_path(nav_msgs::msg::Path& current_path)
{
    if(!test_show_) return;
    current_path.header.frame_id = "map";
    current_path.header.stamp = show_exe_time();
    //経路情報がいつ作成されたものなのか記録
    path_pub_->publish(current_path);
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
    set_way_point(0);//とりあえずphase1のみ、後ほど条件文
    
    start_node_.cost = 0;
    open_list_.push(start_node_);

    while (!open_list_.empty()) {
        Node_ current = open_list_.top();
        open_list_.pop();

        // ゴールに到達した場合、経路を作成
        if (current.x == goal_node_.x && current.y == goal_node_.y) {
            create_path(current);
            break;
        }

        // 探索済みリストに追加
        close_list_.insert(current);

        update_list(current_node_);

        // // 隣接ノードを探索
        // std::vector<Node_> neighbors = create_neighbor_nodes(current);
        // for (auto& neighbor : neighbors) {
        //     if (close_list_.find(neighbor) != close_list_.end()) {
        //         continue; // すでに探索済みならスキップ
        //     }

        //     double new_cost = current.cost + calculate_cost(current, neighbor);
        //     if (new_cost < neighbor.cost || !search_node_from_list(neighbor, open_list_)) {
        //         neighbor.cost = new_cost;
        //         neighbor.parent = std::make_shared<Node_>(current);
        //         open_list_.push(neighbor);
        //     }
        // }
    }

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
