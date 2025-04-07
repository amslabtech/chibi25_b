#include "global_path_planner/global_path_planner.hpp"

using namespace std::chrono_literals;

// デフォルトコンストラクタ
// パラメータの宣言と取得
Astar::Astar() : Node("teamb_path_planner"), clock_(RCL_ROS_TIME)
{
    // ###### パラメータの宣言 ######
    const robot_radius_ = this->declare_parameter<double>("robot_radius",0.3);
    pre_g_value_ = this->declare_parameter<double>("pre_g_value",0.0);

    const int phase = 10;
    //スタート地点を格納 数字はとりあえず適当
    double start_point_x_[phase] = {1.0,0.0,0.0,0.0,0.0};
    double start_point_y_[phase] = {1.0,0.0,0.0,0.0,0.0};
    //経由地点を配列として格納　数字はとりあえず適当
    double way_points_x_[phase] = {1.0,2.0,3.0,4.0,5.0};
    double way_points_y_[phase] = {1.0,2.0,3.0,4.0,5.0};

    // ###### パラメータの取得 ######


    // ###### global_path_とcurrent_node_のframe_id設定 ######
    global_path_.header.frame_id = "map";
    global_path_.header.stamp = rclcpp::Clock().now();

    for (auto &pose : global_path_.poses) {
        pose.header.frame_id = "map";
        pose.header.stamp = rclcpp::Clock().now();
    }

    current_node_.header.frame_id = "map";
    current_node_.header.stamp = rclcpp::Clock().now();

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
// 読み込みをすべてここで行うように修正
void Astar::map_callback(const nav_msgs::msg::OccupancyGrid msg)  //マップの読み込み
{
    map_ = *msg;
    //https://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
    //ChatGPTでは-1(未知)、0(空き)、100(障害物)を示す

    process();
    
    resolution_ = map_.info.resolution;//単位格子当たりの大きさを格納
    width_ = map_.info.width;//マップの幅
    height_ = map_.info.height;//マップの高さ
    origin_x_ = map_.info.origin_x//原点x座標
    origin_y_ = map_.info.origin_y//原点y座標

    RCLCPP_INFO(this->get_logger(), "Grid size: %f meters per cell", resolution_);
    RCLCPP_INFO(this->get_logger(), "Grid size: %f meters per cell", width_);
    RCLCPP_INFO(this->get_logger(), "Grid size: %f meters per cell", height_);

    // マップがからではなかったらtrueを返す
    if(!map_.empty()){
        map_checker_ = true;
    }
}

// マップ全体の障害物を拡張処理（new_map_をpublishする）
// 一次配列用の障害物拡張に変更
void Astar::obs_expander()
{
    if (!map_received_) {
        RCLCPP_WARN(this->get_logger(), "Map not received yet, cannot expand obstacles.");
        return;
    }

    // 新しいマップデータ（拡張後）を作成
    new_map_ = map_;  // 元のマップをコピー
    
    margin_length = static_cast<int>(std::ceil(robot_radius_ / resolution_));  // セル単位の拡張範囲

    // 一次配列だからindex分だけ回して障害物を探索する
    for(int i=0;i<index;i++){
        if(new_map_.data[i] == 100) {  // 障害物セルを拡張
            obs_expand(index, new_map_);
        }
    }
    // 拡張後のマップをPublish
    pub_new_map_->publish(new_map_);
    RCLCPP_INFO(this->get_logger(), "Published expanded obstacle map.");
}

// 指定されたインデックスの障害物を拡張（margin_length分）
// 既にほかの関数にある条件文を削除
void Astar::obs_expand(const int index,nav_msgs::msg::OccupancyGrid new_map_)
{
    int x = index % width_;
    int y = index / width_;

    for (int dy = -margin_length; dy <= margin_length; dy++) {
        for (int dx = -margin_length; dx <= margin_length; dx++) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {  // 範囲チェック
                int n_index = ny * width_ + nx;
                new_map_.data[n_index] = 100;//100まで拡張してしまう

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
Node_ Astar::set_way_point(int phase,int which)
{
    // スタート地点
    if(which == 0){
        if(phase != 0){
            start_point_x_[phase] = way_points_x_[phase-1];
            start_point_y_[phase] = way_points_y_[phase-1];
        }
        // スタートノードに格納
        start_node_.x = round((start_point_x_[phase] - origin_x_) / resolution_);
        start_node_.y = round((start_point_y_[phase] - origin_y_) / resolution_);
        return start_node_;
    }
    // ゴール地点
    if(which == 1){
        //経由地・ゴールを格納するノードを作成
        goal_node_.x = round((way_points_x_[phase] - origin_x_) / resolution_);
        goal_node_.y = round((way_points_y_[phase] - origin_y_) / resolution_);
        return goal_node_;
    }
}

// ノードをたどり，waypoint間のパスを作成．その後グローバルパスに追加
// 参考：push_back(...) https://cpprefjp.github.io/reference/vector/vector/push_back.html
void Astar::create_path(Node_ node)
{
    nav_msgs::msg::Path partial_path;
    partial_path.poses.push_back(node_to_pose(node));

    // ###### パスの作成 ######
    while(!check_same_node(node,start_node_)) {  // 親が存在する限り遡る
        node = node->parent; // 親ノードを取得
        partial_path.poses.push_back(node_to_pose(*node));
    }

    // 逆順になっているので順番を反転
    std::reverse(partial_path.poses.begin(), partial_path.poses.end());

    // ###### パスの追加 ######
    global_path_.poses.insert(global_path_.poses.end(), partial_path.poses.begin(), partial_path.poses.end());
}


// ノード座標（グリッド）をgeometry_msgs::msg::PoseStamped（m単位のワールド座標系）に変換
// コードを簡略化
geometry_msgs::msg::PoseStamped Astar::node_to_pose(const Node_ node)
{
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = node.x * resolution_ + origin_x_;
    pose_stamped.pose.position.y = node.y * resolution_ + origin_y_;

    return pose_stamped;
}


// openリスト内で最もf値が小さいノードを取得する関数
// 搭載されてる関数を使用しない方向に変更
Node_ Astar::select_min_f()
{
    Node_ tmp_;
    tmp_ = open_list_[0];
    for(size_t i=0; i<open_list_.size();i++){
        if(tmp_.f > open_list_[i].f){
            tmp_ = open_list_[i];
        }
    }
    return tmp_;
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
//（含まれる場合はリストの何番目にそのノードが含まれているか調べる，含まれない場合は-1を返す）
int Astar::check_list(const Node_ node, std::vector<Node_>& list)
{
    // search_node_from_listと同じ
    int check = -1;
    int number = 0;
    for(size_t i=0; i<list.size();i++){
        if(check_same_node(node,list)){
            // return node.y * width + node.x;
            check = 0;
            return number = i;
        }
    }
    if(check == -1){
        return check;
    }
}

// list1から指定されたノードを探し，リスト1から削除してリスト2に移動する関数
void Astar::swap_node(const Node_ node, std::vector<Node_>& list1, std::vector<Node_>& list2)
{
    int result = search_node_from_list(node,list1);

    list2.push_back(node);//見つけたノードをlist2の末尾に追加
    list1.erase(list1.begin() + result);//list2から指定のノードを削除   

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
    return (cell_value == 100);

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
        if(check_obs(neighbor_nodes)){
            continue;
        }
        std::tie = search_node(); 
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
        //ここにpub_node_point？？
        pre_g_value_ = node.g;
    } else {
        throw std::runtime_error("Error: open_list_ is empty, no next node available.");
    }
}

// 現在のノードを基に隣接ノードを作成
void Astar::create_neighbor_nodes(const Node_ node, std::vector<Node_>&  neighbor_nodes)
{
    // 動作モデルの作成
    std::vector<Motion_> motion_list;

    // ###### 動作モデルの作成 ######
    get_motion(motion_list);

    // ###### 隣接ノードの作成 ######
    for (const auto& motion : motion_list) {
        Node_ neighbor = get_neighbor_node(node,motion);
        neighbor.h = make_heuristic(node);
        neighbor.g += pre_g_value_;
        neighbor.f = neighbor.g + neighbor.h;
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
    neighbor.g = motion.cost;
    return neighbor;
}

// 指定されたノードがOpenリストまたはCloseリストに含まれているかを調べ，結果をインデックスとともに返す
// 1はOpenリスト，2はCloseリストにノードが含まれていることを示す
// -1はどちらのリストにもノードが含まれていないことを示す
std::tuple<int, int> Astar::search_node(const Node_ node)
{
    // index用
    int open = 0;
    int close = 0;
    // 1はOpenリスト，2はCloseリスト
    int which = 0;
    // index格納
    open = search_node_from_list(node,open_list_);
    close = search_node_from_list(node,close_list__);
    // どっちのリストに入っているか 
    if(open == -1){
        which = 2;
        return std::tuple(which,close);
    }
    if(close == -1){
        which = 1;
        return std::tuple(which,open);
    }else{
        return std::tuple(open,close);
    }

}


// 親ノードかの確認し、親ノードであればtrue
bool Astar::check_parent(const int index, const Node_ node)
{
    return (index == node.parent_y * width_ + node.parent_x);
    //インデックスが親ノードである場合
}



// 指定リスト内のノード検索
// 同一のノードが見つかればそのインデックスを返す
// 見つからなければ-1を返す
int Astar::search_node_from_list(const Node_ node, std::vector<Node_>& list)
{
    int check = -1;
    for(size_t i=0; i<list.size();i++){
        if(check_same_node(node,list)){
            return node.y * width + node.x;
            check = 0;
        }
    }
    if(check == -1){
        return check;
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
    //とりあえずphase1のみ、後ほど条件文
    set_way_point(0);
    
    start_node_.cost = 0;
    open_list_.push(start_node_);

    nav_msgs::msg::Path partial_path;
    nav_msgs::msg::Path perfect_path;
    current_ = open_list_.top();

    while (!open_list_.empty()) {
        open_list_.pop();

        // ゴールに到達した場合、経路を作成
        if (current_.x == goal_node_.x && current_.y == goal_node_.y) {
            create_path(current_,partial_path);
            //ここはperfect_pathに蓄積させるようなコードに変更
            pub_current_path_ -> publish(partial_path);
            break;
        }

        // 探索済みリストに追加
        close_list_.insert(current);

        update_list(current_);

    }
    //とりあえず
    if(current_.x ==  && current_.y == ){
        pub_path_ -> publish(partial_path);
    }

    for(int i=0; i<phase; i++){
        // リストを初期化
        open_list_.clear();
        close_list_.clear();
        // スタートとゴールのノード格納する
        start_node_ = set_way_point(i,0);
        goal_node_ = set_way_point(i,1);
        
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
