#include "global_path_planner/global_path_planner.hpp"

using namespace std::chrono_literals;

// デフォルトコンストラクタ
// パラメータの宣言と取得
Astar::Astar() : Node("teamb_global_path_planner"), clock_(RCL_ROS_TIME)
{
    // ###### パラメータの宣言 ######
    robot_radius_ = this->declare_parameter<double>("robot_radius",0.3);

    const int phase = 6;
    test_show_ = true;

    //スタート地点を格納 数字はとりあえず適当
    

    int select_limit = 1;

    //経由地点を配列として格納　数字はとりあえず適当
    way_points_x_.reserve(50);
    way_points_y_.reserve(50);
    way_points_x_ = {-14.0,-14.0,2.0,19.0,19.0,2.0};
    way_points_y_ = {1.5,-12.5,-12.5,-12.5,1.5,1.5};

    open_list_.reserve(50);
    close_list_.reserve(50);

    // ###### パラメータの取得 ######
start_point_x_ = 2.0;
    start_point_y_ = 1.5;

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
void Astar::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)  //マップの読み込み
{
    map_ = *msg;
    //https://docs.ros.org/en/lunar/api/nav_msgs/html/msg/OccupancyGrid.html
    //ChatGPTでは-1(未知)、0(空き)、100(障害物)を示す
    
    // マップがからではなかったらtrueを返す
    if(!map_.data.empty()){
        map_checker_ = true;
    }

    resolution_ = map_.info.resolution;//単位格子当たりの大きさを格納
    width_ = map_.info.width;//マップの幅
    height_ = map_.info.height;//マップの高さ
    origin_x_ = map_.info.origin.position.x;//原点x座標
    origin_y_ = map_.info.origin.position.y;//原点y座標

    RCLCPP_INFO(this->get_logger(), "Grid size: %f meters per cell", resolution_);
    RCLCPP_INFO(this->get_logger(), "Grid size: %d meters per cell", width_);
    RCLCPP_INFO(this->get_logger(), "Grid size: %d meters per cell", height_);

    process();
}

// マップ全体の障害物を拡張処理（new_map_をpublishする）
// 一次配列用の障害物拡張に変更
void Astar::obs_expander()
{
    // 新しいマップデータ（拡張後）を作成
    new_map_ = map_;  // 元のマップをコピー
    
    margin_length = static_cast<int>(std::ceil(robot_radius_ / resolution_));  // セル単位の拡張範囲

    // 一次配列だからindex分だけ回して障害物を探索する
    for(int i=0;i<new_map_.data.size();i++){
        if(new_map_.data[i] == 100) {  // 障害物セルを拡張
            obs_expand(i, new_map_);
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
    RCLCPP_INFO(this->get_logger(), "heuristic");
    double vertical = 0;//縦マス
    double side = 0;//横マス
    vertical = abs(node.y -goal_node_.y);//目標とゴールまでの差
    side = abs(node.x - goal_node_.x); //目標とゴールまでの差

    // RCLCPP_INFO(this->get_logger(), "node = %d,goal = %d",node.x,goal_node_.x);
    // RCLCPP_INFO(this->get_logger(), "node = %d,goal = %d",node.y,goal_node_.y);
    return sqrt(vertical * vertical + side *side);
    //ピタゴラスの定理を用いて障害物除いた距離を算出
    

}

// スタートとゴールの取得（mからグリッド単位への変換も行う）
Node_ Astar::set_way_point(const int phase,int which)
{
    RCLCPP_INFO(this->get_logger(), "Phase: %d, Waypoint X: %f, Waypoint Y: %f", phase, way_points_x_[phase], way_points_y_[phase]);

    // RCLCPP_INFO(this->get_logger(), "setway_point");
    // スタート地点
    if (phase < 0 || phase >= way_points_x_.size()) {
    RCLCPP_ERROR(this->get_logger(), "Phase out of bounds!");
    return Node_();  // エラーハンドリング
}

    if(which == 0){
            if (phase != 0) {
            
            start_point_x_ = way_points_x_[phase - 1];
            start_point_y_ = way_points_y_[phase - 1];
        }
        RCLCPP_INFO(this->get_logger(), "start %f",start_point_x_);
        
        // スタートノードに格納
        start_node_.x = round((start_point_x_ - origin_x_) / resolution_);
        start_node_.y = round((start_point_y_ - origin_y_) / resolution_);
        RCLCPP_INFO(this->get_logger(), "start node %f",start_node_.x);
        // RCLCPP_INFO(this->get_logger(), "1");
        return start_node_;
    }
    // ゴール地点
    if(which == 1){
        
        
        //経由地・ゴールを格納するノードを作成
        goal_node_.x = round((way_points_x_[phase] - origin_x_) / resolution_);
        RCLCPP_INFO(this->get_logger(),"goal= %f -%f / %f",way_points_x_[phase],origin_x_,resolution_);
        
        goal_node_.y = round((way_points_y_[phase] - origin_y_) / resolution_);
        RCLCPP_INFO(this->get_logger(),"goal %d,%d",goal_node_.x,goal_node_.y);
        return goal_node_;
    }
    else
    return Node_();
    
}

// ノードをたどり，waypoint間のパスを作成．その後グローバルパスに追加
// 参考：push_back(...) https://cpprefjp.github.io/reference/vector/vector/push_back.html
void Astar::create_path(Node_ node)
{
    nav_msgs::msg::Path partial_path;
    RCLCPP_INFO(this->get_logger(), "create_path");
    // ###### パスの作成 ######

    partial_path.poses.push_back(node_to_pose(node));
    for(int i=close_list_.size()-1; i>=0; i--){
        if(!check_same_node(close_list_[i],start_node_)){
            // 特定のノードを特定のリストから探し出す関数で親ノードをたどりスタートまで格納し続ける
            if(check_parent(i-1,close_list_[i])){
                partial_path.poses.push_back(node_to_pose(close_list_[i-1]));
            }
            // デバック用
            show_path(partial_path);
        }
    }

    std::reverse(partial_path.poses.begin(),partial_path.poses.end());
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
    if (open_list_.empty()) {
    RCLCPP_WARN(this->get_logger(), "open_list_ is empty! Returning default Node_");
    return Node_();  // あるいは nullptr とか適切なエラー処理
}


    Node_ tmp_;
    tmp_ = open_list_[0];
    // RCLCPP_INFO(this->get_logger(), "open_list_size = %zu",open_list_.size());
    // RCLCPP_INFO(this->get_logger(), "openlist_size %zu",open_list_.size());
    // select_limit++;
    RCLCPP_INFO(this->get_logger(), "select_limit = %d",select_limit);

        for(std::size_t i=1; i<open_list_.size();i++){
            // RCLCPP_INFO(this->get_logger(), "tmp = %f",tmp_.f);
            if(tmp_.f > open_list_[i].f){
                tmp_ = open_list_[i];
            }
            
            
        }
    
    
    // RCLCPP_INFO(this->get_logger(), "select_min");
    // RCLCPP_INFO(this->get_logger(), "current = %d",tmp_.x);
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
    for(int i=0; i<list.size();i++){
        if(check_same_node(node,list[i])){
            // return node.y * width + node.x;
            check = 0;
            RCLCPP_WARN(this->get_logger(),"same!!");
            return number = i;
        }
    }
    if(check == -1){
        return number;
    }
    else 
    
    return 0;
}

// list1から指定されたノードを探し，リスト1から削除してリスト2に移動する関数
void Astar::swap_node(const Node_ node, std::vector<Node_>& list1, std::vector<Node_>& list2)
{
    int result = search_node_from_list(node,list1);
    
    if (result == -1) {
        RCLCPP_WARN(this->get_logger(), "Node not found in list1, cannot swap.");
        return;
    }

    list2.push_back(list1[result]);//見つけたノードをlist2の末尾に追加
    RCLCPP_INFO(this->get_logger(),"swap!!");
    list1.erase(list1.begin() + result);//list2から指定のノードを削除   
    RCLCPP_INFO(this->get_logger(),"swap##");

    RCLCPP_INFO(this->get_logger(), "list1 size before: %zu", list1.size());
RCLCPP_INFO(this->get_logger(), "list2 size before: %zu", list2.size());
RCLCPP_INFO(this->get_logger(), "swap target index = %d", result);
RCLCPP_INFO(this->get_logger(), "current_node.x = %d", node.x);


}

// 指定のノードが障害物である場合，trueを返す
bool Astar::check_obs(const Node_ node)
{
    // (x, y) の座標を 1次元インデックスに変換
    int index = node.y * width_ + node.x;
    RCLCPP_INFO(this->get_logger(),"index %d",index);
    RCLCPP_INFO(this->get_logger(),"map_size %zu",new_map_.data.size());
    RCLCPP_INFO(this->get_logger(),"current %d",node.x);


    // インデックスが範囲外でないかチェック
    if (index < 0 || index >= new_map_.data.size()) {
        return true;  // 範囲外なら障害物とみなす
        
    }

    // OccupancyGridのdata配列から値を取得
    int cell_value = new_map_.data[index];//int8_tは符号付整数型
    RCLCPP_INFO(this->get_logger(),"obsta %d",cell_value);
    
    // 100（障害物）なら true を返す
    return (cell_value == 100);

}

// 隣接ノードを基にOpenリスト・Closeリストを更新
// 隣接ノードを計算し，障害物を避けつつ，リスト内のノードを適切に追加・更新
// 複数の変数への代入はstd::tie(...)を使用すると便利 https://minus9d.hatenablog.com/entry/2015/05/24/133253
void Astar::update_list(Node_ node)
{
    
    RCLCPP_INFO(this->get_logger(), "update_list");
    // 隣接ノードを宣言
    std::vector<Node_> neighbor_nodes;
    neighbor_nodes.reserve(100000);
    
    create_neighbor_nodes(node,neighbor_nodes);
    //隣接ノードを用意する
    // ###### 隣接ノード ######

    for (auto& neighbor : neighbor_nodes){
        
        if(check_obs(neighbor)){
            RCLCPP_INFO(this->get_logger(), "check");
            continue;
            
        }
        int which = 0;
        int index = 0;
        std::tie(which,index) = search_node(neighbor);//ここがおかしい
        // RCLCPP_INFO(this->get_logger(),"neighbor front =%f , %f",neighbor.h,neighbor.f);
        // どっちもない
        if(which == -1){
            open_list_.push_back(neighbor);
            // RCLCPP_INFO(this->get_logger(), "Forth#open_list_size");
        select_limit++;
        }
        // openにある
        if(which == 1){
            // RCLCPP_INFO(this->get_logger(), "8");
            if(neighbor.f < open_list_[index].f){
                open_list_[index].f = neighbor.f;
                open_list_[index].parent_x = neighbor.parent_x;
                open_list_[index].parent_y = neighbor.parent_y;
            }
        }
        // closeにある
        if(which == 2){
            // RCLCPP_INFO(this->get_logger(), "9");
            if(neighbor.f < close_list_[index].f){
                close_list_.erase(close_list_.begin() + index);
                open_list_.push_back(neighbor);
                // RCLCPP_INFO(this->get_logger(), "fifth#open_list_size = %zu",open_list_.size());
                select_limit++;
                

            }
        }
    }
}

// 現在のノードを基に隣接ノードを作成
void Astar::create_neighbor_nodes(const Node_ node, std::vector<Node_>&  neighbor_nodes)
{
    // 動作モデルの作成
    std::vector<Motion_> motion_list;
    motion_list.reserve(50);

    // ###### 動作モデルの作成 ######
    get_motion(motion_list);

    // ###### 隣接ノードの作成 ######
    for (const auto& motion : motion_list) {
        Node_ neighbor = get_neighbor_node(node,motion);
        neighbor_nodes.push_back(neighbor);
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
Motion_ Astar::motion(const int dx,const int dy,const double cost)
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
    neighbor.g = node.g + motion.cost;
    neighbor.h = make_heuristic(neighbor);
    neighbor.f = neighbor.g + neighbor.h;
    // RCLCPP_INFO(this->get_logger(),"neighbor =%f , %f",neighbor.h,neighbor.f);
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
    int which = -1;
    // index格納
    open = search_node_from_list(node,open_list_);
    close = search_node_from_list(node,close_list_);
    // どっちのリストに入っているか 
    if(open != -1){
        // openにある
        which = 1;
        return std::tuple(which,open);
    }
    if(close != -1){
        // closeにある
        which = 2;
        return std::tuple(which,close);
    }else{
        return std::tuple(which,close);
    }

}


// 親ノードかの確認し、親ノードであればtrue
bool Astar::check_parent(const int index, const Node_ node)
{
    return (close_list_[index].x == node.parent_x && close_list_[index].y == node.parent_y);
    //インデックスが親ノードである場合
}



// 指定リスト内のノード検索
// 同一のノードが見つかればそのインデックスを返す
// リストの初めから何番目かをindexで返す
// 見つからなければ-1を返す
int Astar::search_node_from_list(const Node_ node, std::vector<Node_>& list)
{
    int check = -1;
    int number = 0;
    for(int i=0; i<list.size();i++){
        if(check_same_node(node,list[i])){
        RCLCPP_WARN(this->get_logger(),"same!!");    
        return  i;
        }
    }
    if(check == -1){
        RCLCPP_WARN(this->get_logger(),"ok!!");    
        return check;
    }
    
}


// ［デバック用］指定されたノードの位置をRvizに表示
// test_show_がtrueの場合，ノードの座標をワールド座標系に変換し
// そのノードの情報をRvizにパブリッシュ
void Astar::show_node_point(const Node_ node)
{
    RCLCPP_INFO(this->get_logger(), "show_node");
    if(test_show_)
    {
        // 座標変換
        current_node_.point.x = node.x * resolution_ + origin_x_;
        current_node_.point.y = node.y * resolution_ + origin_y_;
        
        // パブリッシュ
        pub_node_point_->publish(current_node_);
        rclcpp::sleep_for(std::chrono::nanoseconds(
        static_cast<int>(sleep_time_ * 1e9)));
     
    }
}


// ［デバック用］指定されたパスをRvizに表示
// test_show_がtrueの場合，パスのフレームIDを"map"に設定し
// パス情報をRvizにパブリッシュ
void Astar::show_path(nav_msgs::msg::Path& current_path)
{
    if(test_show_)
    {
        // パスのframe設定
        current_path.header.frame_id = "map";
        pub_current_path_->publish(current_path);
        rclcpp::sleep_for(std::chrono::nanoseconds(
        static_cast<int>(sleep_time_ * 1e9)));
    }

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
    // const int total_phase = way_points_x_.size();
    // ###### ウェイポイント間の経路探索 ######

    for(int i=0; i<phase; i++){
        // リストを初期化
        open_list_.clear();
        close_list_.clear();
        // RCLCPP_INFO(this->get_logger(), "first#open_list_size = %zu",open_list_.size());
        // スタートとゴールのノード格納する
        start_node_ = set_way_point(i,0);
        goal_node_ = set_way_point(i,1);
        // RCLCPP_INFO(this->get_logger(), "hooo!!");
        // RCLCPP_INFO(this->get_logger(),"goal %d,%d",goal_node_.x,goal_node_.y);
        open_list_.push_back(start_node_);
       
        select_limit++;
        // RCLCPP_INFO(this->get_logger(), "yeah!!");
        int count = 0;
        while(rclcpp::ok()){
            if(count == 0){
                // RCLCPP_INFO(this->get_logger(), "count%d",count);
                count++;
                current_ = start_node_;

                update_list(current_);
                // RCLCPP_INFO(this->get_logger(), "second#open_list_size = %zu",open_list_.size());
                swap_node(current_,open_list_,close_list_);
                
                // RCLCPP_INFO(this->get_logger(), "continue...");
                // RCLCPP_INFO(this->get_logger(), "third#open_list_size = %zu",open_list_.size());
                continue;
            }
            // RCLCPP_INFO(this->get_logger(), "select");
            current_ = select_min_f();//ここでcurrentが消えてる
            // RCLCPP_INFO(this->get_logger(), "current.x size: %d meters per cell", current_.x);
            // RCLCPP_INFO(this->get_logger(), "current.y size: %d meters per cell", current_.y);

            show_node_point(current_);
            if(check_same_node(current_,goal_node_)){
                create_path(current_);
                RCLCPP_INFO(this->get_logger(), "planning_fin");
                pub_current_path_ -> publish(global_path_);
                RCLCPP_INFO(this->get_logger(), "done");
                break;
            }
            if(!check_same_node(current_,goal_node_)){
                // RCLCPP_INFO(this->get_logger(), "current.x size: %d meters per cell", current_.x);
                // RCLCPP_INFO(this->get_logger(), "current.y size: %d meters per cell", current_.y);
                update_list(current_);
                // RCLCPP_INFO(this->get_logger(), "9#open_list_size = %zu",open_list_.size());
                swap_node(current_,open_list_,close_list_);
                // RCLCPP_INFO(this->get_logger(), "six#open_list_size = %zu",open_list_.size());
                
            }
        }
        RCLCPP_INFO(this->get_logger(), "planning");

        pub_path_ -> publish(global_path_);
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