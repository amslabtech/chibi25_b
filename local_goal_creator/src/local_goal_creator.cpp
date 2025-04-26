#include "local_goal_creator/local_goal_creator.hpp"
LocalGoalCreator::LocalGoalCreator() : Node("LocalGoalCreator")
{
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/e_pose", rclcpp::QoS(1).reliable(),std::bind(&LocalGoalCreator::poseCallback,this,std::placeholders::_1));
    //estimated poseの購読
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/g_path", rclcpp::QoS(1).reliable(),std::bind(&LocalGoalCreator::pathCallback,this,std::placeholders::_1));
    //global pathの購読
    //pubやsubの定義，tfの統合
    local_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/local_goal",rclcpp::QoS(1).reliable());//#########
    //初期値の設定
    hz_= this->declare_parameter<int>("hz", 10);// ループ周期 [Hz]
    goal_index_ = this->declare_parameter<int>("goal", 50);// グローバルパス内におけるローカルゴールのインデックス
    index_step_ = this->declare_parameter<int>("index", 5); // １回で更新するインデックス数
    target_distance_ = this->declare_parameter<float>("distance", 0.5);// 現在位置-ゴール間の距離 [m]
    //初期設定

        goal_.header.frame_id = "map";
}

void LocalGoalCreator::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)//subのコールバック関数
{
    pose_ =*msg;
}

void LocalGoalCreator::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)//subのコールバック関数
{
    path_=*msg;
}

int LocalGoalCreator::getOdomFreq()//hzを返す関数（無くてもいい）
{
    return hz_;
}

void LocalGoalCreator::process()//main文ので実行する関数
{
    //pathが読み込めた場合にpublishGoal関数を実行
    if (!path_.poses.empty()) {
        publishGoal();
    }


}


void LocalGoalCreator::publishGoal()
{
    if (goal_index_ >= static_cast<int>(path_.poses.size())) {
        return; // すでに最後のゴールなら更新しない
    }

    // ゴールまでの距離を取得
    target_distance_ = getDistance();
    double threshold_distance_ = 2.7;

    // ゴールに近づいたら次のゴールを選択
    if (target_distance_ < threshold_distance_) {//if文でもいい（重い）
        goal_index_ += index_step_;

        // インデックスがパスの範囲を超えたら最後のゴールに留まる
        if (goal_index_ >= static_cast<int>(path_.poses.size())) {
            goal_index_ = static_cast<int>(path_.poses.size()) - 1;
            // break;
        }
    }

    if (goal_index_ < static_cast<int>(path_.poses.size())) {
        printf("publish_goal!!\n");

        goal_.point = path_.poses[goal_index_].pose.position;
        goal_.header.stamp = this->now();
        local_goal_pub_->publish(goal_);
    }
}


double LocalGoalCreator::getDistance()
{
    //if (path_.poses.empty()) return std::numeric_limits<double>::max(); // パスが空なら最大値を返す

    double dx = path_.poses[goal_index_].pose.position.x - pose_.pose.position.x;
    double dy = path_.poses[goal_index_].pose.position.y - pose_.pose.position.y;
    return sqrt(dx * dx + dy * dy);
}
