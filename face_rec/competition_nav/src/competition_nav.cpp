#include <ros/ros.h>
#include <robot_audio/robot_iat.h>
#include <robot_audio/Collect.h>
#include <robot_audio/robot_tts.h>
#include <actionlib/client/simple_action_client.h>
#include "move_base_msgs/MoveBaseAction.h"
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

// 导航点结构体定义
struct NavPoint {
    float x;        // x坐标
    float y;        // y坐标
    float z;        // 姿态z
    float w;        // 姿态w
    string name;    // 地点名字
    string intro;   // 介绍语
};

// 全局导航点数组
vector<NavPoint> nav_points = {
    {1.026, 1.109, -0.012, 1.000, "深圳馆", "深圳，中国的科创中心"},
    {1.073, 2.120, 0.013, 1.000, "上海馆", "上海，中国的经济中心"},
    {2.48, 0.13, 0.03, 0.99, "北京馆", "北京，中国的首都"},
    {2.48, 1.077, -0.026, 1.000, "广州馆", "广州，自古以来都是中国的商都"},
    {2.48, 2.120, -0.032, 0.999, "吉林馆", "吉林，位于中国的东北是人参之都"}
};

// 交互类定义
class CompetitionNav {
public:
    CompetitionNav();
    string voiceCollect();  // 语音采集
    string voiceDictation(const char* filename);  // 语音听写
    string voiceTTS(const char* text);  // 语音合成
    void navigateToPoint(const NavPoint& point);  // 导航到目标位置
    void tourAllPoints();  // 参观所有点
    
private:
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* ac;
    ros::ServiceClient collect_client, dictation_client, tts_client;
    void returnToStartPoint(); // 返回起点方法声明
};

// 构造函数
CompetitionNav::CompetitionNav() {
    collect_client = nh.serviceClient<robot_audio::Collect>("voice_collect");
    dictation_client = nh.serviceClient<robot_audio::robot_iat>("voice_iat");
    tts_client = nh.serviceClient<robot_audio::robot_tts>("voice_tts");
}

// 语音采集实现
string CompetitionNav::voiceCollect() {
    ros::service::waitForService("voice_collect");
    robot_audio::Collect srv;
    srv.request.collect_flag = 1;
    collect_client.call(srv);
    return srv.response.voice_filename;
}

// 语音听写实现
string CompetitionNav::voiceDictation(const char* filename) {
    ros::service::waitForService("voice_iat");
    robot_audio::robot_iat srv;
    srv.request.audiopath = filename;
    dictation_client.call(srv);
    return srv.response.text;
}

// 语音合成实现
string CompetitionNav::voiceTTS(const char* text) {
    ros::service::waitForService("voice_tts");
    robot_audio::robot_tts srv;
    srv.request.text = text;
    tts_client.call(srv);
    string cmd = "play " + srv.response.audiopath;
    system(cmd.c_str());
    sleep(1);
    return srv.response.audiopath;
}

// 导航到指定点实现
void CompetitionNav::navigateToPoint(const NavPoint& point) {
    ac = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);
    ROS_INFO("等待导航服务器启动...");
    ac->waitForServer();
    ROS_INFO("导航服务器已启动，发送目标点...");

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    
    goal.target_pose.pose.position.x = point.x;
    goal.target_pose.pose.position.y = point.y;
    goal.target_pose.pose.orientation.z = point.z;
    goal.target_pose.pose.orientation.w = point.w;
    
    ac->sendGoal(goal);
    ac->waitForResult();
    
    if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("成功到达目标点: %s", point.name.c_str());
        // 播放介绍语音
        voiceTTS(point.intro.c_str());
    } else {
        ROS_INFO("未能到达目标点: %s", point.name.c_str());
    }
    
    ac->cancelGoal();
    delete ac;
}

// 返回起点实现
void CompetitionNav::returnToStartPoint() {
    NavPoint startPoint = {0.0, 0.0, 0.0, 1.0, "起点", ""};
    
    ac = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);
    ROS_INFO("等待导航服务器启动...");
    ac->waitForServer();
    ROS_INFO("开始返回起点 (0, 0)");

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    
    goal.target_pose.pose.position.x = startPoint.x;
    goal.target_pose.pose.position.y = startPoint.y;
    goal.target_pose.pose.orientation.z = startPoint.z;
    goal.target_pose.pose.orientation.w = startPoint.w;
    
    ac->sendGoal(goal);
    ac->waitForResult();
    
    if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("成功返回起点");
    } else {
        ROS_INFO("未能返回起点");
    }
    
    ac->cancelGoal();
    delete ac;
}

// 参观所有点实现
void CompetitionNav::tourAllPoints() {
    // 随机打乱导航点顺序
    random_shuffle(nav_points.begin(), nav_points.end());
    
    // 依次导航到每个点
    for(const auto& point : nav_points) {
        navigateToPoint(point);
        ros::Duration(1.0).sleep();  // 短暂停顿
    }
    
    // 自动返回起点
    returnToStartPoint();
    
    // 原提示已删除，此处无任何语音输出
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "competition_nav");
    CompetitionNav nav_system;
    
    string audio_file, text;
    bool waiting_for_command = true;
    
    while(ros::ok() && waiting_for_command) {
        // 1. 等待"带我参观一圈"指令
        ROS_INFO("等待唤醒指令...");
        audio_file = nav_system.voiceCollect();
        text = nav_system.voiceDictation(audio_file.c_str());
        
        // 2. 检测到指令后回复并开始导航
        if(text.find("带我参观一圈") != string::npos) {
            nav_system.voiceTTS("好的，这就带您参观");
            
            // 3. 开始导航所有点
            nav_system.tourAllPoints();
            waiting_for_command = false;
        }
    }
    
    return 0;
}
