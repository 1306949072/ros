#include <ros/ros.h>
#include <robot_audio/robot_iat.h>
#include <robot_audio/Collect.h>
#include <robot_audio/robot_tts.h>
#include <actionlib/client/simple_action_client.h>
#include "move_base_msgs/MoveBaseAction.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <map>

using namespace std;

// 定义地点结构体
struct Point {
    float x;    // x坐标
    float y;    // y坐标
    float z;    // 姿态z
    float w;    // 姿态w
    string name; // 地点名字
    string present; // 介绍语
};

// 定义五个馆的坐标和介绍信息
struct Point m_point[5] = {
    {1.026, 1.109, -0.012, 1.000, "深圳馆", "深圳，是广东副省级市、经济特区。毗邻香港，经济发达，创新力强，有众多世界500 强企业，是粤港澳大湾区中心城市。"},
    {1.073, 2.120, 0.013, 1.000, "上海馆", "上海，简称 “沪” 或 “申”，是中国直辖市，位于长江入海口，是国际经济、金融、贸易、航运、科技创新中心，有独特海派文化。"},
    {2.48, 0.13, 0.03, 0.99, "北京馆", "北京，中国首都，千年古都与现代都市交融，尽显独特魅力。这里有宏伟的故宫、绵延的长城等历史古迹，见证着岁月的沧桑变迁。"},
    {2.48, 1.077, -0.026, 1.000, "广州馆", "广州，别称羊城、花城，广东省会。历史悠久，美食诱人，经济发达，是充满魅力与活力的国家中心城市和粤港澳大湾区核心。"},
    {2.48, 2.120, -0.032, 0.999, "吉林馆", "吉林省，简称 “吉”，地处东北中部，与俄、朝接壤。是重要商品粮基地与老工业基地，有长白山等美景，人文风情浓郁。"}
};

// 定义出发点
struct Point start_point = {0.0, 0.0, 0.0, 1.0, "出发点", "这里是起点"};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> AC;

// 文本清理函数
string clean_text(const string& input) {
    string result = input;
    // 移除空格
    result.erase(remove_if(result.begin(), result.end(), ::isspace), result.end());
    // 移除标点
    result.erase(remove_if(result.begin(), result.end(), ::ispunct), result.end());
    return result;
}

// 模糊匹配函数
bool fuzzy_match(const string& input, const string& target) {
    string clean_input = clean_text(input);
    string clean_target = clean_text(target);
    
    // 完全匹配
    if(clean_input == clean_target) return true;
    
    // 包含关系
    if(clean_input.find(clean_target) != string::npos || 
       clean_target.find(clean_input) != string::npos) {
        return true;
    }
    
    // 处理缺少"馆"字的情况
    if(clean_target.back() == '馆') {
        string target_without_suffix = clean_target.substr(0, clean_target.length()-1);
        if(clean_input == target_without_suffix || 
           clean_input.find(target_without_suffix) != string::npos) {
            return true;
        }
    }
    
    // 添加常见识别错误的映射
    map<string, string> common_errors = {
        {"被禁", "北京"},
        {"光州", "广州"},
        {"上进", "上海"},
        {"北京", "北京馆"},
        {"广州", "广州馆"},
        {"深圳", "深圳馆"},
        {"上海", "上海馆"},
        {"吉林", "吉林馆"}
    };
    
    for(auto& error : common_errors) {
        if(clean_input.find(error.first) != string::npos) {
            string corrected_input = clean_input;
            size_t pos = corrected_input.find(error.first);
            corrected_input.replace(pos, error.first.length(), error.second);
            
            if(corrected_input == clean_target || 
               corrected_input.find(clean_target) != string::npos) {
                return true;
            }
        }
    }
    
    return false;
}

class VoiceNavSystem {
public:
    VoiceNavSystem();
    string voice_collect(); // 语音采集
    string voice_dictation(const char* filename); // 语音听写
    string voice_tts(const char* text); // 语音合成
    void goto_nav(struct Point* point); // 导航到目标位置
    
private:
    ros::NodeHandle n;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* ac;
    ros::ServiceClient collect_client, dictation_client, tts_client;
};

VoiceNavSystem::VoiceNavSystem() {
    collect_client = n.serviceClient<robot_audio::Collect>("voice_collect");
    dictation_client = n.serviceClient<robot_audio::robot_iat>("voice_iat");
    tts_client = n.serviceClient<robot_audio::robot_tts>("voice_tts");
}

string VoiceNavSystem::voice_collect() {
    ros::service::waitForService("voice_collect");
    robot_audio::Collect srv;
    srv.request.collect_flag = 1;
    collect_client.call(srv);
    return srv.response.voice_filename;
}

string VoiceNavSystem::voice_dictation(const char* filename) {
    ros::service::waitForService("voice_iat");
    robot_audio::robot_iat srv;
    srv.request.audiopath = filename;
    dictation_client.call(srv);
    ROS_INFO("语音识别结果: %s", srv.response.text.c_str());
    return srv.response.text;
}

string VoiceNavSystem::voice_tts(const char* text) {
    ros::service::waitForService("voice_tts");
    robot_audio::robot_tts srv;
    srv.request.text = text;
    tts_client.call(srv);
    string cmd = "play " + srv.response.audiopath;
    system(cmd.c_str());
    sleep(1);
    return srv.response.audiopath;
}

void VoiceNavSystem::goto_nav(struct Point* point) {
    ac = new AC("move_base", true);
    ROS_INFO("等待导航服务启动...");
    ac->waitForServer();
    ROS_INFO("导航服务已启动，发送目标点...");
    
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = point->x;
    goal.target_pose.pose.position.y = point->y;
    goal.target_pose.pose.orientation.z = point->z;
    goal.target_pose.pose.orientation.w = point->w;
    
    ac->sendGoal(goal);
    ac->waitForResult();
    
    if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("成功到达目标点!");
        // 播放地点介绍
        voice_tts(point->present.c_str());
    } else {
        ROS_INFO("导航失败!");
    }
    
    delete ac;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "voice_nav_return_system");
    VoiceNavSystem nav_system;
    string dir, text;
    
    while(ros::ok()) {
        // 1. 语音采集
        dir = nav_system.voice_collect();
        // 2. 语音听写
        text = nav_system.voice_dictation(dir.c_str());
        
        // 3. 判断是否包含"我要到XXX馆"
        size_t pos = text.find("我要到");
        if(pos != string::npos) {
            // 提取馆名
            string place = text.substr(pos + 3);
            ROS_INFO("提取的地点名称: %s", place.c_str());
            
            // 4. 查找匹配的馆
            bool found = false;
            for(int i = 0; i < 5; i++) {
                if(fuzzy_match(place, m_point[i].name)) {
                    found = true;
                    // 5. 回复确认信息
                    string reply = "好的，这就带您去" + m_point[i].name;
                    nav_system.voice_tts(reply.c_str());
                    
                    // 6. 导航到目标馆
                    nav_system.goto_nav(&m_point[i]);
                    
                    // 7. 返回出发点
               
                    nav_system.goto_nav(&start_point);
                    
                    break;
                }
            }
            
            if(!found) {
                ROS_INFO("未能识别的场馆: %s", place.c_str());
                nav_system.voice_tts("抱歉，我没有找到您说的馆");
            }
        }
    }
    
    return 0;
}
