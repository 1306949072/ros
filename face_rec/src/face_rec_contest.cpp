#include <ros/ros.h>
#include <face_rec/recognition_results.h>
#include <robot_audio/robot_tts.h>
#include <robot_audio/robot_iat.h>
#include <robot_audio/Collect.h>
#include <string>
#include <iostream>

using namespace std;

// 人脸识别服务客户端
bool face_recognition(ros::NodeHandle &nh, string &recognized_name) {
    ros::ServiceClient face_client = nh.serviceClient<face_rec::recognition_results>("face_recognition_results");
    face_rec::recognition_results srv;
    srv.request.mode = 1;  // 从摄像头获取图像
    
    if (face_client.call(srv)) {
        if (srv.response.success && srv.response.result.num > 0) {
            recognized_name = srv.response.result.face_data[0].name;
            return true;
        }
    }
    return false;
}

// 语音合成函数
void speak(ros::NodeHandle &nh, const string &text) {
    ros::ServiceClient tts_client = nh.serviceClient<robot_audio::robot_tts>("voice_tts");
    robot_audio::robot_tts tts_srv;
    tts_srv.request.text = text;
    if (tts_client.call(tts_srv)) {
        string cmd = "play " + tts_srv.response.audiopath;
        system(cmd.c_str());
    }
}

// 语音采集和识别函数
string listen(ros::NodeHandle &nh) {
    ros::ServiceClient collect_client = nh.serviceClient<robot_audio::Collect>("voice_collect");
    ros::ServiceClient iat_client = nh.serviceClient<robot_audio::robot_iat>("voice_iat");
    
    // 采集语音
    robot_audio::Collect collect_srv;
    collect_srv.request.collect_flag = 1;
    if (collect_client.call(collect_srv)) {
        // 语音识别
        robot_audio::robot_iat iat_srv;
        iat_srv.request.audiopath = collect_srv.response.voice_filename;
        if (iat_client.call(iat_srv)) {
            return iat_srv.response.text;
        }
    }
    return "";
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "face_rec_contest");
    ros::NodeHandle nh;
    
    // 等待所有服务就绪
    ros::service::waitForService("face_recognition_results");
    ros::service::waitForService("voice_collect");
    ros::service::waitForService("voice_iat");
    ros::service::waitForService("voice_tts");
    
    ROS_INFO("系统准备就绪，等待唤醒...");
    
    while (ros::ok()) {
        // 步骤1：等待"我是谁"的询问
        string text = listen(nh);
        if (text.find("我是谁") != string::npos) {
            // 步骤2：回应并开始识别
            speak(nh, "好的，让我看一看");
            
            // 步骤3：进行人脸识别
            string name;
            if (face_recognition(nh, name)) {
                // 步骤4：反馈识别结果
                string response = "你是" + name;
                speak(nh, response);
            } else {
                speak(nh, "抱歉，我没有认出你");
            }
        }
        
        ros::spinOnce();
        sleep(1);
    }
    
    return 0;
}
