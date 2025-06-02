#include"ros/ros.h"
#include"third_pkg/myTestMsg.h"
#include"third_pkg/myTestSrv.h"
void chatterCallback(const third_pkg::myTestMsg::ConstPtr& msg)
{
ROS_INFO("I heard-name:%s,age:%d,ishandsome:%d,salary:%f",msg->name.c_str(),msg->age,msg->handsome,msg->salary);
}
int main(int argc, char **argv)
{
ros::init(argc,argv,"subscribe_node");//ros初始化
ros::NodeHandle n;//定义一个句柄
ros::Rate loop_rate(1);
ros::Subscriber sub = n.subscribe("third_pkg_topic",1000,chatterCallback);
ros::ServiceClient client = n.serviceClient<third_pkg::myTestSrv>("batteryStatus");//客户端定义与初始化
third_pkg::myTestSrv mySrv;
mySrv.request.index = 1;//发送的服务请求：1，获取电量
while(ros::ok())
{
if(client.call(mySrv))
{
ROS_WARN("Client Get Battery:%d",mySrv.response.result);
}
else
{
ROS_ERROR("Failed to call batteryStatus service");
return 1;
}
ros::spinOnce();
loop_rate.sleep();
}
return 0;
}

