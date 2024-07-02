#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <lrobot_base/lrobot_base_ros.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace LRobot;
std::shared_ptr<LRobotBaseRos> robot;

void DetachRobot(int singal){
    (void)signal;
    robot->Stop();
}

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    robot=std::make_shared<LRobotBaseRos>("LRobot");
    if(robot->Initialize()){
        std::cout<<"Robot initialized,start running ..."<<std::endl;
        robot->Run();
    }
    return 0;
}

