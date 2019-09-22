#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <std_srvs/Trigger.h>
int main(int argc, char **argv)
{

    ros::init(argc, argv, "single_position_pub");

 
    ros::NodeHandle n;

  
    ros::Publisher position_pub = n.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 100);

    ros::Rate loop_rate(10);

    std::string yaml_file;
    

    if(n.getParam("joint_names", yaml_file))
    {
        ;
    }
    else
    {
        ROS_ERROR("Failed to open yaml file");
        return 0;
    }

    YAML::Node file;
    file = YAML::LoadFile(yaml_file.c_str());
    if (file == NULL){
        ROS_ERROR("Failed to open yaml file");
        return 0;
    }

    YAML::Node joint = file["joint"];
    uint8_t joint_size = joint["names"].size();


    //std::cout<<"1"<<std::endl;
    
    while (ros::ok())
    {
        trajectory_msgs::JointTrajectory *jnt_tra_msg_;
        jnt_tra_msg_ = new trajectory_msgs::JointTrajectory;
        double step[joint_size];   
        double time_from_start;

        //std::cout<<"2"<<std::endl;

        for (uint8_t index = 0; index < joint_size; index++)
        {
            std::string joint_name = joint["names"][index].as<std::string>();
            
            jnt_tra_msg_->joint_names.push_back(joint_name);
            std::cout<<joint_name<<": ";
            std::cin>>step[index];
        }

        //std::cout<<"3"<<std::endl;
   
        std::cout<<"time_from_start: ";
        std::cin>>time_from_start;

        trajectory_msgs::JointTrajectoryPoint jnt_tra_point;

        
        for (uint8_t size = 0; size < joint_size; size++)
        {
            jnt_tra_point.positions.push_back(step[size]);
        }

        jnt_tra_point.time_from_start.fromSec(time_from_start);

        jnt_tra_msg_->points.push_back(jnt_tra_point);


        position_pub .publish(*jnt_tra_msg_);


        ros::spinOnce();

        loop_rate.sleep();

    }


  return 0;
}