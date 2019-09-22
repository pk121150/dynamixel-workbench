#include <ros/ros.h>
#include <math.h> 
#include <yaml-cpp/yaml.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <std_srvs/Trigger.h>

using namespace std;

double findMax(double *dR);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "position_assign");

 
    ros::NodeHandle n;

    double XX[8] = {17.3, 26.3, 26.3, 17.3, 17.3, 8.3, 8.3, 17.3};
    double YY[8] = {18.5, 18.5, 8.5, 8.5, 18.5, 18.5, 8.5, 8.5};

  
    ros::Publisher position_pub = n.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 100);

    ros::Rate loop_rate(10);

    std::string yaml_file;
    bool init_ = false;

    int mode;
    int autoCount = 0;
    double width = 35.3;
    double height =  27.5;
    double r = 0.92;

    double nowX = 17.3;
    double nowY = 13.5;

    double targetX,targetY;
    double targetX_t,targetY_t;
    bool arrive = true;
    
    double nowR1 = pow(pow(nowX,2)+pow(height- nowY,2),0.5);
    double nowR2 = pow(pow(width - nowX,2)+pow(height- nowY,2),0.5);
    double nowR3 = pow(pow(width - nowX,2)+pow(nowY,2),0.5);
    double nowR4 = pow(pow(nowX,2)+pow(nowY,2),0.5);
    
    double targetR1,targetR2,targetR3,targetR4;
    double dR[4];
    double dis;
    double step[4]; 

    bool arriveX = true;
    bool arriveY = true;

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
        if(init_)
        {
            if(arrive)
            {
                if(mode == 0){
                    do{    
                        cout<<"target X (6 ~ "<<width - 6<<"): ";
                        cin>>targetX;
                    }while((targetX < 6 || targetX > (width -6)) && ros::ok());

                    do{    
                        cout<<"target Y (6 ~ "<<height - 6<<"): ";
                        cin>>targetY;
                    }while((targetY < 6 || targetY > (height -6)) && ros::ok());
                }else{
                    targetX = XX[autoCount%8];
                    targetY = YY[autoCount%8];
                    autoCount++;
                }
                arrive = false;
            }

            dis = pow(pow(targetX - nowX,2)+pow(targetY - nowY,2),0.5);

            cout << "dis = "<< dis <<endl;

            if(targetX == nowX)
            {
                arrive =true;
                targetX_t = targetX;
                targetY_t = targetY;

            }else{
                if(targetY == nowY)
                {
                    arrive =true;
                    targetX_t = targetX;
                    targetY_t = targetY;
                }else{
                    targetX_t = targetX;
                    targetY_t = nowY;
                }
            }    
            
                

            

            /*if(dis <= 7)
            //if(true)
            {
                arrive =true;
                targetX_t = targetX;
                targetY_t = targetY;
            }else{

                targetX_t = nowX+(targetX-nowX)*7/dis;
                targetY_t = nowY+(targetY-nowY)*7/dis;
            }*/

            cout<<"targetX_t = "<<targetX_t<<endl;
            cout<<"targetY_t = "<<targetY_t<<endl;
            targetR1 = pow(pow(targetX_t,2)+pow(height- targetY_t,2),0.5);
            targetR2 = pow(pow(width - targetX_t,2)+pow(height- targetY_t,2),0.5);
            targetR3 = pow(pow(width - targetX_t,2)+pow(targetY_t,2),0.5);
            targetR4 = pow(pow(targetX_t,2)+pow(targetY_t,2),0.5);

            dR[0] = targetR1 - nowR1;
            dR[1] = targetR2 - nowR2;
            dR[2] = targetR3 - nowR3;
            dR[3] = targetR4 - nowR4;

            cout<<"dR1: "<<dR[0]<<endl;
            cout<<"dR2: "<<dR[1]<<endl;
            cout<<"dR3: "<<dR[2]<<endl;
            cout<<"dR4: "<<dR[3]<<endl;

            

            nowX = targetX_t;
            nowY = targetY_t;



            nowR1 = targetR1; 
            nowR2 = targetR2; 
            nowR3 = targetR3; 
            nowR4 = targetR4; 


        }


        trajectory_msgs::JointTrajectory *jnt_tra_msg_;
        jnt_tra_msg_ = new trajectory_msgs::JointTrajectory;
          
        double time_from_start;

        //std::cout<<"2"<<std::endl;

        for (uint8_t index = 0; index < joint_size; index++)
        {
            std::string joint_name = joint["names"][index].as<std::string>();
            
            jnt_tra_msg_->joint_names.push_back(joint_name);
            if(!init_)
            {
                if(index == 0)
                    step[0] = -0.53382531418 ;//-1.6076118657
                else if(index == 1)   
                    step[1] = 1.02930110867;
                else if(index == 2)   
                    step[2] = -1.27320405395;
                else if(index == 3)   
                    step[3] = -0.19174759848;
            }else{

                if(index == 0)
                    step[0] = step[0] - dR[0]/r;
                else if(index == 1)   
                    step[1] = step[1] + dR[1]/r;
                else if(index == 2)   
                    step[2] = step[2] - dR[2]/r;
                else if(index == 3)   
                    step[3] = step[3] + dR[3]/r;
                
            }
            cout<<step[index]<<" "<<endl;
        }
        cout<<endl<<endl;
        //std::cout<<"3"<<std::endl;
    
        if(!init_)
        {
            time_from_start = 2.0;
        }else{
            //time_from_start = dis/4;
            time_from_start = findMax(dR)/(r*2*M_PI*0.25*4);
            
            if(time_from_start == 0)
                time_from_start = 0.1; 
            cout<<"time_from_start = "<< time_from_start<<endl;  
            cout<<"move_time = "<< time_from_start*4<<endl;  
            //std::cout<<"time_from_start: ";
            //std::cin>>time_from_start;
        }

             

        trajectory_msgs::JointTrajectoryPoint jnt_tra_point;

        
        for (uint8_t size = 0; size < joint_size; size++)
        {
            jnt_tra_point.positions.push_back(step[size]);
        }

        jnt_tra_point.time_from_start.fromSec(time_from_start);

        jnt_tra_msg_->points.push_back(jnt_tra_point);
        

        if(!init_)
        {
            std::cout<<"Start init"<<std::endl;
            ros::Duration(2.0).sleep();
        }

        position_pub .publish(*jnt_tra_msg_);

        if(!init_)
        {
            ros::Duration(8.0).sleep();
            std::cout<<"Finish init"<<std::endl;      
            init_ = true;    

            cout<<"mode (0 for assign position, 1 for auto) : ";
            cin>>mode;
        }else{
            //ros::Duration(time_from_start*4).sleep();
            sleep(time_from_start*4+1);
        }


        ros::spinOnce();

        loop_rate.sleep();

    }


  return 0;
}



double findMax(double *dR)
{
    double max = abs(dR[0]);

    for(int i = 1; i < 4; i++)
    {
        if(abs(dR[i]) > max)
            max = abs(dR[i]);
    }

    return max;
}