#include <ros/ros.h>
#include <math.h> 
#include <yaml-cpp/yaml.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_srvs/Trigger.h>
#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>
#include <assert.h>
#include <string.h>
#include <signal.h>
#include "sensor_msgs/JointState.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>

using namespace std;

int mode;
int mode_;
bool close_ = false;
bool getFirstPosition = false;
double nowPosition[4];
double step[4]; 
bool moving = false;
bool moving_ = false;
bool sendFirstGoal = false;
bool targetInput = false;
double dRadian[4];
double nowX ;
double nowY ;
double targetX,targetY;
double targetX_click,targetY_click;
double X_time[10];
double Y_time[10];
double width = 35.3;
double height = 27.5;
double r = 0.9;
double navigationTime[100];
int goalNum;
double goalX[100],goalY[100]; 
double nowX_,nowY_;
int Xcount;
int Ycount;
double Xdis;
double Ydis;
bool Xgo;
bool Ygo;
/*void mySigintHandler(int sig)
{
  ros::shutdown();
}*/

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


void positionCallback(sensor_msgs::JointState msg)
{
    moving = false;
    if(msg.position.size() != 0){
        for(int i = 0; i<4; i++)
        {
            nowPosition[i] = msg.position[i];
            //cout << nowPosition[i] << endl;
            if(sendFirstGoal)
            {
                //cout << abs(nowPosition[i] - step[i]) << endl;
                if(abs(nowPosition[i] - step[i]) > 0.02)
                    moving = true;
            }
        }
        getFirstPosition = true;
    }

    //cout << "moving = " <<moving<<endl; 
}

void clickCallback(geometry_msgs::PointStamped msg)
{
    targetX_click = msg.point.x;
    targetY_click = msg.point.y;
}


int getch(void)
{
    int c=0;
    struct termios org_opts, new_opts;
    int res=0;
    /*---- store old settings ----*/
    res=tcgetattr(STDIN_FILENO, &org_opts);
    assert(res==0);
    /*---- set new terminal parms ----*/
    memcpy(&new_opts, &org_opts, sizeof(new_opts));
    new_opts.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ECHOPRT | ECHOKE | ICRNL);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_opts);
    c=getchar();
    /*---- restore old settings ----*/
    res=tcsetattr(STDIN_FILENO, TCSANOW, &org_opts);
    assert(res==0);
return c;
}


void *thread(void *ptr)
{
    int ch;
    while (!close_)
    {   
        while (!targetInput) 
        {       
            ch=getch();
            if(ch == 32)
            {
                
                if(mode == 0)
                {
                    mode = 1;
                    cout << endl;             
                    cout << "## CHANGE MODE TO AUTO NAVIGATION ##"<<endl<<endl;
                }else
                {
                    mode = 0;
                    cout << endl;
                    cout << "## CHANGE MODE TO POSITION ASSIGN ##"<<endl<<endl;
                }
                
            }else if(ch == 27)
            {
                close_ = true;
                break;
            }
            
            
        }
        
    }
    cout << "CLOSE THREAD" << endl;
    return 0;
}

void *thread2(void *ptr)
{
    //ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    ros::Rate r(10);
    
    
    bool startP = false;
    bool startN = false;

    
    while (ros::ok() && !close_)
    {
        visualization_msgs::Marker text;
        text.header.frame_id="/map";
        text.header.stamp = ros::Time::now();
        text.ns = "text";
        text.action = visualization_msgs::Marker::ADD;
        text.pose.orientation.w = 1.0;
        text.id = 7.5;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.lifetime = ros::Duration();
        text.scale.z = 0.6;
        text.color.b = 0;
        text.color.g = 0;
        text.color.r = 1;
        text.color.a = 1;

        geometry_msgs::Pose pose;
        pose.position.x = 0;
        pose.position.y = 7;
        pose.position.z = 1;
        
        text.pose=pose;
        




        for(int i = 0 ; i< 4 ; i++)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "cylinder";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;

            if( i == 0 ){
                marker.pose.position.x = 7.5;
                marker.pose.position.y = 6.5;               
            }else if( i == 1 ){
                marker.pose.position.x = 7.5;
                marker.pose.position.y = -6.5;
            }else if( i == 2 ){
                marker.pose.position.x = -7.5;
                marker.pose.position.y = -6.5;
            }else{
                marker.pose.position.x = -7.5;
                marker.pose.position.y = 6.5;
            }
            marker.pose.position.z = 1;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 2.0;
            marker.scale.y = 2.0;
            marker.scale.z = 2.0;

            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0;

            marker.lifetime = ros::Duration();

            marker_pub.publish(marker);
            
        }

        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id  = "/map";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns= "lines";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 4;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = 0.4;
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;
        
        geometry_msgs::Point p;
         
        p.x = 6.2;
        p.y = 5.2;
        p.z = 0.2;
        line_strip.points.push_back(p);
        p.x = -6.2;
        p.y = 5.2;
        p.z = 0.2;
        line_strip.points.push_back(p);
        p.x = -6.2;
        p.y = -5.2;
        p.z = 0.2;
        line_strip.points.push_back(p);
        p.x = 6.2;
        p.y = -5.2;
        p.z = 0.2;
        line_strip.points.push_back(p);
        p.x = 6.2;
        p.y = 5.2;
        p.z = 0.2;
        line_strip.points.push_back(p);
        marker_pub.publish(line_strip);

        visualization_msgs::Marker center;
        center.header.frame_id = "/map";
        center.header.stamp = ros::Time::now();
        center.ns = "Sphere";
        center.id = 5;
        center.type = visualization_msgs::Marker::SPHERE;
        center.action = visualization_msgs::Marker::ADD;

        center.pose.orientation.x = 0.0;
        center.pose.orientation.y = 0.0;
        center.pose.orientation.z = 0.0;
        center.pose.orientation.w = 1.0;      

        center.scale.x = 1.0;
        center.scale.y = 1.0;
        center.scale.z = 1.0;

        center.color.r = 0.5f;
        center.color.g = 0.5f;
        center.color.b = 0.5f;
        center.color.a = 1.0;

        center.lifetime = ros::Duration();
        
       
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = "/map";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "line_lists";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.id = 6;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.1;

        line_list.color.r = 0.0;
        line_list.color.g = 0.0;
        line_list.color.b = 0.0;
        line_list.color.a = 1.0;
        line_list.lifetime = ros::Duration();
        if(!moving_ )
        {
            
            center.pose.position.x = nowX/2 - width/4;
            center.pose.position.y = nowY/2 - height/4;
            center.pose.position.z = 1;
            marker_pub.publish(center);
           
            geometry_msgs::Point p1,p2;

            p1.x = nowX/2 - width/4;
            p1.y = nowY/2 - height/4;
            p1.z = 1;
            p2.z = 1;
            line_list.points.push_back(p1);

            p2.x = 7.5;
            p2.y = 6.5;
            line_list.points.push_back(p2);
            line_list.points.push_back(p1);
            p2.x = -7.5;
            p2.y = 6.5;
            line_list.points.push_back(p2);
            line_list.points.push_back(p1);
            p2.x = -7.5;
            p2.y = -6.5;
            line_list.points.push_back(p2);
            line_list.points.push_back(p1);
            p2.x = 7.5;
            p2.y = -6.5;
            line_list.points.push_back(p2);
            
            marker_pub.publish(line_list);


            if(mode == 0)
                text.text="## POSITION ASSIGN MODE ##";
            else
                text.text="## AUTO NAVIGATION MODE ##";
             marker_pub.publish(text);

        }else{
            
            if(mode_ == 0){


                center.pose.position.z = 1;
                center.pose.position.y = nowY_/2 - height/4;


                if(Xgo)
                    for(int i = 0; i < Xcount ; i++)
                    {
                        int count = ceil(X_time[i]*4/0.1);
                        
                        
                        for(int j = 0 ; j < count; j++)
                        {
                            center.pose.position.x = (nowX_ - width/2 + Xdis * i + Xdis*((double)(j+1)/(double)count))/2;                             
                            marker_pub.publish(center);

                            if(mode == 0)
                                text.text="## POSITION ASSIGN MODE ##";
                            else
                                text.text="## AUTO NAVIGATION MODE ##";
                            marker_pub.publish(text);


                            geometry_msgs::Point p1,p2;

                            p1.x = center.pose.position.x;
                            p1.y = nowY_/2 - height/4;
                            p1.z = 1;
                            p2.z = 1;
                            line_list.points.clear();
                            line_list.points.push_back(p1);

                            p2.x = 7.5;
                            p2.y = 6.5;
                            line_list.points.push_back(p2);
                            line_list.points.push_back(p1);
                            p2.x = -7.5;
                            p2.y = 6.5;
                            line_list.points.push_back(p2);
                            line_list.points.push_back(p1);
                            p2.x = -7.5;
                            p2.y = -6.5;
                            line_list.points.push_back(p2);
                            line_list.points.push_back(p1);
                            p2.x = 7.5;
                            p2.y = -6.5;
                            line_list.points.push_back(p2);
                            
                            marker_pub.publish(line_list);


                            ros::spinOnce();

                            if(j < count/8){
                                double sleepTime = 0.17 - 0.08 * ( (double)j/((double)count/8));
                                ros::Duration(sleepTime ).sleep();
                            }else if (j > 7*count/8){
                                double sleepTime = 0.09 + 0.08 * ( ((double)j - (double)7*count/8)/((double)count/8));
                                ros::Duration(sleepTime ).sleep();
                            }else{
                                double sleepTime = 0.07;
                                ros::Duration(sleepTime ).sleep();
                            }
                        }

                        ros::Duration(0.02 * (double)count*3/4 ).sleep();


                    }



                center.pose.position.x = targetX/2 - width/4;
                if(Ygo)
                    for(int i = 0; i < Ycount ; i++)
                    {
                        int count = ceil(Y_time[i]*4/0.1);
                        
                        
                        for(int j = 0 ; j < count; j++)
                        {
                            center.pose.position.y = (nowY_ - height/2 + Ydis * i + Ydis*((double)(j+1)/(double)count))/2;
                            
                                
                            marker_pub.publish(center);

                            if(mode == 0)
                                text.text="## POSITION ASSIGN MODE ##";
                            else
                                text.text="## AUTO NAVIGATION MODE ##";
                            marker_pub.publish(text);


                            geometry_msgs::Point p1,p2;

                            p1.x = center.pose.position.x;
                            p1.y = center.pose.position.y;
                            p1.z = 1;
                            p2.z = 1;
                            line_list.points.clear();
                            line_list.points.push_back(p1);

                            p2.x = 7.5;
                            p2.y = 6.5;
                            line_list.points.push_back(p2);
                            line_list.points.push_back(p1);
                            p2.x = -7.5;
                            p2.y = 6.5;
                            line_list.points.push_back(p2);
                            line_list.points.push_back(p1);
                            p2.x = -7.5;
                            p2.y = -6.5;
                            line_list.points.push_back(p2);
                            line_list.points.push_back(p1);
                            p2.x = 7.5;
                            p2.y = -6.5;
                            line_list.points.push_back(p2);
                            
                            marker_pub.publish(line_list);



                            ros::spinOnce();

                            if(j < count/8){
                                double sleepTime = 0.17 - 0.08 * ( (double)j/((double)count/8));
                                ros::Duration(sleepTime ).sleep();
                            }else if (j > 7*count/8){
                                double sleepTime = 0.09 + 0.08 * ( ((double)j - (double)7*count/8)/((double)count/8));
                                ros::Duration(sleepTime ).sleep();
                            }else{
                                double sleepTime = 0.07;
                                ros::Duration(sleepTime ).sleep();
                            }
                        }

                        ros::Duration(0.02 * (double)count*3/4 ).sleep();


                    }

        
                moving_ = false;
            }else{
                    
                    for(int i = 0; i < goalNum ; i++)
                    {
                        int count = ceil(navigationTime[i]*4/0.1);
                        center.pose.position.z = 1;

                        for(int j = 0 ; j < count; j++)
                        {
                            if(i == 0){
                                center.pose.position.x = (nowX_ - width/2 + (goalX[0] + width/2 - nowX_)*((double)(j+1)/(double)count))/2;
                                center.pose.position.y = (nowY_ - height/2 + (goalY[0] + height/2- nowY_)*((double)(j+1)/(double)count))/2;
                            }else{
                                center.pose.position.x = (goalX[i-1]  + (goalX[i] - goalX[i-1])*((double)(j+1)/(double)count))/2;
                                center.pose.position.y = (goalY[i-1]  + (goalY[i] - goalY[i-1])*((double)(j+1)/(double)count))/2;
                            }
                            
                            marker_pub.publish(center);

                            if(mode == 0)
                                text.text="## POSITION ASSIGN MODE ##";
                            else
                                text.text="## AUTO NAVIGATION MODE ##";
                            marker_pub.publish(text);



                            geometry_msgs::Point p1,p2;

                            p1.x = center.pose.position.x;
                            p1.y = center.pose.position.y;
                            p1.z = 1;
                            p2.z = 1;
                            line_list.points.clear();
                            line_list.points.push_back(p1);

                            p2.x = 7.5;
                            p2.y = 6.5;
                            line_list.points.push_back(p2);
                            line_list.points.push_back(p1);
                            p2.x = -7.5;
                            p2.y = 6.5;
                            line_list.points.push_back(p2);
                            line_list.points.push_back(p1);
                            p2.x = -7.5;
                            p2.y = -6.5;
                            line_list.points.push_back(p2);
                            line_list.points.push_back(p1);
                            p2.x = 7.5;
                            p2.y = -6.5;
                            line_list.points.push_back(p2);
                            
                            marker_pub.publish(line_list);

                            ros::spinOnce();

                            if(j < count/8){
                                double sleepTime = 0.17 - 0.08 * ( (double)j/((double)count/8));
                                ros::Duration(sleepTime ).sleep();
                            }else if (j > 7*count/8){
                                double sleepTime = 0.09 + 0.08 * ( ((double)j - (double)7*count/8)/((double)count/8));
                                ros::Duration(sleepTime ).sleep();
                            }else{
                                double sleepTime = 0.07;
                                ros::Duration(sleepTime ).sleep();
                            }


                            
                            //cout << center.pose.position.x <<" "<<center.pose.position.y<<endl;
                            //ros::Rate r2(10);
                            //r2.sleep();
                            
                        }

                        ros::Duration(0.02 * (double)count*3/4 ).sleep();
                    }
                    nowX_ = goalX[goalNum-1] + width/2;;
                    nowY_ = goalY[goalNum-1] + height/2;
                    moving_ = false;
            }

        }    
        
        ros::spinOnce();
        r.sleep();
    }
    
    cout << "CLOSE THREAD2" << endl;
    return 0;
}




void calculate_target_radian(double nowX, double nowY, double targetX, double targetY, double width, double height, double r)
{
    double nowR[4];
    double targetR[4];
   

    
    

    nowR[0] = pow(pow(nowX,2)+pow(height- nowY,2),0.5);
    nowR[1] = pow(pow(width - nowX,2)+pow(height- nowY,2),0.5);
    nowR[2] = pow(pow(width - nowX,2)+pow(nowY,2),0.5);
    nowR[3] = pow(pow(nowX,2)+pow(nowY,2),0.5);

    targetR[0] = pow(pow(targetX,2)+pow(height- targetY,2),0.5);
    targetR[1] = pow(pow(width - targetX,2)+pow(height- targetY,2),0.5);
    targetR[2] = pow(pow(width - targetX,2)+pow(targetY,2),0.5);
    targetR[3] = pow(pow(targetX,2)+pow(targetY,2),0.5);

    for(int i = 0; i< 4; i++){
        dRadian[i] =  (targetR[i] - nowR[i])/r ;
       // cout << dRadian[i] <<endl;
    }    
   
 
}



int main(int argc, char **argv){


    double center[4] = {-0.7025632262229919,  -0.23316508531570435, 0.7194370031356812, -1.840777039527893};
    
    nowX = width/2;
    nowY = height/2;
    targetX_click = 0;
    targetY_click = 0;
    double targetX_t,targetY_t;
    double nowR[4];

    nowR[0] = pow(pow(nowX,2)+pow(height- nowY,2),0.5);
    nowR[1] = pow(pow(width - nowX,2)+pow(height- nowY,2),0.5);
    nowR[2] = pow(pow(width - nowX,2)+pow(nowY,2),0.5);
    nowR[3] = pow(pow(nowX,2)+pow(nowY,2),0.5);

    mode = 0;
    double targetR[4];
    double dR[4];
    
    
    double dis;
    

    bool arriveX = true;
    bool arriveY = true;

    trajectory_msgs::JointTrajectory *jnt_tra_msg_;
    
    double time_from_start;

    //ros initialize
    ros::init(argc, argv, "position_assign" , ros::init_options::NoSigintHandler);
    ros::NodeHandle n;
    //signal(SIGINT, mySigintHandler);

    ros::Publisher position_pub = n.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 100);
    ros::Subscriber position_sub = n.subscribe("/dynamixel_workbench/joint_states", 1000, positionCallback);
    ros::Subscriber click_sub = n.subscribe("/clicked_point", 100, clickCallback);
    ros::Rate loop_rate(10);

    //sleep(3);

    //navigation param
    
    ros::param::get("/numOfGoals", goalNum);
    vector<double> goalX_;
	vector<double> goalY_;
    

    n.getParam("/goalListX", goalX_);
	n.getParam("/goalListY", goalY_);

    //cout << goalNum <<endl;
    
    for(int i = 0; i < goalNum; i++)
	{
        goalX[i] = goalX_[i];
        goalY[i] = goalY_[i];
        //cout << goalX[i] <<" "<< goalY[i]<<endl; 
	}


    // get joint names and number
    std::string yaml_file;    
    if(n.getParam("joint_names", yaml_file))
    {
        cout << endl;
        cout << "Success to get joint_names" << endl << endl; 
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


    while(!getFirstPosition){
        ros::spinOnce();
        loop_rate.sleep();
    }


    //move to center point
    if(!close_)
    {
        jnt_tra_msg_ = new trajectory_msgs::JointTrajectory;

        for(int i = 0; i < 4; i++)
        {
            dRadian[i] = center[i] - nowPosition[i];
            step[i] = center[i];
        }
        time_from_start = findMax(dRadian)/(2 * M_PI*0.25*4);
        if(time_from_start  < 0.1)
            time_from_start = 0.1;
        trajectory_msgs::JointTrajectoryPoint jnt_tra_point;   
        for (uint8_t size = 0; size < joint_size; size++)
        {
            std::string joint_name = joint["names"][size].as<std::string>();
            jnt_tra_msg_->joint_names.push_back(joint_name);
            jnt_tra_point.positions.push_back(step[size]);
        }
        jnt_tra_point.time_from_start.fromSec(time_from_start);
        jnt_tra_msg_->points.push_back(jnt_tra_point);
        position_pub.publish(*jnt_tra_msg_);
        sendFirstGoal = true;
        //ros::spinOnce();
        //sleep(1);
        cout << endl << "MOVING TO CENTER POINT";
        moving = true;
        while(moving){
            ros::spinOnce();
            sleep(1);
            cout << "." <<endl;
        }

        cout << endl <<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<< endl<< endl;

    }

    pthread_t id2;
    int ret2 = pthread_create(&id2, NULL, thread2, NULL);
    if(ret2) {
        cout << "Create pthread2 error!" << endl;
        return 1;
    }
   


    // choose operation mode
    /*do{
        cout << "Please choose operation mode" << endl;
        cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<< endl;
        cout << "0 for Position Assign" << endl;
        cout << "1 for Auto Navigation" << endl;
        cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<< endl;
        cin>>mode;
        cout << endl;
    }while(mode != 0 && mode != 1);*/

    cout << "## POSITION ASSIGN MODE ##" << endl  << endl;

    //open thread for changing mode
    pthread_t id;
    int ret = pthread_create(&id, NULL, thread, NULL);
    if(ret) {
        cout << "Create pthread1 error!" << endl;
        return 1;
    }

    



    while(ros::ok() && !close_)
    {
        nowX_ = nowX;
        nowY_ = nowY;
        if(mode == 0)
        {
            mode_ =  mode;
            moving_ = false;
            
            int tempXi, tempYi;
            bool invalidX = false;
            bool invalidY = false;

            //targetInput = true;

            cout << "NOW POSITION : ( " << nowX - width/2 <<" , "<< nowY - height/2<<")"<<endl;
            cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
            /*do{
                cout << "PLEASE INPUT TARGET_X (-12 ~ 12) :" ;
                cin >> tempXi ;

                if(tempXi > 12 || tempXi < -12)
                    invalidX = true;
                else
                    invalidX = false;

                if(invalidX)
                    cout << "Invalid Input !!"<<endl;

            }while(invalidX);

            do{
                cout << "PLEASE INPUT TARGET_Y (-10 ~ 10) :" ;
                cin >> tempYi ;

                if(tempYi > 10 || tempYi < -10)
                    invalidY = true;
                else
                    invalidY = false;

                if(invalidY)
                    cout << "Invalid Input !!"<<endl;
            }while(invalidY);
            
            
            
            targetX = tempXi + width/2;
            targetY = tempYi + height/2;*/
            //targetInput = false;
            targetX = targetX_click*2 + width/2;
            targetY = targetY_click*2 + height/2;


            jnt_tra_msg_ = new trajectory_msgs::JointTrajectory;
            
            Xgo = false;
            Ygo = false;

            double total_time = 0;

            for (int i = 0; i < 4; i++)
            {
                std::string joint_name = joint["names"][i].as<std::string>();
                jnt_tra_msg_->joint_names.push_back(joint_name);
               
            }    

            

            

            time_from_start = 0;
            //X_time = 0;
            //Y_time = 0;

            

            if(targetX != nowX){

                Xgo = true;

                

                Xcount = ceil(abs(targetX - nowX)/8);
                Xdis = (targetX - nowX)/Xcount;
                for(int j = 0; j< Xcount;j++)
                {
                    calculate_target_radian(nowX + Xdis*j, nowY, nowX + Xdis*(j+1), nowY, width, height, r);

                    for(int i = 0; i < 4; i++)
                    {
              
                        if(i == 0)
                            step[0] = step[0] - dRadian[0];
                        else if(i == 1)   
                            step[1] = step[1] + dRadian[1];
                        else if(i == 2)   
                            step[2] = step[2] - dRadian[2];
                        else if(i == 3)   
                            step[3] = step[3] + dRadian[3];
                  
                    }
                    time_from_start = time_from_start + findMax(dRadian)/(2 * M_PI*0.25*4);
                    X_time[j] = findMax(dRadian)/(2 * M_PI*0.25*4);
                    if(time_from_start  < 0.1)
                        time_from_start = 0.1;

                    trajectory_msgs::JointTrajectoryPoint jnt_tra_pointx;   
                    for (int i = 0; i < 4; i++)
                    {
                        jnt_tra_pointx.positions.push_back(step[i]);
                    }

                    jnt_tra_pointx.time_from_start.fromSec(time_from_start);
                    jnt_tra_msg_->points.push_back(jnt_tra_pointx);
                
                }               
                
                nowX = targetX;    
                                      
            }

            if(targetY != nowY){
                
                Ygo = true;

                Ycount = ceil(abs(targetY - nowY)/10);
                Ydis = (targetY - nowY)/Ycount;
                for(int j = 0; j< Ycount;j++)
                {
                    
                    calculate_target_radian(nowX, nowY + Ydis*j, nowX, nowY + Ydis*(j+1), width, height, r);
                    for(int i = 0; i < 4; i++)
                    {
                        if(i == 0)
                            step[0] = step[0] - dRadian[0];
                        else if(i == 1)   
                            step[1] = step[1] + dRadian[1];
                        else if(i == 2)   
                            step[2] = step[2] - dRadian[2];
                        else if(i == 3)   
                            step[3] = step[3] + dRadian[3];
                    }

                    time_from_start = time_from_start + findMax(dRadian)/(2 * M_PI*0.25*4);
                    Y_time[j] = findMax(dRadian)/(2 * M_PI*0.25*4);
                    if(time_from_start  < 0.1)
                        time_from_start = 0.1;

                    trajectory_msgs::JointTrajectoryPoint jnt_tra_pointy;    
                    for (int i = 0; i < 4; i++)
                    {
                        jnt_tra_pointy.positions.push_back(step[i]);
                    }

                    jnt_tra_pointy.time_from_start.fromSec(time_from_start);
                    jnt_tra_msg_->points.push_back(jnt_tra_pointy);
                }

                
                nowY = targetY;
                //Y_time = time_from_start - X_time;
                
            }
            total_time = time_from_start * 4 + 1;

            if(Xgo || Ygo){
                position_pub.publish(*jnt_tra_msg_);
                moving_ = true;
                cout << "START MOVING !!!" <<endl;
                cout << "TARGET POINT : ( " << targetX - width/2 <<" , "<< targetY - height/2 <<")"<<endl<<endl;  

            }else{
                //cout << "ORIGINAL POINT ~~" <<endl<<endl;
            }
            //cout << "moving~~~~~~~~~ = " <<moving_<<endl;  
            sleep(total_time);

        }else if(mode == 1)
        {
            mode_ =  mode;
            moving_ = false;
            double total_time = 0;
            

            if(goalNum == 0)
            {
                cout << "Navigation Point not set yet! " <<endl;
                cout << "Turn to another mode" <<endl<<endl;
                mode = 0;
                continue;
            }
            jnt_tra_msg_ = new trajectory_msgs::JointTrajectory;

            for (int i = 0; i < 4; i++)
            {
                std::string joint_name = joint["names"][i].as<std::string>();
                jnt_tra_msg_->joint_names.push_back(joint_name);
               
            }  
            
            calculate_target_radian(nowX, nowY, goalX[0] + width/2, goalY[0] + height/2, width, height, r);
            for(int i = 0; i < 4; i++)
            {
                if(i == 0)
                    step[0] = step[0] - dRadian[0];
                else if(i == 1)   
                    step[1] = step[1] + dRadian[1];
                else if(i == 2)   
                    step[2] = step[2] - dRadian[2];
                else if(i == 3)   
                    step[3] = step[3] + dRadian[3];
            }
            
            time_from_start = findMax(dRadian)/(2 * M_PI*0.25*4);
            
            if(time_from_start  < 0.1)
                time_from_start = 0.1;
            navigationTime[0] = time_from_start;

            trajectory_msgs::JointTrajectoryPoint jnt_tra_point1;    
            for (int i = 0; i < 4; i++)
            {
                jnt_tra_point1.positions.push_back(step[i]);
            }

            jnt_tra_point1.time_from_start.fromSec(time_from_start);
            jnt_tra_msg_->points.push_back(jnt_tra_point1);



            for(int i = 0; i < goalNum - 1; i++ )
            {
                calculate_target_radian(goalX[i] + width/2, goalY[i] + height/2, goalX[i+1] + width/2, goalY[i+1] + height/2, width, height, r);
                for(int i = 0; i < 4; i++)
                {
                    if(i == 0)
                        step[0] = step[0] - dRadian[0];
                    else if(i == 1)   
                        step[1] = step[1] + dRadian[1];
                    else if(i == 2)   
                        step[2] = step[2] - dRadian[2];
                    else if(i == 3)   
                        step[3] = step[3] + dRadian[3];
                }
                
                time_from_start = time_from_start+findMax(dRadian)/(2 * M_PI*0.25*4);
                navigationTime[i+1] = findMax(dRadian)/(2 * M_PI*0.25*4);
                if(time_from_start  < 0.1)
                    time_from_start = 0.1;

                trajectory_msgs::JointTrajectoryPoint jnt_tra_point;    
                for (int i = 0; i < 4; i++)
                {
                    jnt_tra_point.positions.push_back(step[i]);
                }

                jnt_tra_point.time_from_start.fromSec(time_from_start);
                jnt_tra_msg_->points.push_back(jnt_tra_point);

            }

            total_time = time_from_start * 4 + 1.5;
            position_pub.publish(*jnt_tra_msg_);
            nowX = goalX[goalNum -1] + width/2;
            nowY = goalY[goalNum -1] + height/2;
            moving_ = true;
            sleep(total_time);

            
        }

        ros::spinOnce();

        //cout << mode << endl;
        loop_rate.sleep();
    }


    pthread_join(id, NULL);
    
    return 0;
}
