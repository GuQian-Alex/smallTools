#include<moveit/move_group_interface/move_group_interface.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>
#include<moveit_msgs/DisplayTrajectory.h>
#include<moveit_msgs/DisplayRobotState.h>
#include<moveit_msgs/AttachedCollisionObject.h>
#include<moveit_msgs/CollisionObject.h>
#include<iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include<visualization_msgs/Marker.h>
#include<string>


using namespace std;

#define PI_CONST 3.141592653589793238462643383279502884
char a[20];
int flag;
double setp_size;
vector<double> RPY;
visualization_msgs::Marker P;


void pathplanning(moveit::planning_interface::MoveGroupInterface& group,moveit::planning_interface::MoveGroupInterface::Plan& my_plan){
            double x=0;
            double y=0;
            double z=0;
            double YAW=0;
            double PITCH=0;
            double ROLL=0;
            cout<<"请输入x坐标："<<endl;
            cin>>x;
            cout<<"请输入y坐标："<<endl;
            cin>>y;
            cout<<"请输入z坐标："<<endl;
            cin>>z;
            cout<<"请输入Yaw坐标："<<endl;
            cin>>YAW;
            cout<<"请输入Pitch坐标："<<endl;
            cin>>PITCH;
            cout<<"请输入Roll坐标："<<endl;
            cin>>ROLL;
            cout<<"目标输入完成！开始计算！";
            tfScalar yaw,pitch,roll;
            yaw=YAW; pitch=PITCH; roll=ROLL;
            cout<<"欧拉角rpy("<<roll<<","<<pitch<<","<<yaw<<")";
            tf::Quaternion q;
            q.setRPY(roll,pitch,yaw); //将roll, pitch,yas赋给四元数后，可以得到转换后的四元数
            std::cout<<"，转化到四元数q:"<<"("<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w()<<")"<<std::endl;
            geometry_msgs::Pose target_pose1;
            target_pose1.orientation.w=q.w();
            target_pose1.orientation.x=q.x();
            target_pose1.orientation.y=q.y();
            target_pose1.orientation.z=q.z();
            target_pose1.position.x=x;
            target_pose1.position.y=y;
            target_pose1.position.z=z;
            group.setPoseTarget(target_pose1);
            moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
            ROS_INFO("Visualizing plan 1 (pose goal) %s", success.val? "":"FAILED");
            cout<<"是否执行规划？ 是请输入3，否请按任意健  "<<endl;
            int flag2;
            cin>>flag2;
             if(flag2==3){
                 group.move();
             }
}

void printPose(moveit::planning_interface::MoveGroupInterface& group){
            std::cout<<"Current Position: "<< group.getCurrentPose()<<std::endl;
}
void printRPY(moveit::planning_interface::MoveGroupInterface& group){
                  std::cout<<"Current Position: "<< group.getCurrentPose()<<std::endl;//显示当前位置
                  RPY= group.getCurrentRPY();
                  cout<<"Current RPY:"<<"Roll= "<<RPY[0]<<"Pitch="<<RPY[1]<<"Yaw="<<RPY[2]<<endl;
}

void microMove(moveit::planning_interface::MoveGroupInterface& group,moveit::planning_interface::MoveGroupInterface::Plan& my_plan){
             cout<<"请输入微动的步长："<<endl;
             cin>>setp_size;
             geometry_msgs::PoseStamped PoseS;
             PoseS=group.getCurrentPose();

              geometry_msgs::Pose target_pose1;
              target_pose1.orientation.w=PoseS.pose.orientation.w;
              target_pose1.orientation.x=PoseS.pose.orientation.x;
              target_pose1.orientation.y=PoseS.pose.orientation.y;
              target_pose1.orientation.z=PoseS.pose.orientation.z;
              target_pose1.position.x=PoseS.pose.position.x+setp_size;
              target_pose1.position.y=PoseS.pose.position.y;
              target_pose1.position.z=PoseS.pose.position.z;
              group.setPoseTarget(target_pose1);
              moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
              ROS_INFO("Visualizing plan 1 (pose goal) %s", success.val? "":"FAILED");
              group.move();


}

void MarkerViz(moveit::planning_interface::MoveGroupInterface& group,ros::NodeHandle& node_handle){
             ros::Publisher pub= node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    
             P.header.stamp=ros::Time::now();
             P.header.frame_id="base_link";
             P.color.r=1.0;
             P.color.g=1.0;
             P.color.b=1.0;
             P.color.a=1.0;
             P.scale.x=0.1;
             P.scale.y=0.05;
             P.scale.z=0.05;
             P.lifetime=ros::Duration();
             P.type=visualization_msgs::Marker::ARROW;
             P.action=visualization_msgs::Marker::ADD;
             P.id=0;
         
            P.pose=group.getPoseTarget().pose;

            pub.publish(P);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "moveit_goal_set");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(2.0);
    cout<<"please input the move group name:"<<endl;
    cin>>a;
    string mgName=a;
    moveit::planning_interface::MoveGroupInterface group("mgName");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1,true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit::core::RobotState start_state(*group.getCurrentState());
    std::cout<<"Current Position: "<< group.getCurrentPose()<<std::endl;//显示当前位置
     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
     while(ros::ok())
         {

                std::cout<<"功能列表：1、路径规划----------2、获取当前RPY -----------3、微动-------------4、显示目标位姿箭头---------------5、打印位姿----------0、退出 "<<std::endl;
                std::cin>>flag;
                      switch(flag){
                               case 1:
                                      pathplanning(group,my_plan);
                                      break;
                               case 2:
                                      printRPY(group);
                                       break;
                              case 3:
                                      microMove(group,my_plan);
                                      break;
                              case 4:
                                      MarkerViz(group,node_handle);
                                     break;
                              case 5:
                                     printPose(group);
                                      break;
                              case 0:
                                     return 0;
                             default:
                                     break;
                  }
            }
   
    return 0;
}
