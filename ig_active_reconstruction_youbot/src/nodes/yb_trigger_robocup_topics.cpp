#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdexcept>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "yb_trigger_robocup_topics");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //Create client for yb_ros_server
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  
  ros::Publisher client_mux = nh.advertise<std_msgs::String>("/mcr_perception/mux_pointcloud/event_in", 1);
  ros::Publisher client_mux_select = nh.advertise<std_msgs::String>("/mcr_perception/mux_pointcloud/select", 1);
  ros::Publisher client_cloud_acc = nh.advertise<std_msgs::String>("/mcr_perception/cloud_accumulator/event_in", 1);
  //ros::Publisher client_cloud_acc_publish = nh.advertise<std_msgs::String>("/mcr_perception/cloud_accumulator/event_in", 1);

  uint numb_subscriber = 0;
  int i = 1;
  std_msgs::String string_msg;
  while(ros::ok())
  {
    switch(i)
    {
      case 0:
        if (numb_subscriber == 0)
        {
          ROS_INFO_STREAM("Advertising e_trigger to mcr_perception");
          string_msg.data = "e_trigger";
          client_mux.publish(string_msg);
          numb_subscriber = client_mux.getNumSubscribers();
          std::cout << "Subscribers: " << numb_subscriber << "\n";
          i = 0;
        } 
        else {
          std::cout << "Subscribers e_trigger: " << numb_subscriber << "\n";
          numb_subscriber = 0;
          i = 1;
        }
        break;

      case 1:
        if (numb_subscriber == 0)
        {
          ROS_INFO("Advertising depth camera topic to mcr_perception");
          string_msg.data = "/arm_cam3d/depth_registered/points";
          client_mux_select.publish(string_msg.data);
          numb_subscriber = client_mux_select.getNumSubscribers();
          i = 1;
        }
        else
        {
          std::cout << "Subscribers topic: " << numb_subscriber << "\n";
          numb_subscriber = 0;
          i = 2;
        }
        break;
      
      case 2:
        if (numb_subscriber == 0)
        {
          ROS_INFO("Advertising e_start to cloud accumulator topic");
          string_msg.data = "e_start";
          client_cloud_acc.publish(string_msg.data);
          numb_subscriber = client_cloud_acc.getNumSubscribers();
          i = 2;
        }
        else
        {
          std::cout << "Subscribers e_start: " << numb_subscriber << "\n";
          numb_subscriber = 0;
          i = 3;
        }
        break;
      
      case 3:
        if (numb_subscriber < 1)
        {
          ROS_INFO("Advertising e_start_publish to cloud accumulator topic");
          string_msg.data = "e_start_publish";
          client_cloud_acc.publish(string_msg.data);
          numb_subscriber = client_cloud_acc.getNumSubscribers();
          i = 3;
        }
        else
        {
          std::cout << "Subscribers e_start_publish: " << numb_subscriber << "\n";
          numb_subscriber = 0;
          i = 4;
        }
        break;
      
      case 4:
        break;
    }

    std::cout << "Subscribers cloud_acc event_in: " << client_cloud_acc.getNumSubscribers() <<"\n";
    sleep(2);
    
    ros::spinOnce();
  }

}