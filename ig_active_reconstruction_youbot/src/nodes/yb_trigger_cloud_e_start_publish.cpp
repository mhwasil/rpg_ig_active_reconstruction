#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdexcept>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yb_trigger_cloud_e_start_publish");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //Create client for yb_ros_server
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    //ros::Publisher client_mux = nh.advertise<std_msgs::String>("/mcr_perception/mux_pointcloud/event_in", 1);
    //ros::Publisher client_mux_select = nh.advertise<std_msgs::String>("/mcr_perception/mux_pointcloud/select", 1);
    ros::Publisher client_cloud_acc = nh.advertise<std_msgs::String>("/mcr_perception/cloud_accumulator/event_in", 1);
    //ros::Publisher client_cloud_acc_publish = nh.advertise<std_msgs::String>("/mcr_perception/cloud_accumulator/event_in", 1);

    uint numb_subscriber = 0;
    int i = 0;
    std_msgs::String string_msg;
    while (ros::ok())
    {
        switch (i)
        {
        case 0:
            if (numb_subscriber == 0)
            {
                ROS_INFO("Advertising e_start_publish to cloud accumulator topic");
                string_msg.data = "e_start_publish";
                client_cloud_acc.publish(string_msg.data);
                numb_subscriber = client_cloud_acc.getNumSubscribers();
                i = 0;
            }
            else
            {
                std::cout << "Subscribers: " << numb_subscriber << "\n";
                numb_subscriber = 0;
                i = 1;
            }
            break;

        case 1:
            break;
        }

        ros::spinOnce();
    }
}