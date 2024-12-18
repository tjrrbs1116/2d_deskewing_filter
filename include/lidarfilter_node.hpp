#include <mutex>
#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <algorithm>

#include "nav2_util/string_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "rclcpp/logging.hpp"  
#include <sophus/geometry.hpp>
#include <sophus/se3.hpp>
#include <sophus/interpolate.hpp>

#include "struct.hpp"
#include "filter_utils.hpp"


#include <Eigen/Dense>

namespace Filter
{

    class lidarfilter_node : public rclcpp::Node
    
    {
        typedef std::recursive_mutex mutex_t;

        public:
            explicit lidarfilter_node (const rclcpp::NodeOptions &);

        
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
            rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr deskewed_scan_pub_;
            rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr test_pub;
            void timerCallback();
            void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

            mutex_t * getMutex()
            {
                return &access_;
            }



            // about 

            // aobut odom
            std::vector<nav_msgs::msg::Odometry> odom_poses;

            double d_start_time;
            double odom_cb_time;
            double lidar_cb_time;

            //about parameter
            void GetParameter();
            std::string lidar_topic_name_ = "scan";
            std::string odom_topic_name_ = "default_odom";
            double odom_hz_ = 20.;
            double lidar_hz_ = 10.;
            int poses_size_ = 300; 
            double synk_tolerance_ = 0.005;
            double max_velocity_ = 2.0;
            unsigned int scan_point_ = 360;
            double scan_duration_ = 10.0;
            // about callback group
            std::thread callback_group_executor_thread;
            rclcpp::executors::MultiThreadedExecutor callback_group_executor_;
            std_msgs::msg::String current_string;
            rclcpp::CallbackGroup::SharedPtr subscriber_cb_group_;
            rclcpp::CallbackGroup::SharedPtr timer_cb_group_;





        private:
            

        protected:

        mutex_t  access_;

        
    };





}
