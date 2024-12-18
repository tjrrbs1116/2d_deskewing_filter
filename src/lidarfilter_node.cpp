#include "lidarfilter_node.hpp"

using namespace std::chrono_literals;

namespace Filter
{

    lidarfilter_node::lidarfilter_node(const rclcpp::NodeOptions &options)
        : Node("lidarfilter_node", options)
    {
        RCLCPP_INFO(get_logger(), "template starting");

        this->declare_parameter("lidar_topic", lidar_topic_name_);
        this->declare_parameter("odom_topic", odom_topic_name_);
        this->declare_parameter<int>("poses_size", poses_size_);
        this->declare_parameter<double>("odom_hz", 20);
        this->declare_parameter<double>("lidar_hz", 10);
        this->declare_parameter<double>("synk_tolerance", 0.005);
        this->declare_parameter<double>("max_velocity", 2.0);
        this->declare_parameter<double>("scan_duration",10.0);
        this->declare_parameter<unsigned int>("scan_point", 300);
        GetParameter();

        RCLCPP_INFO(get_logger(), "lidar_topic: %s", lidar_topic_name_.c_str());
        RCLCPP_INFO(get_logger(), "odom_topic: %s", odom_topic_name_.c_str());
        RCLCPP_INFO(get_logger(), "poses_size: %d", poses_size_);
        RCLCPP_INFO(get_logger(), "odom_hz: %f", odom_hz_);
        RCLCPP_INFO(get_logger(), "lidar_hz: %f", lidar_hz_);
        RCLCPP_INFO(get_logger(), "synk_tolerance: %f", synk_tolerance_);
        RCLCPP_INFO(get_logger(), "max_velocity: %f", max_velocity_);


        subscriber_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = subscriber_cb_group_;

        lidar_sub = create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic_name_, rclcpp::SystemDefaultsQoS(),
            std::bind(&lidarfilter_node::lidarCallback, this, std::placeholders::_1), sub_options);

        odom_sub = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_name_, rclcpp::SystemDefaultsQoS(),
            std::bind(&lidarfilter_node::odomCallback, this, std::placeholders::_1), sub_options);

        deskewed_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("deskewed_scan", 10);

        test_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("test", 10); 

        callback_group_executor_.add_callback_group(subscriber_cb_group_, this->get_node_base_interface());
        callback_group_executor_thread = std::thread([this]()
                                                     { callback_group_executor_.spin(); });

    }

    void lidarfilter_node::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        if(std::abs(time - odom_cb_time - (1.0 / odom_hz_)) > 0.1)
        {
            RCLCPP_ERROR(get_logger(), "odom topic delay error");
        }

        std::lock_guard<mutex_t> lock(access_);  

        odom_poses.push_back(*msg);

        if(odom_poses.size() > poses_size_)
        {
            odom_poses.erase(odom_poses.begin());
        }

        // RCLCPP_INFO(get_logger(), "odom_poses size: %d", odom_poses.size());
    
        odom_cb_time = time;
    }

    void lidarfilter_node::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {

        bool time_sync_error = false;
        double l_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        double l_dt;

        if(msg->time_increment < 1e-9)
        {
            l_dt = (1/scan_duration_) / scan_point_;
        }
        else
        {
            l_dt = msg->time_increment;
        }

        std::vector<double> times;
        std::vector<Eigen::Vector3d> raw_point;


        for(size_t i = 0; i < msg->ranges.size(); i++)
        {
            double t = l_time + l_dt * i;
            double angle = msg->angle_min + i * msg->angle_increment;
            double range = msg->ranges[i];

            if(range < msg->range_min || range > msg->range_max)
            {
                continue;
            }

            Eigen::Vector3d point;
            point << range * std::cos(angle), range * std::sin(angle), 0.0;
            raw_point.push_back(point);
            times.push_back(t);
        }

        if(odom_poses.size() != poses_size_){
            return;
        }

        int idx0 = -1;
        for(size_t p = odom_poses.size()-1; p >= 0; p--)
        {
            double time = odom_poses[p].header.stamp.sec + odom_poses[p].header.stamp.nanosec * 1e-9;

            if(time < times.front())
            {
                idx0 = p;
                break;
            }
        }

        double closest_time_diff = std::numeric_limits<double>::max();
        int idx1 = -1;

        for (size_t p = 0; p < odom_poses.size(); p++)
        {
            double time = odom_poses[p].header.stamp.sec + odom_poses[p].header.stamp.nanosec * 1e-9;
            double time_diff = std::abs(time - times.back());

            if (time_diff < closest_time_diff)
            {
                closest_time_diff = time_diff;
                idx1 = p;
            }
        }

        
        if(idx0 == -1 || idx1 == -1)
        {
        
            RCLCPP_WARN(get_logger(), "Time synk Error");
            time_sync_error = true;
        }
        
        if(closest_time_diff > synk_tolerance_)
        {
            RCLCPP_WARN(get_logger(), 
            "There is no odometry coordinate matching the time of the last LiDAR point "
            "[Check the sink_time_offset_ parameter]");
            time_sync_error = true;
        }
        
        // // precise deskewing
        Eigen::Matrix4d tf0 = odomToTransform(odom_poses[idx0]);
        Eigen::Matrix4d tf1 = odomToTransform(odom_poses[idx1]);

        double odom0_x = tf0(0, 3);
        double odom0_y = tf0(1, 3);
        double odom1_x = tf1(0, 3);
        double odom1_y = tf1(1, 3);

        double tick = (max_velocity_ / lidar_hz_) * (times[idx1] - times[idx0]);


        if (std::sqrt((odom1_x - odom0_x) * (odom1_x - odom0_x) + (odom1_y - odom0_y) * (odom1_y - odom0_y)) > tick)
        {
            RCLCPP_WARN(get_logger(),"The difference in odometry position exceeded the maximum velocity"
            "[Odometry data needs to be checked]");
            time_sync_error = true;
        }

        Eigen::Matrix4d dtf = Eigen::Matrix4d::Identity();  

        if(!time_sync_error)
        {
            dtf = tf0.inverse() * tf1;
        }

        std::vector<Eigen::Vector3d> filtered_points(raw_point.size());

        for(size_t p = 0; p < raw_point.size(); p++)
        {
            double t = times[p];
            double denominator = times[idx1] - times[idx0];
            double alpha = (t-times[idx0])/(times[idx1]-times[idx0]);

            alpha = std::clamp(alpha, 0.0, 1.0);
            if(std::fabs(denominator) < 1e-9)
            {
                dtf = Eigen::Matrix4d::Identity();
            } 

            Eigen::Matrix4d tf = Sophus::interpolate<Sophus::SE3d>(Sophus::SE3d::fitToSE3(Eigen::Matrix4d::Identity()), Sophus::SE3d::fitToSE3(dtf), alpha).matrix();
            filtered_points[p] = tf.block(0,0,3,3)*raw_point[p] + tf.block(0,3,3,1);
        }

        // Convert deskewed points back to LaserScan format
        sensor_msgs::msg::LaserScan deskewed_scan;
        deskewed_scan.header = msg->header;
        deskewed_scan.angle_min = msg->angle_min;
        deskewed_scan.angle_max = msg->angle_max;
        deskewed_scan.angle_increment = msg->angle_increment;
        deskewed_scan.time_increment = msg->time_increment;
        deskewed_scan.scan_time = msg->scan_time;
        deskewed_scan.range_min = msg->range_min;
        deskewed_scan.range_max = msg->range_max;
        deskewed_scan.ranges.resize(msg->ranges.size(), std::numeric_limits<float>::infinity());

        for (size_t i = 0; i < filtered_points.size(); i++)
        {
            double x = filtered_points[i].x();
            double y = filtered_points[i].y();
            double range = std::sqrt(x * x + y * y);
            double angle = std::atan2(y, x);

            // Calculate index in LaserScan data
            int index = std::round((angle - deskewed_scan.angle_min) / deskewed_scan.angle_increment);
            if (index >= 0 && index < static_cast<int>(deskewed_scan.ranges.size()))
            {
                deskewed_scan.ranges[index] = range;
            }
        }

        deskewed_scan_pub_->publish(deskewed_scan);
        test_pub->publish(*msg);
    }

    void lidarfilter_node::GetParameter()
    {
        RCLCPP_INFO(get_logger(), "Getting parameter value");

        if (this->get_parameter("scan_duration", scan_duration_)
        {
            scan_duration_ = scan_duration_;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Parameter not set, using default: %s", scan_duration_);
        }


        if (this->get_parameter("scan_point", scan_point_)
        {
            scan_point_ = scan_point_;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Parameter not set, using default: %s", scan_point_);
        }



        if (this->get_parameter("max_velocity", max_velocity_))
        {
            max_velocity_ = max_velocity_;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Parameter not set, using default: %s", max_velocity_);
        }

        if( this->get_parameter("synk_tolerance", synk_tolerance_))
        {
            synk_tolerance_ = synk_tolerance_;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Parameter not set, using default: %s", synk_tolerance_);
        }


        if (this->get_parameter("lidar_hz", lidar_hz_))
        {
            lidar_hz_ = lidar_hz_;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Parameter not set, using default: %s", lidar_hz_);
        }

        if (this->get_parameter("lidar_topic", lidar_topic_name_))
        {
            lidar_topic_name_ = nav2_util::strip_leading_slash(lidar_topic_name_);
        }
        else
        {
            lidar_topic_name_ = lidar_topic_name_;
            RCLCPP_WARN(get_logger(), "Parameter not set, using default: %s", lidar_topic_name_.c_str());
        }

        if (this->get_parameter("odom_topic", odom_topic_name_))
        {
            odom_topic_name_ = nav2_util::strip_leading_slash(odom_topic_name_);
        }
        else
        {
            odom_topic_name_ = odom_topic_name_;
            RCLCPP_WARN(get_logger(), "Parameter not set, using default: %s", odom_topic_name_.c_str());
        }

        if (this->get_parameter("poses_size", poses_size_))
        {
            if(poses_size_ > 2000)
            {
                poses_size_ = 2000;
                RCLCPP_WARN(get_logger(), "Parameter is too large, using default: %s", poses_size_);
            }
            if(poses_size_ < 10)
            {
                poses_size_ = 10;
                RCLCPP_WARN(get_logger(), "Parameter is too small, using default: %s", poses_size_);
            }
            poses_size_ = poses_size_;
        }

        else
        {
            RCLCPP_WARN(get_logger(), "Parameter not set, using default: %s", poses_size_);
        }

        if(this->get_parameter("odom_hz", odom_hz_))
        {
            odom_hz_ = odom_hz_;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Parameter not set, using default: %s", odom_hz_);
        }
    }

}
