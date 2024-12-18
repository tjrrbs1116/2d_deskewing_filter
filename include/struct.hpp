
#include <Eigen/Dense>

struct ODOM_POSE_2D
{
    double t;

    Eigen::Vector3d pose; 
    Eigen::Vector3d vel; 

    ODOM_POSE_2D()
    {
        t = 0;
        pose.setZero();
        vel.setZero();
    }

    ODOM_POSE_2D(const ODOM_POSE_2D& _in)
    {
        t = _in.t;
        pose = _in.pose;
        vel = _in.vel;
    }

    ODOM_POSE_2D& operator=(const ODOM_POSE_2D& _in)
    {
        t = _in.t;
        pose = _in.pose;
        vel = _in.vel;
        return *this;
    }
};

struct ODOM_POSE_3D
{
    double t; 
    Eigen::Vector3d position; 
    Eigen::Quaterniond orientation; 

    Eigen::Vector3d linear_velocity; 
    Eigen::Vector3d angular_velocity;


    ODOM_POSE_3D()
        : t(0),
          position(Eigen::Vector3d::Zero()),
          orientation(Eigen::Quaterniond::Identity()),
          linear_velocity(Eigen::Vector3d::Zero()),
          angular_velocity(Eigen::Vector3d::Zero())
    {}

    ODOM_POSE_3D(const ODOM_POSE_3D& _in)
        : t(_in.t),
          position(_in.position),
          orientation(_in.orientation),
          linear_velocity(_in.linear_velocity),
          angular_velocity(_in.angular_velocity)
    {}

    ODOM_POSE_3D& operator=(const ODOM_POSE_3D& _in)
    {
        t = _in.t;
        position = _in.position;
        orientation = _in.orientation;
        linear_velocity = _in.linear_velocity;
        angular_velocity = _in.angular_velocity;
        return *this;
    }
};