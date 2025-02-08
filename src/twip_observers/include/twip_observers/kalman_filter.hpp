#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>

class KalmanFilter : public rclcpp::Node
{
public:
    KalmanFilter(const std::string &name);
    void statePrediction();
    void measurementUpdate();

private:
    void odomCallback(const nav_msgs::msg::Odometry &);
    void imuCallback(const sensor_msgs::msg::Imu &);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // State variables and covariance matrices
    Eigen::Vector3d mean_;                 // [roll, pitch, yaw]
    Eigen::Matrix3d variance_;             // Covariance matrix of the state
    Eigen::Matrix3d motion_variance_;      // Covariance matrix of the motion
    Eigen::Matrix3d measurement_variance_; // Covariance matrix of the meas
    Eigen::Vector3d motion_;               // [delta_roll, delta_pitch, delta_yaw]

    bool is_first_odom_;
    Eigen::Vector3d last_angles_; // Last measured angles
    Eigen::Vector3d imu_angles_;  // Measured angles from the IMU

    nav_msgs::msg::Odometry kalman_odom_;

    void quaternionToEuler(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw);
    void eulerToQuaternion(const double roll, const double pitch, const double yaw, geometry_msgs::msg::Quaternion &q);
};

#endif // KALMAN_FILTER_HPP