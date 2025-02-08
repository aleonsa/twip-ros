#include "twip_observers/kalman_filter.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

KalmanFilter::KalmanFilter(const std::string &name)
    : Node(name),
      is_first_odom_(true)
{
    // Initialize the state variables
    mean_ = Eigen::Vector3d::Zero();
    variance_ = Eigen::Matrix3d::Identity() * 1000.0;
    motion_variance_ = Eigen::Matrix3d::Identity() * 4.0;
    measurement_variance_ = Eigen::Matrix3d::Identity() * 0.5;
    motion_ = Eigen::Vector3d::Zero();
    last_angles_ = Eigen::Vector3d::Zero();
    imu_angles_ = Eigen::Vector3d::Zero();

    // Subscribers and publishers
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "twip_controller/odom_noisy", 10,
        std::bind(&KalmanFilter::odomCallback, this, _1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/out", 1000,
        std::bind(&KalmanFilter::imuCallback, this, _1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "twip_controller/odom_kalman", 10);
}

void KalmanFilter::odomCallback(const nav_msgs::msg::Odometry &odom)
{
    kalman_odom_ = odom;
    double roll, pitch, yaw;
    quaternionToEuler(odom.pose.pose.orientation, roll, pitch, yaw);

    if (is_first_odom_)
    {
        last_angles_ << roll, pitch, yaw;
        mean_ << roll, pitch, yaw;
        is_first_odom_ = false;
        return;
    }

    Eigen::Vector3d current_angles(roll, pitch, yaw);
    motion_ = current_angles - last_angles_;

    statePrediction();
    measurementUpdate();

    // Actualizar para la siguiente iteración
    last_angles_ = current_angles;

    // Actualizar y publicar el mensaje de odometría filtrado
    geometry_msgs::msg::Quaternion q;
    eulerToQuaternion(mean_(0), mean_(1), mean_(2), q);
    kalman_odom_.pose.pose.orientation = q;
    odom_pub_->publish(kalman_odom_);
}

void KalmanFilter::imuCallback(const sensor_msgs::msg::Imu &imu)
{
    quaternionToEuler(imu.orientation, imu_angles_(0), imu_angles_(1), imu_angles_(2));
}

void KalmanFilter::measurementUpdate()
{
    Eigen::Matrix3d K = variance_ * (variance_ + measurement_variance_).inverse();
    mean_ = mean_ + K * (imu_angles_ - mean_);
    variance_ = (Eigen::Matrix3d::Identity() - K) * variance_;
}

void KalmanFilter::statePrediction()
{
    mean_ = mean_ + motion_;
    variance_ = variance_ + motion_variance_;
}

void KalmanFilter::quaternionToEuler(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw)
{
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);
}

void KalmanFilter::eulerToQuaternion(const double roll, const double pitch, const double yaw, geometry_msgs::msg::Quaternion &q)
{
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    q.x = quat.x();
    q.y = quat.y();
    q.z = quat.z();
    q.w = quat.w();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KalmanFilter>("kalman_filter");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}