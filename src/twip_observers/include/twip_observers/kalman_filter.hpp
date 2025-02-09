// File: kalman_filter.hpp
#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>

class KalmanFilter : public rclcpp::Node
{
public:
    KalmanFilter(const std::string &name);

private:
    void imuCallback(const sensor_msgs::msg::Imu &msg);
    void updateParameters();
    void predict(const double dt, const Eigen::Vector3d &angular_velocity);
    void update(const Eigen::Vector3d &measurement);
    void normalizeAngles(Eigen::VectorXd &state);                        // Cambiado para aceptar el estado completo
    Eigen::Vector3d normalizeAngleVector(const Eigen::Vector3d &angles); // Nueva función
    void quaternionToEuler(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw);
    void eulerToQuaternion(const double roll, const double pitch, const double yaw, geometry_msgs::msg::Quaternion &q);

    // ROS2 publishers/subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr kalman_pub_;

    // Estado del filtro
    Eigen::VectorXd x_; // Estado [roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate]
    Eigen::MatrixXd P_; // Covarianza del estado
    Eigen::MatrixXd Q_; // Covarianza del ruido del proceso
    Eigen::MatrixXd R_; // Covarianza del ruido de medición

    // Variables de tiempo
    rclcpp::Time last_time_;
    bool first_measurement_;
};

#endif // KALMAN_FILTER_HPP