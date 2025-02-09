// File: kalman_filter.cpp
#include "twip_observers/kalman_filter.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;

KalmanFilter::KalmanFilter(const std::string &name)
    : Node(name), first_measurement_(true)
{
    // Inicializar estado: [roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate]
    x_ = Eigen::VectorXd::Zero(6);

    // Matriz de covarianza inicial
    P_ = Eigen::MatrixXd::Identity(6, 6);
    P_.topLeftCorner(3, 3) *= 0.001;     // Confianza alta en ángulos iniciales
    P_.bottomRightCorner(3, 3) *= 0.005; // Confianza alta en velocidades iniciales

    // Ruido del proceso - Basado en el ruido del IMU
    Q_ = Eigen::MatrixXd::Identity(6, 6);
    // Para ángulos: integración del ruido de velocidad angular
    const double dt = 1.0 / 100.0;                        // 100 Hz
    const double ang_vel_variance = std::pow(2e-4, 2);    // varianza = stddev²
    Q_.topLeftCorner(3, 3) *= ang_vel_variance * dt * dt; // Integración del ruido
    Q_.bottomRightCorner(3, 3) *= ang_vel_variance;       // Ruido directo en velocidades

    // Ruido de medición
    R_ = Eigen::MatrixXd::Identity(3, 3);
    // El ruido en orientación viene de la integración de la velocidad angular
    const double orientation_stddev = 2e-4 * std::sqrt(dt); // Aproximación simple
    R_ *= std::pow(orientation_stddev, 2);

    // Ajuste fino por eje
    // R_(2, 2) *= ; // Yaw típicamente más ruidoso

    // ROS2 parameters para sintonización en tiempo real
    declare_parameter("process_noise_scale", 1.0);
    declare_parameter("measurement_noise_scale", 1.0);

    // Timer para actualizar parámetros
    auto timer_callback =
        [this]() -> void
    {
        this->updateParameters();
        RCLCPP_INFO(this->get_logger(), "Updated Kalman filter parameters: Q = %f, R = %f",
                    this->Q_(0, 0), this->R_(0, 0));
    };

    create_wall_timer(std::chrono::seconds(1), timer_callback);

    // ROS2 setup con QoS específico para IMU
    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/out", qos, std::bind(&KalmanFilter::imuCallback, this, _1));

    kalman_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/filtered", 10);
}

void KalmanFilter::updateParameters()
{
    // Factores de escala para ajuste fino
    double process_scale = get_parameter("process_noise_scale").as_double();
    double measurement_scale = get_parameter("measurement_noise_scale").as_double();

    Q_ *= process_scale;
    R_ *= measurement_scale;
}

void KalmanFilter::predict(const double dt, const Eigen::Vector3d &angular_velocity)
{
    // Matriz de transición de estado
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F.topRightCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3) * dt;

    // Factor de suavizado para velocidades angulares
    const double alpha = 1; // Aumentado para más suavizado

    // Predicción del estado con suavizado
    x_.head(3) += x_.tail(3) * dt;
    x_.tail(3) = alpha * x_.tail(3) + (1 - alpha) * angular_velocity;

    // Actualizar covarianza - Escalado por dt
    P_ = F * P_ * F.transpose() + Q_ * dt;
}

void KalmanFilter::imuCallback(const sensor_msgs::msg::Imu &imu)
{
    // Obtener tiempo actual
    rclcpp::Time current_time = this->get_clock()->now();

    // Extraer mediciones
    Eigen::Vector3d measured_angles;
    quaternionToEuler(imu.orientation,
                      measured_angles(0),
                      measured_angles(1),
                      measured_angles(2));

    Eigen::Vector3d angular_velocity(imu.angular_velocity.x,
                                     imu.angular_velocity.y,
                                     imu.angular_velocity.z);

    if (first_measurement_)
    {
        x_.head(3) = measured_angles;
        x_.tail(3) = angular_velocity;
        last_time_ = current_time;
        first_measurement_ = false;
        return;
    }

    // Calcular dt real
    double dt = (current_time - last_time_).seconds();
    if (dt > 0.02)
    { // Skip si el dt es muy grande (> 20ms)
        last_time_ = current_time;
        return;
    }
    last_time_ = current_time;

    // Predicción y actualización
    predict(dt, angular_velocity);
    update(measured_angles);

    // Publicar resultado
    auto filtered_imu = imu;
    geometry_msgs::msg::Quaternion q;
    eulerToQuaternion(x_(0), x_(1), x_(2), q);

    filtered_imu.header.stamp = current_time;
    filtered_imu.orientation = q;
    filtered_imu.angular_velocity.x = x_(3);
    filtered_imu.angular_velocity.y = x_(4);
    filtered_imu.angular_velocity.z = x_(5);

    // Copiar covarianzas
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> orientation_cov(filtered_imu.orientation_covariance.data());
    orientation_cov = P_.topLeftCorner(3, 3);

    kalman_pub_->publish(filtered_imu);
}

void KalmanFilter::update(const Eigen::Vector3d &measurement)
{
    // Matriz de medición (solo observamos los ángulos)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
    H.leftCols(3) = Eigen::MatrixXd::Identity(3, 3);

    // Calcular innovación usando la nueva función de normalización
    Eigen::Vector3d current_angles = x_.head(3);
    Eigen::Vector3d innovation = normalizeAngleVector(measurement - current_angles);

    // Ganancia de Kalman
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    // Actualizar estado y covarianza
    x_ += K * innovation;
    P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
}

Eigen::Vector3d KalmanFilter::normalizeAngleVector(const Eigen::Vector3d &angles)
{
    Eigen::Vector3d normalized = angles;
    for (int i = 0; i < 3; i++)
    {
        while (normalized(i) > M_PI)
        {
            normalized(i) -= 2 * M_PI;
        }
        while (normalized(i) < -M_PI)
        {
            normalized(i) += 2 * M_PI;
        }
    }
    return normalized;
}

void KalmanFilter::normalizeAngles(Eigen::VectorXd &state)
{
    // Normalizar solo los primeros tres elementos (ángulos)
    for (int i = 0; i < 3; i++)
    {
        while (state(i) > M_PI)
        {
            state(i) -= 2 * M_PI;
        }
        while (state(i) < -M_PI)
        {
            state(i) += 2 * M_PI;
        }
    }
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
