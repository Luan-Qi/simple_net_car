#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <algorithm>

class PIDController {
public:
  PIDController(double kp, double ki, double kd, 
               double i_max, double out_min, double out_max)
    : kp_(kp), ki_(ki), kd_(kd),
      i_max_(i_max), out_min_(out_min), out_max_(out_max) 
  {
    reset();
  }

  double compute(double setpoint, double measured_value, rclcpp::Time current_time) {
    // 计算误差
    const double error = setpoint - measured_value;
    
    // 处理首次调用
    if (last_time_.nanoseconds() == 0) {
      last_time_ = current_time;
      last_error_ = error;
      return std::clamp(kp_ * error, out_min_, out_max_);
    }

    // 计算时间差
    const double dt = (current_time - last_time_).seconds();
    if (dt <= 0) {  // 无效时间差
      return std::clamp(kp_ * error, out_min_, out_max_);
    }

    // 计算比例项
    const double proportional = kp_ * error;

    // 计算积分项（带限幅）
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -i_max_, i_max_);
    const double integral = ki_ * integral_;

    // 计算微分项
    const double derivative = kd_ * (error - last_error_) / dt;

    // 计算总输出
    double output = proportional + integral + derivative;
    output = std::clamp(output, out_min_, out_max_);

    // 保存状态
    last_error_ = error;
    last_time_ = current_time;

    return output;
  }

  void reset() {
    integral_ = 0.0;
    last_error_ = 0.0;
    last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void set_gains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

private:
  // PID参数
  double kp_;
  double ki_;
  double kd_;
  
  // 限幅参数
  double i_max_;
  double out_min_;
  double out_max_;
  
  // 状态变量
  double integral_;
  double last_error_;
  rclcpp::Time last_time_;
};

class VelocityControlNode : public rclcpp::Node {
public:
  VelocityControlNode() : Node("velocity_controller") {
    // 参数声明
    declare_parameters();
    
    // 创建PID控制器
    initialize_pid();
    
    // 创建订阅和定时器
    target_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "target_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_velocity_ = msg->linear.x;
        pid_->reset();  // 目标变化时重置控制器
      });
    
    current_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "current_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        current_velocity_ = msg->linear.x;
      });
    
    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      [this]() { this->control_loop(); });
    
    control_pub_ = create_publisher<std_msgs::msg::Float64>("control_output", 10);
  }

private:
  void declare_parameters() {
    declare_parameter("kp", 0.5);
    declare_parameter("ki", 0.0);
    declare_parameter("kd", 0.1);
    declare_parameter("i_max", 1.0);
    declare_parameter("out_min", -1.0);
    declare_parameter("out_max", 1.0);
  }

  void initialize_pid() {
    const double kp = get_parameter("kp").as_double();
    const double ki = get_parameter("ki").as_double();
    const double kd = get_parameter("kd").as_double();
    const double i_max = get_parameter("i_max").as_double();
    const double out_min = get_parameter("out_min").as_double();
    const double out_max = get_parameter("out_max").as_double();
    
    pid_ = std::make_unique<PIDController>(
      kp, ki, kd, i_max, out_min, out_max);
  }

  void control_loop() {
    const auto now = this->now();
    const double output = pid_->compute(
      target_velocity_, 
      current_velocity_,
      now);
    
    std_msgs::msg::Float64 msg;
    msg.data = output;
    control_pub_->publish(msg);
  }

  std::unique_ptr<PIDController> pid_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr current_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  double target_velocity_ = 0.0;
  double current_velocity_ = 0.0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityControlNode>());
  rclcpp::shutdown();
  return 0;
}