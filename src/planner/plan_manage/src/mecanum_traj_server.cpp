#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include "bspline_opt/uniform_bspline.h"
#include "traj_utils/msg/bspline.hpp"

using ego_planner::UniformBspline;

class MecanumTrajServer : public rclcpp::Node
{
    public:
    MecanumTrajServer():Node("mecanum_traj_server")
    {
        //参数声明
        declare_parameter("max_linear_velocity",1.5);
        declare_parameter("max_angular_velocity",1.5);
        declare_parameter("yaw_gain",1.5);
        declare_parameter("position_kp_x",1.0);
        declare_parameter("position_kp_y",1.0);
        declare_parameter("goal_position_tolerance",1.0);

        get_parameter("max_linear_velocity",max_v_);
        get_parameter("max_angular_velocity",max_w_);
        get_parameter("yaw_gain",yaw_gain_);
        get_parameter("position_kp_x", position_kp_x_);
        get_parameter("position_kp_y", position_kp_y_);
        get_parameter("goal_position_tolerance", goal_position_tolerance_);

        // 订阅 Ego 输出的 B 样条轨迹
        // topic:
        // planning/bspline
        bspline_sub_ = create_subscription<traj_utils::msg::Bspline>(
            "planning/bspline",
            10,
            std::bind(&MecanumTrajServer::bsplineCallback,
                        this,
                        std::placeholders::_1        
                )
        );
        /*订阅车辆当前位姿
        这里需要当前 yaw，用来计算车体坐标系下的速度。
        topic:odom_world
        */ 
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom_world",
            10,
            std::bind(&MecanumTrajServer::odomCallback,
                      this,
                      std::placeholders::_1));

        // 发布麦轮底盘速度
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10
        );
        timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MecanumTrajServer::controlCallback,this)
        );
        RCLCPP_INFO(get_logger(), "[MecanumTrajServer] ready."); 
        
    }




    private:

        /*
     * normalizeAngle()
     * 作用：把角度归一化到 [-pi, pi]。
     * 为什么：desired_yaw 和 current_yaw 的差值可能超过 pi。
     * 如果直接使用，会让车错误地转大圈。
     */
    static double normalizeAngle(double angle)
    {
        while (angle > M_PI)
        {
            angle -= 2.0 * M_PI;
        }

        while (angle < -M_PI)
        {
            angle += 2.0 * M_PI;
        }

        return angle;
    }

     /*
     * bsplineCallback()
     * 作用：
     *   接收 Ego 发布的 traj_utils/Bspline，并重新构造 UniformBspline。
     * 为什么：
     *   Ego 输出的消息里存的是 B 样条控制点和 knots。我们需要把它恢复成可以连续求值的位置轨迹和速度轨迹。
     */
    void bsplineCallback(const traj_utils::msg::Bspline::SharedPtr msg)
    {
        if(msg->pos_pts.empty())
        {
            return;
        }
        //1.把消息中的控制点转成 Eigen 矩阵
        // pos_pts 每列是一个三维控制点。对二维小车来说，z 强制为 0。
        Eigen::MatrixXd pos_pts(3,msg->pos_pts.size());
        Eigen::VectorXd knots(msg->knots.size());
//size_t 是什么意思？我有定义这个？
        for (size_t i = 0;i <msg->pos_pts.size();++i)
        {
            pos_pts(0,i) = msg->pos_pts[i].x;
            pos_pts(1, i) = msg->pos_pts[i].y;

            //真正的2D规划 z固定为0
            pos_pts(2,i) = 0.0;

        }
        //2.读取kont 向量
        for (size_t i = 0;i<msg->knots.size();++i)
        {
            knots(i) = msg->knots[i];
        }

        //3.构造B样条
        UniformBspline pos_traj(pos_pts, msg->order, 0.1);
        pos_traj.setKnot(knots);
        //4.保存位置轨迹 速度轨迹 加速度轨迹
        traj_.clear();
        traj_.push_back(pos_traj);
        traj_.push_back(pos_traj.getDerivative());
        traj_.push_back(traj_[1].getDerivative());

        //5.保存轨迹起始时间和总时长
        start_time_ = rclcpp::Time(msg->start_time,now().get_clock_type());
        traj_duration_ = traj_[0].getTimeSum();

        receive_traj_ = true;
    }

   /*
     * odomCallback()
     * 作用：获取当前车辆的 yaw。
     * 为什么：
     *  麦轮速度是车体坐标系下的。Ego 轨迹速度是 map 坐标系下的。 
     * 要把 map 速度转换成车体速度，必须知道当前车头朝向。
     */

     void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
     {
        // 保存实际位置，位置闭环必须使用它
        current_pos_(0) = msg->pose.pose.position.x;
        current_pos_(1) = msg->pose.pose.position.y;
        current_pos_(2) = 0.0;

        //保持当前车头朝向
        current_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
        have_odom_ = true;
     }
    /*
     * controlCallback()
     * 作用：周期性采样轨迹，计算 /cmd_vel。
     * 控制逻辑：
     *   1. 取出当前时刻的速度向量
     *   2. 由速度方向计算期望车头朝向
     *   3. 计算当前车头与期望方向的偏差
     *   4. 把 map 速度变成车体速度
     *   5. 发布 vx、vy、wz
     */
    void controlCallback()
    {
        geometry_msgs::msg::Twist cmd;

        // 1. 还没有轨迹或还没有里程计，发送零速度
        if (!receive_traj_ || !have_odom_)
        {
            cmd_pub_->publish(cmd);
            return;
        }

        // 2. 计算轨迹开始后的相对时间
        const double t_cur = (now() - start_time_).seconds();

        // 时间还没到轨迹起点，发送零速度
        if (t_cur < 0.0)
        {
            cmd_pub_->publish(cmd);
            return;
        }

        // 3. 轨迹求值时间限制在 [0, traj_duration_]
        const double t_eval =
            std::clamp(t_cur, 0.0, traj_duration_);

        // 4. 取期望位置和期望速度
        Eigen::Vector3d p_des =
            traj_[0].evaluateDeBoorT(t_eval);

        Eigen::Vector3d v_des =
            traj_[1].evaluateDeBoorT(t_eval);

        // 5. 轨迹时间结束后不要立刻停车。
        //    继续以轨迹终点作为期望位置，用位置误差修正到终点。
        if (t_cur >= traj_duration_)
        {
            p_des = traj_[0].evaluateDeBoorT(traj_duration_);
            v_des.setZero();
        }

        // 6. 计算世界坐标系下的位置误差
        Eigen::Vector2d e_world(
            p_des.x() - current_pos_.x(),
            p_des.y() - current_pos_.y());

        // 7. 世界坐标系误差转车体坐标系
        //    e_body = R(-yaw) * e_world
        const double c = std::cos(current_yaw_);
        const double s = std::sin(current_yaw_);

        Eigen::Vector2d e_body(  //在计算什么？
            c * e_world.x() + s * e_world.y(),
            -s * e_world.x() + c * e_world.y());

        // 8. 期望速度从世界坐标系转车体坐标系
        Eigen::Vector2d v_world(
            v_des.x(),
            v_des.y());

        Eigen::Vector2d v_body(
            c * v_world.x() + s * v_world.y(),
            -s * v_world.x() + c * v_world.y());

        // 9. 速度前馈 + 位置误差反馈
        Eigen::Vector2d v_cmd =
            v_body +
            Eigen::Vector2d(
                position_kp_x_ * e_body.x(),
                position_kp_y_ * e_body.y());

        // 10. 限制总平移速度
        if (v_cmd.norm() > max_v_)
        {
            v_cmd = v_cmd.normalized() * max_v_;
        }

        cmd.linear.x = v_cmd.x();
        cmd.linear.y = v_cmd.y();

        // 11. 计算期望车头方向
        //     有速度时朝向速度方向
        //     低速或终点附近时朝向位置误差方向
        Eigen::Vector2d dir_vec;

        if (v_world.norm() > 0.05)
        {
            dir_vec = v_world;
        }
        else
        {
            dir_vec = e_world;
        }

        // 如果方向向量有效，计算航向误差
        if (dir_vec.norm() > 1e-4)
        {
            const double desired_yaw =
                std::atan2(dir_vec.y(), dir_vec.x());

            const double yaw_error =
                tf2NormalizeAngle(desired_yaw - current_yaw_);

            cmd.angular.z = std::clamp(
                yaw_gain_ * yaw_error,
                -max_w_,
                max_w_);
        }
        else
        {
            cmd.angular.z = 0.0;
        }

        // 12. 真正到达终点后停车
        // 这里使用实际位置误差，不再只看轨迹时间
        if (t_cur >= traj_duration_ &&
            e_world.norm() < goal_position_tolerance_)
        {
            geometry_msgs::msg::Twist stop_cmd;
            cmd_pub_->publish(stop_cmd);

            receive_traj_ = false;
            return;
        }

        // 13. 发布最终速度
        cmd_pub_->publish(cmd);
    }

    // B样条轨迹订阅
    rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr bspline_sub_;
    //车辆状态订阅
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    //麦轮底盘速度发布
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    //控制定时器
    rclcpp::TimerBase::SharedPtr timer_;
    //位置 速度 加速度轨迹
    std::vector<UniformBspline> traj_;

    //轨迹起始时间
    rclcpp::Time start_time_;
    //定义参数，并给予初始值
    //轨迹总时长
    double traj_duration_{0.0};
    //当前车头yaw
    double current_yaw_{0.0};

    //是否收到轨迹
    bool receive_traj_{false};

    //是否收到odom
    bool have_odom_{false};

        // 参数
    double max_v_{0.5};
    double max_w_{1.5};
    double yaw_gain_{1.5};
    //当前小车的实际位置，来自/odom_world
    Eigen::Vector3d current_pos_{0.0,0.0,0.0};
    //位置误差反馈系数 Kp越大，纠正偏斜越快，但是太大会产生抖动
    double position_kp_x_{1.0};
    double position_kp_y_{1.0};
    //终点位置容差
    double goal_position_tolerance_{0.05};
};


int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MecanumTrajServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}