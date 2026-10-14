#pragma once

#include <memory>
#include <vector>
#include <mutex>
#include <string>
#include <cmath>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class GridMap
{
public:
    // 保持和原始 swarm GridMap 相同的只能指针类型 
    using Ptr = std::shared_ptr<GridMap>;

    GridMap() = default;
    ~GridMap() = default;

    // planner_manager 初始化时会调用
    void initMap(rclcpp::Node::SharedPtr node);
    
    //bspline_opt path_searching 过程中会调用
    double getResolution() const;;

    // 发布 2D 膨胀障碍地图，供 RViz 查看规划器实际使用的膨胀层
    void publishInflateMap();

        /*
      原来的 swarm GridMap 使用：
          int getInflateOccupancy(Eigen::Vector3d pos)
      新版本继续保留这个签名，便于 bspline_opt 和 path_searching 直接复用。
      但是内部只使用：
          pos.x()
          pos.y()
    */
   int getInflateOccupancy(const Eigen::Vector3d& pos) const;
    bool getOdomDepthTimeout() const
    {
        return false;
    }

private:
    // map回调
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    // /scan 回调
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // 世界坐标 -> 栅格索引
    bool worldToGrid(double x, double y, int& gx, int& gy) const;

    // 栅格索引 -> 一维数组下标
    int index(int gx, int gy) const;

    //重新分配二维珊格
    void resizeMap(
        double origent_x, 
        double origent_y,
        int width,
        int height,
        double resolution
    );

    // 标记一个栅格为障碍物
    void markOccupied(int gx,int gy);

    // 对 occ_grid_ 做膨胀，结果写入 inflate_grid_
    void rebuildInflate();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    //2D膨胀地图发布
   
    std::string inflate_topic_{"/map_inflate"}; // 膨胀地图发布
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflate_map_pub_;
    rclcpp::TimerBase::SharedPtr inflate_map_timer_;


    // map 回调getInflateOccupancy() 之间的互斥锁，需要保护
    mutable std::mutex mutex_;
    
    // 参数
    std::string map_frame_{"map"};
    std::string map_topic_{"/map"};
    std::string scan_topic_{"/scan"};

    // /map 的元信息
    double resolution_{0.05};
    double origin_x_{0.0};
    double origin_y_{0.0};
    int width_{0};
    int height_{0};

    // 0 表示自由，1 表示占据
    std::vector<uint8_t> occ_grid_;
    std::vector<uint8_t> inflate_grid_;

    // 障碍物膨胀半径，单位米
    double inflate_radius_{0.35};

    // 雷达最大使用距离
    double max_scan_range_{6.0};

    // 每隔多少束雷达点取一个点，减少计算量
    int scan_step_{4};

};