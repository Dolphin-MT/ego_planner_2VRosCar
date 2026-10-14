#include "plan_env/grid_map.h"

#include <algorithm>
#include <limits>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

void GridMap::initMap(rclcpp::Node::SharedPtr node)
{
    node_ = node;

    //参数声明
    node->declare_parameter("map_frame", map_frame_);
    node->declare_parameter("map_topic", map_topic_);
    node->declare_parameter("scan_topic", scan_topic_);
    node->declare_parameter("inflate_radius", inflate_radius_);
    node->declare_parameter("max_scan_range", max_scan_range_);
    node->declare_parameter("scan_step", scan_step_);
    node->declare_parameter("inflate_topic",inflate_topic_);
    // 参数读取
    node->get_parameter("map_frame", map_frame_);
    node->get_parameter("map_topic", map_topic_);
    node->get_parameter("scan_topic", scan_topic_);
    node->get_parameter("inflate_radius", inflate_radius_);
    node->get_parameter("max_scan_range", max_scan_range_);
    node->get_parameter("scan_step", scan_step_);
    node->get_parameter("inflate_topic",inflate_topic_);

    // TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 订阅 slam_toolbox 的 /map
    map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_,
         rclcpp::QoS(1).transient_local(),
         std::bind(&GridMap::mapCallback, this, std::placeholders::_1));
    //订阅二维雷达
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&GridMap::scanCallback, this, std::placeholders::_1));
    
    //For地图膨胀
    inflate_map_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
        inflate_topic_,
        rclcpp::QoS(1).transient_local().reliable());
    //创建2hz的定时器
    inflate_map_timer_ = node->create_wall_timer(
        200ms,
        std::bind(&GridMap::publishInflateMap,this)
    );

    RCLCPP_INFO(node->get_logger(),
    "[GridMap2D] map_topic=%s scan_topic=%s inflate_topic=%s map_frame=%s resolution=%.3f inflate=%.3f",
    map_topic_.c_str(),
    scan_topic_.c_str(),
    inflate_topic_.c_str(),
    map_frame_.c_str(),
    resolution_,
    inflate_radius_);
}

double GridMap::getResolution() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return resolution_;
}

/*发布二维障碍物膨胀功能*/
void GridMap::publishInflateMap()
{
    if(!inflate_map_pub_)
    {
        return;
    }

    nav_msgs::msg::OccupancyGrid msg;
    // 先加锁复制地图数据。
    // 不要在锁内直接 publish，避免地图回调和 ROS 发布互相阻塞。
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // 地图还没有初始化
        if(width_<= 0 || height_ <= 0)
        {
            return;
        }

        const int cell_num = width_ * height_;

        // inflate_grid_ 大小必须和地图尺寸一致
        if(static_cast<int>(inflate_grid_.size()) != cell_num)
        {
            return;
        }

        // 填充消息头
        msg.header.frame_id = map_frame_;
        // 填充地图元信息
        msg.info.resolution = static_cast<float>(resolution_);
        msg.info.width = static_cast<uint32_t>(width_);
        msg.info.height = static_cast<uint32_t>(height_);

        msg.info.origin.position.x = origin_x_;
        msg.info.origin.position.y = origin_y_;
        msg.info.origin.position.z = 0.0;

        // 2D 地图没有旋转，四元数使用单位旋转
        msg.info.origin.orientation.x = 0.0;
        msg.info.origin.orientation.y = 0.0;
        msg.info.origin.orientation.z = 0.0;
        msg.info.origin.orientation.w = 1.0;

        // 填充占据数据
        // inflate_grid_ 中：
        //   0 -> 可通行
        //   1 -> 膨胀障碍物
        //
        // OccupancyGrid 中：
        //   0   -> 自由
        //   100 -> 占据
        msg.data.resize(cell_num);

        for(int i = 0;i< cell_num;++i)
        {
            msg.data[i] = inflate_grid_[i] ? 100 :0;
        }
    }
    // 时间戳必须放在锁外面取，避免持锁时间过长
    msg.header.stamp = node_->now();
     // 发布膨胀地图
    inflate_map_pub_->publish(msg);


}



/*
 * worldToGrid()
 *
 * 作用：
 *   把世界坐标 (x, y) 转成地图栅格坐标 (gx, gy)。
 *
 * 为什么：
 *   A* 和 B 样条都在世界坐标下工作，
 *   但底层障碍物存储是二维数组，需要先转成栅格索引。
 *
 * 公式：
 *   gx = floor((x - origin_x) / resolution)
 *   gy = floor((y - origin_y) / resolution)
 */
bool GridMap::worldToGrid(double x,double y,int& gx,int& gy) const
{
    if (width_ <= 0 || height_ <= 0|| resolution_ <= 0.0)
    {
        return false;
    }

    gx = static_cast<int>(std::floor((x- origin_x_) / resolution_));
    gy = static_cast<int>(std::floor((y- origin_y_) / resolution_));
    if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_)
    {
        return false;
    }
    return true;
}

int GridMap::index(int gx,int gy) const
{
    return gy * width_ + gx;
}

void GridMap::resizeMap(double origin_x,
                        double origin_y,
                        int width,
                        int height,
                        double resolution)
{
    origin_x_ = origin_x;
    origin_y_ = origin_y;
    width_ = width;
    height_ = height;
    resolution_ = resolution;

    occ_grid_.assign(width_ * height_,0);
    inflate_grid_.assign(width_ * height_,0);
}

void GridMap::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

    const auto& info = msg->info;

    // 重新分配栅格地图
    resizeMap(
        info.origin.position.x,
        info.origin.position.y,
        static_cast<int>(info.width),
        static_cast<int>(info.height),
        info.resolution
    );
    const size_t cell_num = static_cast<size_t>(width_) * static_cast<size_t>(height_);
    if (msg->data.size() < cell_num)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[GridMap2D] OccupancyGrid data size mismatch: got %zu expected %zu",
                    msg->data.size(),
                    cell_num);
        return;       
    }

    for (size_t i = 0;i < cell_num;++i)
    {
        const int8_t value = msg->data[i];

        /*ROS中 
        100 = occupied
        0 = free
        -1 = unknown
        */
        if(value == 100)
        {
            occ_grid_[i] = 1;
        }
        else
        {
            occ_grid_[i] = 0;
        }

    }
    // 重建膨胀栅格
    rebuildInflate();
}

/*
 * scanCallback()
 * 作用：
 *   1. 把 LaserScan 中的每个点从 laser 坐标系变换到 map 坐标系
 *   2. 只用 x、y，把它转换成栅格坐标
 *   3. 标记为障碍物
 *
 * 为什么：
 *   /map 更新比较慢，不能反映最近出现的障碍物。
 *   /scan 频率高，可以补充实时障碍物。
 *
 * 注意：
 *   z 坐标完全忽略。
 *   雷达高度只参与 laser -> map 的 TF 变换，
 *   不参与规划维度。
 */
void GridMap::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

     // 如果 /map 还没有接收到，就还没有地图尺寸，不能处理 scan
    if (width_ <= 0 ||height_ <= 0)
    {
        return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;

    try
    {
        // 把 laser 坐标系下的点变换到 map 坐标系
        tf_msg = tf_buffer_->lookupTransform(
            map_frame_,
            msg->header.frame_id,
            msg->header.stamp,
            rclcpp::Duration::from_seconds(0.1)
        );
    }

    catch (const std::exception&)
    {
        //TF还没准备好，就直接返回
        return;
    }
    const int step = std::max(1,scan_step_);

    for (size_t i = 0;i<msg->ranges.size();i += step)
    {
        const float r = msg->ranges[i];
 
        // 无效距离
        if(!std::isfinite(r))
        {
            continue;
        }
        // 超出雷达有效量程
        if(r <msg->range_min || r > msg->range_max)
        {
            continue;
        }
         // 超过我们设定的最大使用距离
        if (r> max_scan_range_)
        {
            continue;
        }
        // 计算该激光束的角度
        const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;

        geometry_msgs::msg::PointStamped p_laser;
        geometry_msgs::msg::PointStamped p_map;

        // laser 坐标系下的点
        p_laser.header = msg->header;
        p_laser.point.x = static_cast<double>(r) * std::cos(angle);
        p_laser.point.y = static_cast<double>(r) * std::sin(angle);
        // 2D 雷达本身在同一个平面内，这里 z 只作为占位
        p_laser.point.z = 0.0;
        // laser -> map
        tf2::doTransform(p_laser, p_map, tf_msg);

        int gx = 0;
        int gy = 0;

        if(!worldToGrid(p_map.point.x,p_map.point.y,gx,gy))
        {
            continue;
        }

        markOccupied(gx,gy);

    }
    // 新扫描点加入后，重新膨胀障碍物
    rebuildInflate();
}

/*
 * markOccupied()
 * 作用：
 *   把一个栅格标记为障碍物。
 * 为什么：
 *   mapCallback() 和 scanCallback() 都需要修改障碍物栅格。
 *   把它们统一到一个函数里，方便做越界检查。
 */
void GridMap::markOccupied(int gx,int gy)
{
    //越界检查
    if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_)
    {
        return;
    }  
    const int id = index(gx,gy);
    if(id < 0|| id >= static_cast<int>(occ_grid_.size()))
    {
        return;
    }

    // 标记为障碍物
    occ_grid_[id] = 1;
}
/*
 * rebuildInflate()
 * 作用：
 *   对 occ_grid_ 做膨胀，结果写入 inflate_grid_。
 * 为什么：
 *   Ego 不能只检查“障碍物中心点”，
 *   还要保证整个车体不会撞到障碍物。
 * 例如：
 *   车体半径 0.3 m
 *   栅格分辨率 0.05 m
 * 那么每个障碍物周围 0.3 m 的栅格都要被标记为膨胀障碍物。
 */
void GridMap::rebuildInflate()
{
    //先复制一份原始障碍物
    inflate_grid_ = occ_grid_;
    
    const int radius_cells = 
        static_cast<int>(std::ceil(inflate_radius_ / resolution_));


    for (int gy = 0;gy < height_;++gy)
    {
        for (int gx = 0;gx< width_;++gx)
        {
            const int id = index(gx, gy);

            if (id < 0 || id >= static_cast<int>(occ_grid_.size()))
            {
                continue;
            }

            // 只有原始障碍物才需要膨胀
            if (occ_grid_[id] == 0)
            {
                continue;
            }

            // 以当前障碍物为中心，向外膨胀
            for (int dy = -radius_cells; dy <= radius_cells; ++dy)
            {
                for (int dx = -radius_cells; dx <= radius_cells; ++dx)
                {
                    // 圆形膨胀
                    if (dx * dx + dy * dy > radius_cells * radius_cells)
                    {
                        continue;
                    }

                    const int nx = gx + dx;
                    const int ny = gy + dy;

                    if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_)
                    {
                        continue;
                    }

                    inflate_grid_[index(nx, ny)] = 1;
                }
            }
        }
    }
}

/*
 * getInflateOccupancy()
 * 作用：
 *   给 bspline_opt 和 path_searching 使用。
 * 参数：
 *   pos 是 Eigen::Vector3d。
 * 但是：
 *   我们完全忽略 pos.z()，只使用 pos.x() 和 pos.y()。
 * 返回：
 *   1 = 该点被膨胀障碍物占据
 *   0 = 该点自由
 * 为什么超出地图返回 1：
 *   规划器不应该把路径规划到未知地图边界之外。
 *   超出地图范围按障碍物处理更安全。
 */
 int GridMap::getInflateOccupancy(const Eigen::Vector3d& pos) const
 {
    std::lock_guard<std::mutex> lock(mutex_);

    int gx = 0;
    int gy = 0;

    if (!worldToGrid(pos.x(), pos.y(), gx, gy))
    {
        return 1; // 超出地图范围按障碍物处理
    }

    const int id = index(gx, gy);
    if (id < 0 || id >= static_cast<int>(inflate_grid_.size()))
    {
        return 1; // 越界按障碍物处理
    }

    return inflate_grid_[id] ? 1 : 0;
 }

