#ifndef EXPLORE_PKG__FRONTIER_EXPLORER_HPP_
#define EXPLORE_PKG__FRONTIER_EXPLORER_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <set>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace explore_pkg
{

struct Frontier
{
  std::vector<geometry_msgs::msg::Point> points;
  geometry_msgs::msg::Point centroid;
  double size;
};

class FrontierExplorer
{
public:
  FrontierExplorer();
  ~FrontierExplorer();

  // 检测前沿点
  std::vector<Frontier> detectFrontiers(
    const nav_msgs::msg::OccupancyGrid & map,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double exploration_radius = -1.0  // -1 表示无限制
  );

  // 选择最佳前沿点作为目标
  geometry_msgs::msg::PoseStamped selectBestFrontier(
    const std::vector<Frontier> & frontiers,
    const geometry_msgs::msg::PoseStamped & robot_pose
  );

  // 计算建图完成度
  double calculateExplorationProgress(const nav_msgs::msg::OccupancyGrid & map);

private:
  // 将地图坐标转换为索引
  int mapToIndex(int x, int y, int width) const;

  // 将索引转换为地图坐标
  void indexToMap(int index, int & x, int & y, int width) const;

  // 检查点是否在地图范围内
  bool isValid(int x, int y, int width, int height) const;

  // 检查点是否是前沿点（未知区域与已知区域的边界）
  bool isFrontierPoint(
    const nav_msgs::msg::OccupancyGrid & map,
    int x, int y
  ) const;

  // 使用广度优先搜索聚类前沿点
  std::vector<Frontier> clusterFrontiers(
    const nav_msgs::msg::OccupancyGrid & map,
    const std::vector<std::pair<int, int>> & frontier_points
  );

  // 计算两点之间的欧氏距离
  double distance(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2
  ) const;

  // 检查点是否在探索范围内
  bool isInExplorationRadius(
    const geometry_msgs::msg::Point & point,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double radius
  ) const;

  // 参数
  int min_frontier_size_;  // 最小前沿点数量
  double frontier_distance_threshold_;  // 前沿点聚类距离阈值
};

}  // namespace explore_pkg

#endif  // EXPLORE_PKG__FRONTIER_EXPLORER_HPP_

