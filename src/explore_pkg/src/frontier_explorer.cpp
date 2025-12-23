#include "explore_pkg/frontier_explorer.hpp"
#include <iostream>
#include <queue>
#include <set>
#include <utility>

namespace explore_pkg
{

FrontierExplorer::FrontierExplorer()
: min_frontier_size_(20),
  frontier_distance_threshold_(0.5)
{
}

FrontierExplorer::~FrontierExplorer()
{
}

int FrontierExplorer::mapToIndex(int x, int y, int width) const
{
  return y * width + x;
}

void FrontierExplorer::indexToMap(int index, int & x, int & y, int width) const
{
  x = index % width;
  y = index / width;
}

bool FrontierExplorer::isValid(int x, int y, int width, int height) const
{
  return x >= 0 && x < width && y >= 0 && y < height;
}

bool FrontierExplorer::isFrontierPoint(
  const nav_msgs::msg::OccupancyGrid & map,
  int x, int y) const
{
  // 检查中心点必须是未知区域 (-1)
  int index = mapToIndex(x, y, map.info.width);
  if (map.data[index] != -1) {
    return false;
  }

  // 检查8邻域中是否有已知区域（0或100）
  int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
  int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};

  for (int i = 0; i < 8; i++) {
    int nx = x + dx[i];
    int ny = y + dy[i];

    if (isValid(nx, ny, map.info.width, map.info.height)) {
      int nindex = mapToIndex(nx, ny, map.info.width);
      if (map.data[nindex] == 0 || map.data[nindex] == 100) {
        return true;  // 找到已知区域，是前沿点
      }
    }
  }

  return false;
}

std::vector<Frontier> FrontierExplorer::clusterFrontiers(
  const nav_msgs::msg::OccupancyGrid & map,
  const std::vector<std::pair<int, int>> & frontier_points)
{
  std::vector<Frontier> frontiers;
  std::set<std::pair<int, int>> visited;

  for (const auto & point : frontier_points) {
    if (visited.find(point) != visited.end()) {
      continue;
    }

    // BFS 聚类
    std::queue<std::pair<int, int>> queue;
    std::vector<geometry_msgs::msg::Point> cluster_points;
    queue.push(point);
    visited.insert(point);

    while (!queue.empty()) {
      auto current = queue.front();
      queue.pop();

      geometry_msgs::msg::Point p;
      p.x = current.first * map.info.resolution + map.info.origin.position.x;
      p.y = current.second * map.info.resolution + map.info.origin.position.y;
      p.z = 0.0;
      cluster_points.push_back(p);

      // 检查8邻域
      int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
      int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};

      for (int i = 0; i < 8; i++) {
        int nx = current.first + dx[i];
        int ny = current.second + dy[i];
        std::pair<int, int> neighbor(nx, ny);

        if (visited.find(neighbor) == visited.end() &&
          isValid(nx, ny, map.info.width, map.info.height))
        {
          // 检查是否是前沿点且距离足够近
          int nindex = mapToIndex(nx, ny, map.info.width);
          if (map.data[nindex] == -1) {
            // 计算距离
            double dist = std::sqrt(
              (nx - current.first) * (nx - current.first) +
              (ny - current.second) * (ny - current.second)) * map.info.resolution;

            if (dist <= frontier_distance_threshold_) {
              queue.push(neighbor);
              visited.insert(neighbor);
            }
          }
        }
      }
    }

    if (static_cast<int>(cluster_points.size()) >= min_frontier_size_) {
      Frontier frontier;
      frontier.points = cluster_points;

      // 计算质心
      double sum_x = 0.0, sum_y = 0.0;
      for (const auto & pt : cluster_points) {
        sum_x += pt.x;
        sum_y += pt.y;
      }
      frontier.centroid.x = sum_x / cluster_points.size();
      frontier.centroid.y = sum_y / cluster_points.size();
      frontier.centroid.z = 0.0;
      frontier.size = cluster_points.size();

      frontiers.push_back(frontier);
    }
  }

  return frontiers;
}

std::vector<Frontier> FrontierExplorer::detectFrontiers(
  const nav_msgs::msg::OccupancyGrid & map,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double exploration_radius)
{
  std::vector<Frontier> frontiers;
  std::vector<std::pair<int, int>> frontier_points;

  // 遍历地图，找到所有前沿点
  for (int y = 0; y < static_cast<int>(map.info.height); y++) {
    for (int x = 0; x < static_cast<int>(map.info.width); x++) {
      if (isFrontierPoint(map, x, y)) {
        // 转换为世界坐标
        geometry_msgs::msg::Point world_point;
        world_point.x = x * map.info.resolution + map.info.origin.position.x;
        world_point.y = y * map.info.resolution + map.info.origin.position.y;
        world_point.z = 0.0;

        // 如果设置了探索范围，检查是否在范围内
        if (exploration_radius > 0.0) {
          if (!isInExplorationRadius(world_point, robot_pose, exploration_radius)) {
            continue;
          }
        }

        frontier_points.push_back(std::make_pair(x, y));
      }
    }
  }

  // 聚类前沿点
  frontiers = clusterFrontiers(map, frontier_points);

  return frontiers;
}

geometry_msgs::msg::PoseStamped FrontierExplorer::selectBestFrontier(
  const std::vector<Frontier> & frontiers,
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (frontiers.empty()) {
    geometry_msgs::msg::PoseStamped empty_pose;
    return empty_pose;
  }

  // 选择距离最近且大小合适的前沿点
  double best_score = -1.0;
  int best_index = -1;

  for (size_t i = 0; i < frontiers.size(); i++) {
    const auto & frontier = frontiers[i];
    double dist = distance(frontier.centroid, robot_pose.pose.position);

    // 评分：考虑距离和前沿点大小
    // 距离越近、前沿点越大，分数越高
    double score = frontier.size / (1.0 + dist);

    if (score > best_score) {
      best_score = score;
      best_index = i;
    }
  }

  if (best_index >= 0) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = robot_pose.header.stamp;
    goal.pose.position = frontiers[best_index].centroid;
    goal.pose.orientation.w = 1.0;  // 朝向可以后续优化
    return goal;
  }

  geometry_msgs::msg::PoseStamped empty_pose;
  return empty_pose;
}

double FrontierExplorer::calculateExplorationProgress(
  const nav_msgs::msg::OccupancyGrid & map)
{
  int total_cells = map.info.width * map.info.height;
  int explored_cells = 0;

  for (int value : map.data) {
    if (value == 0 || value == 100) {  // 已知区域（自由或障碍）
      explored_cells++;
    }
  }

  return static_cast<double>(explored_cells) / total_cells * 100.0;
}

double FrontierExplorer::distance(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2) const
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool FrontierExplorer::isInExplorationRadius(
  const geometry_msgs::msg::Point & point,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double radius) const
{
  double dist = distance(point, robot_pose.pose.position);
  return dist <= radius;
}

}  // namespace explore_pkg

