#ifndef VIS4EARTH_GRAPH_VISER_DBSCAN_H
#define VIS4EARTH_GRAPH_VISER_DBSCAN_H

#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>
#include <osg/Vec3>
#include <set>
#include <memory>
#include <string>

// 计算欧几里得距离
float calculateDistance(const osg::Vec3 &pos1, const osg::Vec3 &pos2) {
    osg::Vec3 diff = pos1 - pos2;
    return diff.length();
}

// 获取给定点的 epsilon 邻域
std::vector<int> regionQuery(int pointIndex, const std::vector<osg::Vec3> &positions,
                             float epsilon) {
    std::vector<int> neighbors;
    const osg::Vec3 &point = positions[pointIndex];

    // 遍历所有点，找到距离小于 epsilon 的点
    for (int i = 0; i < positions.size(); ++i) {
        if (i != pointIndex && calculateDistance(point, positions[i]) <= epsilon) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

// 判断两个节点是否通过边连接
bool areConnected(int pointA, int pointB,
                  const std::vector<std::pair<std::string, std::string>> &edges,
                  const std::vector<std::string> &nodeIds) {
    std::string idA = nodeIds[pointA];
    std::string idB = nodeIds[pointB];
    for (const auto &edge : edges) {
        if ((edge.first == idA && edge.second == idB) ||
            (edge.first == idB && edge.second == idA)) {
            return true; // 找到连接
        }
    }
    return false; // 没有连接
}

// 查找最近的簇ID
int findNearestCluster(int pointIndex, const std::vector<int> &labels,
                       const std::vector<osg::Vec3> &positions,
                       const std::vector<std::pair<std::string, std::string>> &edges,
                       const std::vector<std::string> &nodeIds) {
    int nearestCluster = -1;
    float minDistance = std::numeric_limits<float>::max();

    // 遍历所有点，找到离该点最近的点，且该点已经被分配了簇，并且有直接连接
    for (int i = 0; i < labels.size(); ++i) {
        if (labels[i] != -1 &&
            areConnected(pointIndex, i, edges, nodeIds)) { // 只考虑有连接且已分配的簇
            float distance = calculateDistance(positions[pointIndex], positions[i]);
            if (distance < minDistance) {
                minDistance = distance;
                nearestCluster = labels[i]; // 最近的簇ID
            }
        }
    }
    return nearestCluster;
}

// DBSCAN 算法：基于点的位置（osg::Vec3）进行聚类
std::vector<int> DBSCAN(const std::vector<osg::Vec3> &positions, float epsilon, int minPts,
                        const std::vector<std::pair<std::string, std::string>> &dbscanedges,
                        const std::vector<std::string> &nodeIds) {
    std::vector<int> labels(positions.size(), -1); // -1 表示未分类
    int clusterId = 0;
    std::vector<bool> visited(positions.size(), false); // 记录每个点是否访问过

    // 遍历所有点
    for (int i = 0; i < positions.size(); ++i) {
        if (visited[i])
            continue; // 跳过已经访问过的点

        visited[i] = true;

        // 找到该点的 epsilon 邻域
        std::vector<int> neighbors = regionQuery(i, positions, epsilon);

        // 如果邻域中的点数小于 minPts，则标记为噪声点
        if (neighbors.size() < minPts) {
            int nearestCluster = findNearestCluster(i, labels, positions, dbscanedges, nodeIds);
            if (nearestCluster != -1) {
                labels[i] = nearestCluster; // 分配到最近的簇
            }
        } else {
            // 否则，形成新的簇
            labels[i] = clusterId;

            // 将邻域内的点逐步添加到簇中
            std::set<int> neighborSet(neighbors.begin(), neighbors.end());

            // 扩展簇
            while (!neighborSet.empty()) {
                auto iter = neighborSet.begin();
                int currentNeighborIndex = *iter;
                neighborSet.erase(iter);

                if (!visited[currentNeighborIndex]) {
                    visited[currentNeighborIndex] = true;

                    // 找到当前点的 epsilon 邻域
                    std::vector<int> neighborNeighbors =
                        regionQuery(currentNeighborIndex, positions, epsilon);
                    if (neighborNeighbors.size() >= minPts) {
                        neighborSet.insert(neighborNeighbors.begin(), neighborNeighbors.end());
                    }
                }

                // 如果该点还未被分配到簇中，则将其添加到当前簇
                if (labels[currentNeighborIndex] == -1) {
                    labels[currentNeighborIndex] = clusterId;
                }
            }
            // 完成当前簇的扩展，准备下一个簇
            clusterId++;
        }
    }
    return labels;
}

#endif