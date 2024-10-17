#ifndef LOUVAIN_ALGORITHM_H
#define LOUVAIN_ALGORITHM_H

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

// 定义边结构，不再需要，因为我们直接使用 std::pair<std::string, std::string>

// 定义图的邻接表，使用 unordered_map
using AdjacencyList = std::unordered_map<std::string, std::unordered_map<std::string, float>>;

// 计算图的总权重
double calculateTotalWeight(const AdjacencyList &graph) {
    double totalWeight = 0;
    for (const auto &node : graph) {
        for (const auto &neighbor : node.second) {
            totalWeight += neighbor.second;
        }
    }
    return totalWeight / 2.0; // 无向图，每条边计算两次
}

// 模块度增益的计算
double modularityGain(const std::string &node, const std::string &community,
                      const AdjacencyList &graph,
                      const std::unordered_map<std::string, std::string> &communityMap,
                      const std::unordered_map<std::string, double> &communityWeights,
                      double totalWeight) {
    double ki_in = 0; // 节点 node 和社区内其他节点的连接权重
    double sum_in = communityWeights.at(community); // 社区的总权重
    double ki = 0;                                  // 节点 node 的总权重

    for (const auto &neighbor : graph.at(node)) {
        ki += neighbor.second; // 节点 node 的总度数
        if (communityMap.at(neighbor.first) == community) {
            ki_in += neighbor.second; // 节点连接到的邻居是否在同一个社区
        }
    }

    double deltaQ = (ki_in / totalWeight) - (ki * sum_in) / (2.0 * totalWeight * totalWeight);
    return deltaQ;
}

// 第一阶段：局部移动
bool firstPhase(AdjacencyList &graph, std::unordered_map<std::string, std::string> &communityMap,
                std::unordered_map<std::string, double> &communityWeights, double totalWeight) {
    bool improvement = false;
    bool moved = true;

    while (moved) {
        moved = false;

        for (const auto &nodePair : graph) {
            const std::string &node = nodePair.first;
            const std::string &currentCommunity = communityMap[node];

            // 计算节点邻居所属的社区
            std::unordered_map<std::string, double> neighborCommunities;
            for (const auto &neighbor : graph[node]) {
                const std::string &neighborCommunity = communityMap[neighbor.first];
                neighborCommunities[neighborCommunity] += neighbor.second;
            }

            // 移出当前社区
            communityWeights[currentCommunity] -= nodePair.second.size();
            communityMap[node] = "";

            // 找到增益最大的社区
            std::string bestCommunity = currentCommunity;
            double maxGain = -std::numeric_limits<double>::infinity();
            for (const auto &communityPair : neighborCommunities) {
                const std::string &candidateCommunity = communityPair.first;
                double gain = modularityGain(node, candidateCommunity, graph, communityMap,
                                             communityWeights, totalWeight);

                if (gain > maxGain) {
                    maxGain = gain;
                    bestCommunity = candidateCommunity;
                }
            }

            // 将节点加入最佳社区
            communityMap[node] = bestCommunity;
            communityWeights[bestCommunity] += nodePair.second.size();

            if (bestCommunity != currentCommunity) {
                moved = true;
                improvement = true;
            }
        }
    }
    return improvement;
}

// 第二阶段：社区合并，构建新的简化图
AdjacencyList
aggregateCommunities(const AdjacencyList &graph,
                     const std::unordered_map<std::string, std::string> &communityMap) {
    AdjacencyList newGraph;
    for (const auto &nodePair : graph) {
        const std::string &node = nodePair.first;
        const std::string &community1 = communityMap.at(node);

        for (const auto &neighborPair : nodePair.second) {
            const std::string &neighbor = neighborPair.first;
            const std::string &community2 = communityMap.at(neighbor);

            if (community1 != community2) {
                newGraph[community1][community2] += neighborPair.second;
                newGraph[community2][community1] += neighborPair.second;
            }
        }
    }
    return newGraph;
}

// Louvain 算法函数，输入边和权重，输出社区标签
std::vector<int> Louvain(const std::vector<std::pair<std::string, std::string>> &edges,
                         const std::vector<float> &weights) {
    // 构建邻接表 AdjacencyList
    AdjacencyList graph;
    for (size_t i = 0; i < edges.size(); ++i) {
        const auto &edge = edges[i];
        graph[edge.first][edge.second] = weights[i];
        graph[edge.second][edge.first] = weights[i]; // 无向图，双向边
    }

    // 计算图的总权重
    double totalWeight = calculateTotalWeight(graph);

    // 初始化：每个节点是自己的社区
    std::unordered_map<std::string, std::string> communityMap;
    std::unordered_map<std::string, double> communityWeights;

    for (const auto &node : graph) {
        communityMap[node.first] = node.first;
        communityWeights[node.first] = 0;

        for (const auto &neighbor : graph[node.first]) {
            communityWeights[node.first] += neighbor.second;
        }
    }

    // 第一阶段和第二阶段的模块度优化
    bool improvement = true;
    while (improvement) {
        // 第一阶段：节点的局部移动
        improvement = firstPhase(graph, communityMap, communityWeights, totalWeight);

        // 第二阶段：社区合并
        graph = aggregateCommunities(graph, communityMap);
    }

    // 将社区映射转为 clusterLabels
    std::vector<int> clusterLabels;
    std::unordered_map<std::string, int> communityToCluster;
    int clusterId = 0;

    for (const auto &nodePair : communityMap) {
        const std::string &node = nodePair.first;
        const std::string &community = nodePair.second;

        // 如果社区还没有被分配 clusterId，分配一个新的 clusterId
        if (communityToCluster.find(community) == communityToCluster.end()) {
            communityToCluster[community] = clusterId++;
        }

        clusterLabels.push_back(communityToCluster[community]);
    }

    return clusterLabels;
}

#endif // LOUVAIN_ALGORITHM_H