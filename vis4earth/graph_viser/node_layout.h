#ifndef VIS4EARTH_GRAPH_VISER_NODE_LAYOUT_H
#define VIS4EARTH_GRAPH_VISER_NODE_LAYOUT_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <vis4earth/graph_viser/graph.h>

namespace VIS4Earth {

class NodeLayouter {
  public:
    struct LayoutParam {
        double repulsion;
        double spring_k;
        double attraction;
        double edgeLength;
        int Iteration;
    };

  private:
    VIS4Earth::Graph origGraph; // 原图，全程不修改
    VIS4Earth::Graph layoutedGraph;

  public:
    VIS4Earth::Graph getLayoutedGraph() const { return layoutedGraph; }

    void setGraph(const VIS4Earth::Graph &graph) {
        origGraph = graph;
        layoutedGraph = graph;
    }

    void setParameter(const LayoutParam &param) {
        layoutedGraph.setGraphLayoutParams(param.repulsion, param.spring_k, param.attraction,
                                           param.edgeLength);
    }

    void layout(int iterations) {
        for (int i = 0; i < iterations; ++i) {
            updateLayout(layoutedGraph, 0.05);
        }
    }

    void restrictedLayout(const Area &restrictedArea, int iterations) {
        layoutedGraph.enableNodeRestriction(restrictedArea);
        std::unordered_set<int> fixedNodes;
        auto nodes = layoutedGraph.getNodes();

        for (int k = 0; k < nodes.size(); k++) {
            std::string id = std::to_string(k);
            auto it = nodes.find(id);
            if (it != nodes.end()) {
                if (it->second.pos.x >= restrictedArea.leftBound &&
                    it->second.pos.x <= restrictedArea.rightBound &&
                    it->second.pos.y <= restrictedArea.upperBound &&
                    it->second.pos.y >= restrictedArea.bottomBound) {
                    fixedNodes.insert(k);
                }
                std::cout << "x: " << it->second.pos.x << ", y: " << it->second.pos.y << std::endl;
            } else {
                std::cout << "Key not found." << std::endl;
            }

            // if (PosWithin(restrictedArea,nodes[id].pos)) {
            //     uset.insert(k);
            // }
        }

        layoutedGraph.setNodesRestriction(fixedNodes);

        for (int i = 0; i < iterations; ++i) {
            updateLayout(layoutedGraph, 0.05);
        }
    }

    bool posWithin(const Area &restrictedArea, const glm::vec3 &pos) const {
        return (pos.x >= restrictedArea.leftBound && pos.x <= restrictedArea.rightBound &&
                pos.y <= restrictedArea.upperBound && pos.y >= restrictedArea.bottomBound);
    }

    void updateAllEdges(VIS4Earth::Graph &graph) {
        auto nodes = graph.getNodes();
        auto edges = graph.getEdges();
        for (auto &edge : edges) {
            edge.start = nodes[edge.sourceLabel].pos;
            edge.end = nodes[edge.targetLabel].pos;
            if (!edge.subdivs.empty()) {
                edge.subdivs[0] = Edge::center(edge.start, edge.end);
            }
        }
        graph.setEdges(edges);
    }

    void updateRepulsion(VIS4Earth::Graph &graph) {
        double dx, dy, dz, f, fx, fy, fz, d;
        auto nodes = graph.getNodes();
        auto nodesNotMove = graph.getNodesNotMove();
        double maxDistance = graph.getEdgeLength() * 2; // 限制排斥力作用范围为边长的两倍

        for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
            std::string n1_id = std::to_string(i);

            // 如果节点受到限制，不进行移动
            if (graph.getNodesRestriction() && nodesNotMove.find(i) != nodesNotMove.end()) {
                continue;
            }

            Node &n1 = nodes[n1_id];

            for (int j = 0; j < static_cast<int>(nodes.size()); ++j) {
                if (i == j)
                    continue;

                std::string n2_id = std::to_string(j);
                Node &n2 = nodes[n2_id];

                // 计算节点之间的距离
                dx = n1.pos.x - n2.pos.x;
                dy = n1.pos.y - n2.pos.y;
                dz = n1.pos.z - n2.pos.z;
                d = distance(n1.pos, n2.pos);

                // 如果距离超出最大范围，则忽略排斥力的计算
                if (d > maxDistance)
                    continue;

                // 计算排斥力，排斥力与距离的平方成反比，避免过于靠近
                double dsq = std::max(d * d, graph.getEdgeLength()); // 确保不会除以零
                f = graph.getRepulsion() * 128 * 128 / dsq;          // 调整排斥系数

                // 计算每个方向上的排斥力分量
                fx = f * dx / d;
                fy = f * dy / d;
                fz = f * dz / d;

                // 施加排斥力
                n1.force += glm::vec3(fx, fy, fz); // n1 受到 n2 的排斥
            }
        }

        // 更新计算后的力
        graph.setNodes(nodes);
    }

    void updateSpring(VIS4Earth::Graph &graph) {
        double dx, dy, dz, f, fx, fy, fz, d, dsq;
        double targetEdgeLength = graph.getEdgeLength();
        double minForceThreshold = 0.01; // 最小力阈值
        auto nodes = graph.getNodes();
        auto nodePairs = graph.getNodePairs();
        auto nodesNotMove = graph.getNodesNotMove();
        // 显式欧拉法
        for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
            std::string n1_id = std::to_string(i);
            if (graph.getNodesRestriction() && nodesNotMove.find(i) != nodesNotMove.end()) {
                continue;
            }
            for (int j = 0; j < static_cast<int>(nodes.size()); ++j) {
                if (i == j)
                    continue;

                std::string n2_id = std::to_string(j);
                Node &n1 = nodes[n1_id];
                Node &n2 = nodes[n2_id];

                // 只对存在边的节点对施加弹簧力
                if (nodePairs.find({i, j}) != nodePairs.end()) {
                    dx = n2.pos.x - n1.pos.x;
                    dy = n2.pos.y - n1.pos.y;
                    dz = n2.pos.z - n1.pos.z;
                    d = distance(n1.pos, n2.pos); // 当前节点距离

                    // 计算距离差
                    dsq = (d > targetEdgeLength) ? (d - targetEdgeLength) : (targetEdgeLength - d);

                    // 保证施加的力不会过小
                    f = graph.getSpring() * dsq;
                    if (f < minForceThreshold) {
                        f = minForceThreshold;
                    }

                    // 施加弹簧力
                    fx = f * dx / d;
                    fy = f * dy / d;
                    fz = f * dz / d;

                    // 更新节点的力
                    n1.force += glm::vec3(fx, fy, fz);
                    n2.force -= glm::vec3(fx, fy, fz); // 相反方向作用力
                }
            }
        }
        graph.setNodes(nodes);
    }

    void updateCenterSpring(VIS4Earth::Graph &graph) {
        auto nodes = graph.getNodes();
        auto nodesNotMove = graph.getNodesNotMove();
        int n = nodes.size();

        // 初始化 Eigen 矩阵和向量
        Eigen::VectorXd b(n * 3); // 目标力向量（每个节点的吸引力）
        Eigen::VectorXd x(n * 3); // 待求解节点位置
        Eigen::SparseMatrix<double> A(n * 3, n * 3); // 稀疏矩阵表示的力矩阵

        b.setZero(); // 初始化 b 为 0
        x.setZero(); // 初始化 x 为 0

        std::vector<Eigen::Triplet<double>> coefficients; // 用于高效构造稀疏矩阵 A

        glm::vec3 center = graph.getGravitationCenter(); // 获取重心
        double attraction = graph.getAttraction();       // 吸引力系数

        // 构造力矩阵和目标向量，同时保留 n1.force 的更新
        for (int i = 0; i < n; ++i) {
            std::string n1_id = std::to_string(i);
            if (graph.getNodesRestriction() && nodesNotMove.find(i) != nodesNotMove.end()) {
                // 对静止节点，直接将其位置固定在当前点
                x(i * 3 + 0) = nodes[n1_id].pos.x;
                x(i * 3 + 1) = nodes[n1_id].pos.y;
                x(i * 3 + 2) = nodes[n1_id].pos.z;
                continue;
            }

            Node &n1 = nodes[n1_id];

            // 计算与重心的距离和方向
            glm::vec3 delta = n1.pos - center;
            double d = glm::length(delta);
            if (d < 1e-5) {
                d = 1e-5; // 避免除以零
            }

            // 吸引力大小
            double f = attraction * d * d;

            // 更新 n1.force
            glm::vec3 force;
            force.x = static_cast<float>(-f * (delta.x / d));
            force.y = static_cast<float>(-f * (delta.y / d));
            force.z = static_cast<float>(-f * (delta.z / d));
            n1.force += force; // 更新节点力

            // 目标力向量 b
            b(i * 3 + 0) += n1.force.x;
            b(i * 3 + 1) += n1.force.y;
            b(i * 3 + 2) += n1.force.z;

            // 力矩阵对角线系数（自身引力）
            double coeff = attraction;
            for (int dim = 0; dim < 3; ++dim) {
                int i_dim = i * 3 + dim;
                coefficients.emplace_back(i_dim, i_dim, coeff);
            }
        }

        // 将 coefficients 转化为稀疏矩阵 A
        A.setFromTriplets(coefficients.begin(), coefficients.end());

        // 使用共轭梯度法求解 Ax = b
        Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> solver;
        solver.setMaxIterations(1000); // 最大迭代次数
        solver.setTolerance(1e-6);     // 收敛条件
        solver.compute(A);
        if (solver.info() != Eigen::Success) {
            throw std::runtime_error("Error: Unable to factorize matrix A in CG solver!");
        }
        x = solver.solve(b);
        if (solver.info() != Eigen::Success) {
            throw std::runtime_error("Error: Unable to solve Ax = b in CG solver!");
        }

        // 将更新后的节点返回到图中
        graph.setNodes(nodes);
    }

    void updateLayout(VIS4Earth::Graph &graph, double deltaT) {
        updateRepulsion(graph);
        updateSpring(graph);
        updateCenterSpring(graph);
        glm::vec3 minPos = glm::vec3(std::numeric_limits<float>::max());
        glm::vec3 maxPos = glm::vec3(std::numeric_limits<float>::lowest());

        auto nodes = graph.getNodes();
        // 计算所有节点的最小和最大坐标
        for (const auto &node : nodes) {
            minPos = glm::vec3(30.0, -20.0, 0.0);
            maxPos = glm::vec3(60.0, 20.0, 0.0);
        }
        for (int i = 0; i < nodes.size(); i++) {
            std::string n1_id = std::to_string(i);
            // calculate acceleration
            nodes[n1_id].acc = nodes[n1_id].force;
            glm::vec3 newPos = nodes[n1_id].pos; // 新计算的坐标
            // 改进欧拉法 - 中点计算
            glm::vec3 midVel = nodes[n1_id].vel +
                               (nodes[n1_id].acc * static_cast<float>(deltaT) * 0.5f); // 中间速度
            glm::vec3 midPos = nodes[n1_id].pos +
                               (nodes[n1_id].vel * static_cast<float>(deltaT) * 0.5f); // 中间位置

            // 使用中点值更新速度和位置
            nodes[n1_id].vel += nodes[n1_id].acc * static_cast<float>(deltaT);
            newPos += midVel * static_cast<float>(deltaT);

            const int maxIterations = 100; // 最大迭代次数
            int iterationCount = 0;
            Area restriction = graph.getRestrictedArea();
            glm::vec3 translation = (newPos - nodes[n1_id].pos) * 0.5f; // 定义并缩小位移
            newPos = nodes[n1_id].pos + translation;
            // 检查新坐标是否在restrictArea内
            if (!graph.getNodesRestriction() || !posWithin(restriction, newPos)) {
                while (!(minPos.x <= newPos.x && newPos.x <= maxPos.x && minPos.y <= newPos.y &&
                         newPos.y <= maxPos.y)) {
                    translation *= 0.5f;
                    newPos = nodes[n1_id].pos + translation;
                    iterationCount++;
                    if (iterationCount >= maxIterations || glm::length(translation) < 1e-5f) {
                        std::cerr << "Warning: Exceeded max iterations or translation too small!"
                                  << std::endl;
                        // 直接限制在范围内，退出
                        newPos.x = glm::clamp(newPos.x, minPos.x, maxPos.x);
                        newPos.y = glm::clamp(newPos.y, minPos.y, maxPos.y);
                        newPos.z = glm::clamp(newPos.z, minPos.z, maxPos.z);
                        break;
                    }
                }
                nodes[n1_id].pos = newPos;
            }

            // 清零力和加速度，防止累积
            nodes[n1_id].force = glm::vec3(0.0f, 0.0f, 0.0f);
            nodes[n1_id].acc = glm::vec3(0.0f, 0.0f, 0.0f);

            // 速度衰减处理
            nodes[n1_id].vel *= 0.1f;
        }
        graph.setNodes(nodes);
        updateAllEdges(graph);
    }
};

} // namespace VIS4Earth

#endif // VIS4EARTH_GRAPH_VISER_NODE_LAYOUT_H
