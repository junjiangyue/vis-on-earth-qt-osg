#ifndef VIS4EARTH_GRAPH_VISER_NODE_LAYOUT_H
#define VIS4EARTH_GRAPH_VISER_NODE_LAYOUT_H

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
            updateLayout(layoutedGraph, 0.1);
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
            updateLayout(layoutedGraph, 0.1);
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
        double dx, dy, dz, f, fx, fy, fz, d, dsq;
        auto nodes = graph.getNodes();
        auto nodesNotMove = graph.getNodesNotMove();
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

                dx = n1.pos.x - n2.pos.x;
                dy = n1.pos.y - n2.pos.y;
                dz = n1.pos.z - n2.pos.z;
                d = distance(n1.pos, n2.pos);
                dsq = (d > graph.getEdgeLength()) ? graph.getEdgeLength() : d;
                f = graph.getRepulsion() * 128 * 128 / std::pow(dsq, 2);
                fx = f * dx / d;
                fy = f * dy / d;
                fz = f * dz / d;
                n1.force += glm::vec3(fx, fy, fz);
            }
        }
        graph.setNodes(nodes);
    }

    void updateSpring(VIS4Earth::Graph &graph) {
        double dx, dy, dz, f, fx, fy, fz, d, dsq;
        auto nodes = graph.getNodes();
        auto nodePairs = graph.getNodePairs();
        auto nodesNotMove = graph.getNodesNotMove();
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
                if (nodePairs.find({i, j}) != nodePairs.end()) {
                    dx = n2.pos.x - n1.pos.x;
                    dy = n2.pos.y - n1.pos.y;
                    dz = n2.pos.z - n1.pos.z;
                    d = distance(n1.pos, n2.pos);
                    dsq = (d > graph.getEdgeLength()) ? d - graph.getEdgeLength() : 0;
                    f = graph.getSpring() * dsq;
                    fx = f * dx / d;
                    fy = f * dy / d;
                    fz = f * dz / d;
                    n1.force += glm::vec3(fx, fy, fz);
                }
            }
        }
        graph.setNodes(nodes);
    }

    void updateCenterSpring(VIS4Earth::Graph &graph) {
        double dx, dy, dz, f, fx, fy, fz, d;
        auto nodes = graph.getNodes();
        auto nodesNotMove = graph.getNodesNotMove();
        for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
            std::string n1_id = std::to_string(i);
            if (graph.getNodesRestriction() && nodesNotMove.find(i) != nodesNotMove.end()) {
                continue;
            }
            Node &n1 = nodes[n1_id];
            glm::vec3 center = graph.getGravitationCenter();
            d = distance(n1.pos, center);
            f = graph.getAttraction() * d;
            dx = n1.pos.x - center.x;
            dy = n1.pos.y - center.y;
            dz = n1.pos.z - center.z;
            fx = f * dx / d;
            fy = f * dy / d;
            fz = f * dz / d;
            n1.force -= glm::vec3(fx, fy, fz);
        }
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
            minPos = glm::min(minPos, node.second.pos);
            maxPos = glm::max(maxPos, node.second.pos);
        }
        for (int i = 0; i < nodes.size(); i++) {
            std::string n1_id = std::to_string(i);
            // calculate acceleration
            nodes[n1_id].acc = nodes[n1_id].force;
            // calculate velocity
            glm::vec3 deltaVelocity = nodes[n1_id].acc;

            deltaVelocity *= deltaT;
            nodes[n1_id].vel += deltaVelocity;
            // calculate position change
            glm::vec3 translation = nodes[n1_id].vel;
            translation *= deltaT;
            // TODO: updata pos if new pos is in the restricted area
            Area restriction = graph.getRestrictedArea();
            // 计算新的坐标
            glm::vec3 newPos = nodes[n1_id].pos + translation;

            // 检查新坐标是否在restrictArea内
            if (!graph.getNodesRestriction() || !posWithin(restriction, newPos)) {
                // 确保 newPos 在原始范围内
                if (minPos.x <= newPos.x && newPos.x <= maxPos.x && minPos.y <= newPos.y &&
                    newPos.y <= maxPos.y && minPos.z <= newPos.z && newPos.z <= maxPos.z) {
                    // 更新坐标
                    nodes[n1_id].pos = newPos;
                }
            }
            glm::vec3 zero = glm::vec3(0, 0, 0);
            nodes[n1_id].force = zero;
            nodes[n1_id].acc = zero;
            nodes[n1_id].vel = zero;
        }
        graph.setNodes(nodes);
        updateAllEdges(graph);
    }
};

} // namespace VIS4Earth

#endif // VIS4EARTH_GRAPH_VISER_NODE_LAYOUT_H
