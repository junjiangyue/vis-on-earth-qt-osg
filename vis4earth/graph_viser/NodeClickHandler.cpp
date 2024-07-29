#include "NodeClickHandler.h"
#include "VIS4Earth.h" // 包含 PerGraphParam 定义

NodeClickHandler::NodeClickHandler(std::map<std::string, osg::ShapeDrawable *> &nodesMap,
                                   osgViewer::Viewer *viewer,
                                   VIS4Earth::GraphRenderer::PerGraphParam *graphParam)
    : nodesMap(nodesMap), viewer(viewer), graphParam(graphParam) {}

bool NodeClickHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH &&
        ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
        float x = ea.getX();
        float y = ea.getY();
        if (viewer) {
            osgUtil::LineSegmentIntersector::Intersections intersections;
            if (viewer->computeIntersections(x, y, intersections)) {
                for (const auto &intersection : intersections) {
                    osg::NodePath &nodePath = intersection.nodePath;
                    for (auto *node : nodePath) {
                        auto *shape = dynamic_cast<osg::ShapeDrawable *>(node->asDrawable());
                        if (shape) {
                            auto it = std::find_if(
                                nodesMap.begin(), nodesMap.end(),
                                [shape](const std::pair<std::string, osg::ShapeDrawable *> &pair) {
                                    return pair.second == shape;
                                });
                            if (it != nodesMap.end()) {
                                const std::string &nodeId = it->first;
                                if (collapsedNodes.find(nodeId) != collapsedNodes.end()) {
                                    expandNode(nodeId);
                                } else {
                                    collapseNode(nodeId);
                                }
                                return true;
                            }
                        }
                    }
                }
            }
        }
    }
    return false;
}

void NodeClickHandler::collapseNode(const std::string &nodeId) {
    auto neighbors = getNeighbors(nodeId);
    for (const auto &neighbor : neighbors) {
        auto neighborNeighbors = getNeighbors(neighbor);
        for (const auto &nn : neighborNeighbors) {
            if (nn != nodeId) {
                setEdgeVisible(nodeId, nn, true); // 添加边
            }
        }
        setNodeVisible(neighbor, false);         // 隐藏邻居节点
        setEdgeVisible(nodeId, neighbor, false); // 隐藏边
    }
    collapsedNodes.insert(nodeId);
    // 调用更新方法重新绘制图形
    graphParam->update(viewer);
}

void NodeClickHandler::expandNode(const std::string &nodeId) {
    auto neighbors = getNeighbors(nodeId);
    for (const auto &neighbor : neighbors) {
        setNodeVisible(neighbor, true);         // 显示邻居节点
        setEdgeVisible(nodeId, neighbor, true); // 显示边
        auto neighborNeighbors = getNeighbors(neighbor);
        for (const auto &nn : neighborNeighbors) {
            if (nn != nodeId) {
                setEdgeVisible(nodeId, nn, false); // 移除虚拟边
            }
        }
    }
    collapsedNodes.erase(nodeId);
    // 调用更新方法重新绘制图形
    graphParam->update(viewer);
}

std::vector<std::string> NodeClickHandler::getNeighbors(const std::string &nodeId) {
    std::vector<std::string> neighbors;
    for (const auto &edge : *graphParam->edges) { // 假设 graphParam 存储边的列表
        if (edge.from == nodeId && edge.visible) {
            neighbors.push_back(edge.to);
        } else if (edge.to == nodeId && edge.visible) {
            neighbors.push_back(edge.from);
        }
    }
    return neighbors;
}

void NodeClickHandler::setNodeVisible(const std::string &nodeId, bool visible) {
    if (graphParam->nodes->find(nodeId) != graphParam->nodes->end()) {
        graphParam->nodes->at(nodeId).visible = visible;
    }
}

void NodeClickHandler::setEdgeVisible(const std::string &from, const std::string &to,
                                      bool visible) {
    for (auto &edge : *graphParam->edges) {
        if ((edge.from == from && edge.to == to) || (edge.from == to && edge.to == from)) {
            edge.visible = visible;
        }
    }
}