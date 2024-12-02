
#include <osg/ShapeDrawable>
#include <osgUtil/LineSegmentIntersector>

#include <vis4earth/graph_viser/NodeClickHandler.h>

namespace VIS4Earth {

NodeClickHandler::NodeClickHandler(GraphRenderer *graphRenderer, osgViewer::Viewer *viewer)
    : graphRenderer(graphRenderer), viewer(viewer) {}

// 在 handle 函数内进行类型和数据检查
bool NodeClickHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH &&
        ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
        float x = ea.getX();
        float y = ea.getY();

        osgUtil::LineSegmentIntersector::Intersections intersections;
        if (viewer->computeIntersections(x, y, intersections)) {
            for (const auto &intersection : intersections) {
                auto drawable = intersection.drawable.get();
                if (drawable && strcmp(drawable->className(), "ShapeDrawable") == 0) {
                    auto sphere = dynamic_cast<osg::ShapeDrawable *>(drawable);

                    if (sphere) {

                        //std::cout << "You clicked on a sphere!" << std::endl;
                        std::string nodeId = getNodeIdFromSphere(sphere);
                        if (nodeId.empty()) {
                            //std::cout << "Node ID is Null " << std::endl;
                            continue;
                        }

                        // 打印点击的节点 ID
                       // std::cout << "Clicked Node ID: " << nodeId << std::endl;

                        if (collapsedNodes.find(nodeId) != collapsedNodes.end()) {
                            expandNode(nodeId);
                            collapsedNodes.erase(nodeId);
                        } else {
                            collapseNode(nodeId);
                            collapsedNodes.insert(nodeId);
                        }
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

void NodeClickHandler::collapseNode(const std::string &nodeId) {
    auto nodes = graphRenderer->getNodes("LoadedGraph");
    auto neighbors = getNeighbors(nodeId);
    auto edges = graphRenderer->getEdges("LoadedGraph");
    // 获取当前节点的等级
    int currentNodeLevel = nodes->at(nodeId).level;
    auto nodeMapping = graphRenderer->getNodeMapping("LoadedGraph");

    if (nodes->at(nodeId).isRepresent) {
        auto &clusterNodes = nodeMapping->at(nodeId);
        // 隐藏簇中的所有非代表节点
        for (const std::string &cnodeId : clusterNodes) {
            if (cnodeId != nodeId) {
                // 将该节点标记为隐藏
                setNodeVisible(cnodeId, false);
            }
        }
        // 添加与外部连接的边
        for (auto &edge : *edges) {
            if (std::find(clusterNodes.begin(), clusterNodes.end(), edge.from) !=
                clusterNodes.end()) {
                // 如果边的起点属于该簇并且终点不在该簇，则重定向到代表节点
                if (std::find(clusterNodes.begin(), clusterNodes.end(), edge.to) ==
                    clusterNodes.end()) {
                    GraphRenderer::Edge newEdge;
                    newEdge.from = nodeId;
                    newEdge.to = edge.to;
                    newEdge.isAdd = true;
                    // 设置细分点
                    auto it = nodes->find(nodeId);
                    newEdge.subDivs.emplace_back(it->second.pos);
                    it = nodes->find(edge.to);
                    newEdge.subDivs.emplace_back(it->second.pos);
                    graphRenderer->getEdges("LoadedGraph")->push_back(newEdge);
                } else if (std::find(clusterNodes.begin(), clusterNodes.end(), edge.to) !=
                           clusterNodes.end()) {
                    // 如果边的终点属于该簇并且起点不在该簇，则重定向到代表节点
                    GraphRenderer::Edge newEdge;
                    newEdge.from = edge.from;
                    newEdge.to = nodeId;
                    newEdge.isAdd = true;
                    // 设置细分点
                    auto it = nodes->find(nodeId);
                    newEdge.subDivs.emplace_back(it->second.pos);
                    it = nodes->find(nodeId);
                    newEdge.subDivs.emplace_back(it->second.pos);
                    graphRenderer->getEdges("LoadedGraph")->push_back(newEdge);
                }
            }
        }

    } else {
        // 连接邻居的邻居并隐藏邻居
        for (const auto &neighbor : neighbors) {
            // 仅当邻居节点的等级高于当前节点时才进行收缩
            if (nodes->at(neighbor).level > currentNodeLevel && nodes->at(neighbor).degree == 1) {
                setNodeVisible(neighbor, false);
            }
        }
    }

    graphRenderer->update("LoadedGraph");
}

void NodeClickHandler::expandNode(const std::string &nodeId) {
    auto nodes = graphRenderer->getNodes("LoadedGraph");
    auto edges = graphRenderer->getEdges("LoadedGraph");
    auto neighbors = getNeighbors(nodeId);
    auto nodeMapping = graphRenderer->getNodeMapping("LoadedGraph");
    // 恢复当前节点的可见性
    setNodeVisible(nodeId, true);
    if (nodes->at(nodeId).isRepresent) {
        auto &clusterNodes = nodeMapping->at(nodeId);
        // 隐藏簇中的所有非代表节点
        for (const std::string &cnodeId : clusterNodes) {
            if (cnodeId != nodeId) {
                // 将该节点标记为隐藏
                setNodeVisible(cnodeId, true);
                // 恢复与该邻居节点相连的边的可见性
                for (auto &edge : *edges) {
                    if ((edge.from == nodeId && edge.to == cnodeId) ||
                        (edge.from == cnodeId && edge.to == nodeId)) {
                        edge.visible = true;
                    }
                }
            }
        }
        for (auto &edge : *edges) {
            if (edge.isAdd == true) {
                edge.visible = false;
            }
        }
        // 恢复邻居节点及其边的可见性
        for (const auto &neighbor : neighbors) {
            // 仅当邻居节点之前被隐藏时才恢复
            if (!nodes->at(neighbor).visible) {
                setNodeVisible(neighbor, true);

                // 恢复与该邻居节点相连的边的可见性
                for (auto &edge : *edges) {
                    if ((edge.from == nodeId && edge.to == neighbor) ||
                        (edge.from == neighbor && edge.to == nodeId)) {
                        edge.visible = true;
                    }
                }
            }
        }
    }

    // 更新图形
    graphRenderer->update("LoadedGraph");
}

std::vector<std::string> NodeClickHandler::getNeighbors(const std::string &nodeId) {
    std::vector<std::string> neighbors;
    auto edges = graphRenderer->getEdges("LoadedGraph");

    for (const auto &edge : *edges) {
        if (edge.from == nodeId) {
            neighbors.push_back(edge.to);
        } else if (edge.to == nodeId) {
            neighbors.push_back(edge.from);
        }
    }

    return neighbors;
}

void NodeClickHandler::setNodeVisible(const std::string &nodeId, bool visible) {
    auto nodes = graphRenderer->getNodes("LoadedGraph");
    if (nodes->find(nodeId) != nodes->end()) {
        nodes->at(nodeId).visible = visible;
    }
    // 获取当前图的边
    auto edges = graphRenderer->getEdges("LoadedGraph");

    // 遍历边并设置可见性
    for (auto &edge : *edges) {
        if (edge.from == nodeId || edge.to == nodeId) {
            edge.visible = visible;
        }
    }
}

void NodeClickHandler::setEdgeVisible(const std::string &from, const std::string &to,
                                      bool visible) {
    auto edges = graphRenderer->getEdges("LoadedGraph");
    for (auto &edge : *edges) {
        if ((edge.from == from && edge.to == to) || (edge.to == from && edge.from == to)) {
            edge.visible = visible;
        }
    }
}
// getNodeIdFromSphere 函数
std::string NodeClickHandler::getNodeIdFromSphere(osg::ShapeDrawable *sphere) {
    osg::Vec3 sphereCenter;
    // 获取用户数据中的中心点
    osg::Referenced *userData = sphere->getUserData();
    if (userData) {
        // 尝试将 userData 转换为 osg::Vec3Array 类型
        osg::Vec3Array *vec3Array = dynamic_cast<osg::Vec3Array *>(userData);
        if (vec3Array) {
            // 确保 vec3Array 中有数据
            if (!vec3Array->empty()) {
                // 获取第一个 Vec3 数据
                sphereCenter = vec3Array->at(0);
                //std::cout << "First Vec3: (" << sphereCenter.x() << ", " << sphereCenter.y() << ", "
                 //         << sphereCenter.z() << ")" << std::endl;
            } else {
                //std::cout << "Vec3Array is empty" << std::endl;
            }
        } else {
            //std::cout << "User data is not of type osg::Vec3Array" << std::endl;
        }
    } else {
       // std::cout << "User data is null" << std::endl;
    }

    // 查找与 sphereCenter 匹配的节点 ID
    auto nodes = graphRenderer->getNodes("LoadedGraph");
    if (nodes) {
        for (const auto &node : *nodes) {
            if ((node.second.pos - sphereCenter).length() < 0.01f) {
                return node.first;
            }
        }
    }
    //std::cout << "No matching node found" << std::endl;
    return "";
}
} // namespace VIS4Earth