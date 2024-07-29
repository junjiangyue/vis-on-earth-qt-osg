#ifndef NODECLICKHANDLER_H
#define NODECLICKHANDLER_H

#include <map>
#include <osg/ShapeDrawable>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <set>
#include <string>
#include <vector>

// 前向声明 PerGraphParam
namespace VIS4Earth {
class GraphRenderer {
  public:
    class PerGraphParam;
};
} // namespace VIS4Earth

class NodeClickHandler : public osgGA::GUIEventHandler {
  public:
    NodeClickHandler(std::map<std::string, osg::ShapeDrawable *> &nodesMap,
                     osgViewer::Viewer *viewer,
                     VIS4Earth::GraphRenderer::PerGraphParam *graphParam);

    virtual bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

  private:
    std::map<std::string, osg::ShapeDrawable *> &nodesMap;
    osgViewer::Viewer *viewer;
    VIS4Earth::GraphRenderer::PerGraphParam *graphParam;

    std::set<std::string> collapsedNodes; // 跟踪已收缩的节点

    void collapseNode(const std::string &nodeId);
    void expandNode(const std::string &nodeId);
    std::vector<std::string> getNeighbors(const std::string &nodeId);
    void setNodeVisible(const std::string &nodeId, bool visible);
    void setEdgeVisible(const std::string &from, const std::string &to, bool visible);
};

#endif // NODECLICKHANDLER_H