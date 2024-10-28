#ifndef NODECLICKHANDLER_H
#define NODECLICKHANDLER_H

#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <set>
#include <string>
#include <vis4earth/graph_viser/graph_display.h>

namespace VIS4Earth {

class NodeClickHandler : public osgGA::GUIEventHandler {
  public:
    NodeClickHandler(GraphRenderer *graphRenderer, osgViewer::Viewer *viewer);

    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) override;

  private:
    GraphRenderer *graphRenderer;
    osgViewer::Viewer *viewer;
    std::set<std::string> collapsedNodes;

    void collapseNode(const std::string &nodeId);
    void expandNode(const std::string &nodeId);
    std::vector<std::string> getNeighbors(const std::string &nodeId);
    void setNodeVisible(const std::string &nodeId, bool visible);
    void setEdgeVisible(const std::string &from, const std::string &to, bool visible);
    std::string getNodeIdFromSphere(osg::ShapeDrawable *sphere);
};

} // namespace VIS4Earth

#endif // NODECLICKHANDLER_H