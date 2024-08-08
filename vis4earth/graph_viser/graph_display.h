#ifndef VIS4EARTH_GRAPH_VISER_GRAPH_DISPLAY_H
#define VIS4EARTH_GRAPH_VISER_GRAPH_DISPLAY_H

#include <QFileDialog>
#include <QMessageBox>

#include <array>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

#include <osg/AnimationPath>
#include <osg/CoordinateSystemNode>
#include <osg/CullFace>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/NodeCallback>
#include <osg/Point>
#include <osg/ShapeDrawable>
#include <osgAnimation/AnimationManagerBase>
#include <osgAnimation/BasicAnimationManager>
#include <osgAnimation/StackedTransform>

#include <qtcore/qtimer>

#include <vis4earth/geographics_cmpt.h>
#include <vis4earth/graph_viser/edge_bundling.h>
#include <vis4earth/graph_viser/graph_io.h>
#include <vis4earth/graph_viser/node_layout.h>
#include <vis4earth/osg_util.h>
#include <vis4earth/qt_osg_reflectable.h>
#include <vis4earth/volume_cmpt.h>

namespace Ui {
class GraphRenderer;
}

namespace VIS4Earth {

class GraphRenderer : public QtOSGReflectableWidget {
    Q_OBJECT
  public:
    double size = 1.0;

    VIS4Earth::Graph myGraph;

    VIS4Earth::EdgeBundling::BundlingParam mybundlingParam;
    VIS4Earth::NodeLayouter::LayoutParam myLayoutParam = {
        myLayoutParam.attraction = 25.1, myLayoutParam.edgeLength = 10,
        myLayoutParam.repulsion = 0.2, myLayoutParam.spring_k = 12.4,
        myLayoutParam.Iteration = 100};
    VIS4Earth::Area myRestriction;
    bool restrictionOn = false;
    struct Node {
        osg::Vec3 pos;
        osg::Vec3 color;
        std::string id;
        bool visible = true; // 默认可见
        int level = 0;       // 默认等级为0 用来展示多分辨率的图和节点收缩
    };

    struct Edge {
        std::string from;
        std::string to;
        std::vector<osg::Vec3> subDivs;
        bool visible = true; // 默认可见
        bool isAdd = false;  // 默认不是后添加的边
    };
    struct CoordRange {
        float minX;
        float maxX;
        float minY;
        float maxY;
    };
    CoordRange coordRange;

  private:
    struct PerRendererParam {
        osg::ref_ptr<osg::Group> grp;

        PerRendererParam() : grp(new osg::Group) {}
    };

    PerRendererParam param;

    class PerGraphParam {
      private:
        float minLongitude, maxLongitude;
        float minLatitude, maxLatitude;
        float minHeight, maxHeight;
        float nodeGeomSize;
        float textSize;
        bool volStartFromLonZero;
        bool arrowFlowEnabled = false; // 标志变量
        bool isAnimating = false;
        std::shared_ptr<std::map<std::string, Node>> nodes;
        std::shared_ptr<std::vector<Edge>> edges;

        osg::ref_ptr<osg::Group> grp;

      public:
        VIS4Earth::Area restriction;
        bool restrictionOFF = true;

        PerGraphParam(std::shared_ptr<std::map<std::string, Node>> nodes,
                      std::shared_ptr<std::vector<Edge>> edges, PerRendererParam *renderer)
            : nodes(std::move(nodes)), edges(std::move(edges)), grp(new osg::Group) {
            const float MinHeight = static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) * 1.1f;
            const float MaxHeight = static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) * 1.3f;

            minLongitude = deg2Rad(-10.f);
            maxLongitude = deg2Rad(+10.f);
            minLatitude = deg2Rad(-20.f);
            maxLatitude = deg2Rad(+20.f);
            minHeight = MinHeight;
            maxHeight = MaxHeight;
            volStartFromLonZero = false;

            auto states = grp->getOrCreateStateSet();
            states->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
        }
        std::shared_ptr<std::map<std::string, Node>> getNodes() { return nodes; }
        std::shared_ptr<std::vector<Edge>> getEdges() { return edges; }
        osg::ref_ptr<osg::Geode> lineGeode;
        osg::ref_ptr<osg::Geometry> lineGeometry;

        void update();
        void createArrowAnimation(const osg::Vec3 &start, const osg::Vec3 &end,
                                  const osg::Vec4 &color);
        void startArrowAnimation();
        void createHighlightAnimation(const osg::Vec3 &start, const osg::Vec3 &end,
                                      const osg::Vec4 &baseColor, const osg::Vec4 &highlightColor);
        void startHighlightAnimation();
        void startTextureAnimation();
        void setRestriction(VIS4Earth::Area res);
        bool setLongitudeRange(float minLonDeg, float maxLonDeg);

        bool setLatitudeRange(float minLatDeg, float maxLatDeg);

        bool setHeightFromCenterRange(float minH, float maxH);

        void setGraphStartFromLongitudeZero(bool flag) { volStartFromLonZero = flag; }

        void setNodeGeometrySize(float sz) { nodeGeomSize = sz; }
        void setTextGeometrySize(float sz) { textSize = sz; }

      private:
        float deg2Rad(float deg) { return deg * osg::PI / 180.f; };

        friend class GraphRenderer;
    };
    std::map<std::string, PerGraphParam> graphs;

  public:
    osg::Group *getGroup() { return param.grp.get(); }
    GraphRenderer(QWidget *parent = nullptr);

    void addGraph(const std::string &name, std::shared_ptr<std::map<std::string, Node>> nodes,
                  std::shared_ptr<std::vector<Edge>> edges);
    PerGraphParam *getGraph(const std::string &name) {
        auto itr = graphs.find(name);
        if (itr == graphs.end())
            return nullptr;
        return &(itr->second);
    }
    std::map<std::string, PerGraphParam> &getGraphs() { return graphs; }

    std::shared_ptr<std::map<std::string, Node>> getNodes(const std::string &graphName);
    std::shared_ptr<std::vector<Edge>> getEdges(const std::string &graphName);
    void update(const std::string &graphName);
    void GraphRenderer::setEdges(const std::string &graphName,
                                 std::shared_ptr<std::vector<Edge>> edges) {
        auto it = graphs.find(graphName);
        if (it != graphs.end()) {
            it->second.edges = edges;
        }
    }

  protected:
    Ui::GraphRenderer *ui;

    // void initOSGResource();
  private slots:
    void loadPointsCSV();

    void loadEdgesCSV();

    void loadAndDrawGraph();

    void applyParams();
    void showGraph();

    void showBundling();
    void setAttraction(double value);

    void setEdgeLength(double value);

    void setRepulsion(double value);
    void setSpringK(double value);

    void setIteration(int value);

    void setRegionRestriction(bool enabled);

    void setMinX(double value);

    void setMaxX(double value);

    void setMinY(double value);
    void setMaxY(double value);
    void onArrowFlowButtonClicked();
    void onHighlightFlowButtonClicked();
    void onTextureFlowButtonClicked();
    void VIS4Earth::GraphRenderer::onGlobalSpringConstantChanged(double value);

    void VIS4Earth::GraphRenderer::onNumberOfIterationsChanged(int value);
    void VIS4Earth::GraphRenderer::onRemainingIterationsChanged(int value);

    void VIS4Earth::GraphRenderer::onCyclesLeftChanged(int value);

    void VIS4Earth::GraphRenderer::onCompatibilityThresholdChanged(double value);

    void VIS4Earth::GraphRenderer::onSmoothWidthChanged(double value);

    void VIS4Earth::GraphRenderer::onDisplacementChanged(double value);

    void VIS4Earth::GraphRenderer::onEdgeDistanceChanged(double value);

    void VIS4Earth::GraphRenderer::onGravitationIsOnToggled(bool checked);

    void VIS4Earth::GraphRenderer::onGravitationCenterXChanged(double value);

    void VIS4Earth::GraphRenderer::onGravitationCenterYChanged(double value);

    void VIS4Earth::GraphRenderer::onGravitationCenterZChanged(double value);

    void VIS4Earth::GraphRenderer::onGravitationExponentChanged(double value);

    void VIS4Earth::GraphRenderer::onEdgeWeightThresholdChanged(double value);

    void VIS4Earth::GraphRenderer::onEdgePercentageThresholdChanged(double value);

    void VIS4Earth::GraphRenderer::onSizeSliderValueChanged(int value);

    void VIS4Earth::GraphRenderer::onFontSizeSliderValueChanged(int value);
};

} // namespace VIS4Earth

#endif // !VIS4EARTH_GRAPH_VISER_GRAPH_RENDERER_H
