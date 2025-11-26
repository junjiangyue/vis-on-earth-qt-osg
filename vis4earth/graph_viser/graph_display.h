#ifndef VIS4EARTH_GRAPH_VISER_GRAPH_DISPLAY_H
#define VIS4EARTH_GRAPH_VISER_GRAPH_DISPLAY_H

#include <QFileDialog>
#include <QMessageBox>

#include <algorithm>
#include <array>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

#include <osg/AnimationPath>
#include <osg/BlendFunc>
#include <osg/CoordinateSystemNode>
#include <osg/CullFace>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/NodeCallback>
#include <osg/Point>
#include <osg/Program>
#include <osg/Shader>
#include <osg/ShapeDrawable>
#include <osg/Texture2D>
#include <osgAnimation/AnimationManagerBase>
#include <osgAnimation/BasicAnimationManager>
#include <osgAnimation/StackedTransform>
#include <osgText/Text>

//#include <qtcore/qtimer>
#include <QtCore/QTimer>

#include "graph_draw.h"
#include <vis4earth/geographics_cmpt.h>
#include <vis4earth/graph_viser/edge_bundling.h>
#include <vis4earth/graph_viser/geographic_regions.h>
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
    int graphTypeIndex;
    double cameraHeightPresent;
    std::shared_ptr<VIS4Earth::Graph> myGraph;
    std::future<void> compatibilityFuture; // 保存异步任务状态
    // VIS4Earth::Graph myGraph;
    std::future<void> _bundlingFuture;

    // 标记整个“全量 bundling 任务”是否在运行
    std::atomic<bool> _bundlingAllRunning{false};

    // 标记每个 LOD 的 bundling 是否已经完成并写入文件
    std::array<std::atomic<bool>, 4> _lodBundlingReady;

    // 存储结果文件目录（建议设为源数据所在目录）
    QString _bundledResultDir;

    // 根据 LOD 生成对应的结果文件路径
    QString bundledFilePathForLOD(int lod) const {
        return _bundledResultDir + QString("/bundled_edges_result_lod%1.csv").arg(lod);
    }
    VIS4Earth::EdgeBundling::BundlingParam mybundlingParam = {
        mybundlingParam.K = 0.1,
        mybundlingParam.I = 50,
        mybundlingParam.cycles = 3,
        mybundlingParam.iter = 50,
        mybundlingParam.compatibilityThreshold = 0.6,
        mybundlingParam.smoothWidth = 3,
        mybundlingParam.S = 0.4,
        mybundlingParam.edgeDistance = 1e-4,
        mybundlingParam.gravitationIsOn = true,
        mybundlingParam.gravitationCenter = glm::vec3(-75.0, 30.0, 0.0),
        mybundlingParam.gravitationExponent = -10.0,
        mybundlingParam.edgeWeightThreshold = -2.0,
        mybundlingParam.edgePercentageThreshold = -1.0};
    VIS4Earth::NodeLayouter::LayoutParam myLayoutParam = {
        myLayoutParam.repulsion = 0.2, myLayoutParam.edgeLength = 10,
        myLayoutParam.attraction = 25.1, myLayoutParam.spring_k = 12.4,
        myLayoutParam.Iteration = 100};
    VIS4Earth::Area myRestriction;
    bool restrictionOn = false;
    struct Node {
        osg::Vec3 pos;
        osg::Vec3 color;
        std::string id;
        bool visible = true; // 默认可见
        bool isRepresent = false;
        int level;
        float size = 1; // 节点大小
        int degree = 0;
        int cluster; // 簇ID
        bool isHover = false;
        std::string label;
    };
    struct Edge {
        std::string id;
        std::string from;
        std::string to;
        float maxHeight;
        float highlightPos = 0.0f; // 0.0到1.0之间的高光位置
        float speed = 0.2f;
        std::vector<osg::Vec3> subDivs;
        float weight = 0;
        bool visible = true; // 默认可见
        bool isAdd = false;  // 默认不是后添加的边
        // 自定义比较运算符
        bool operator<(const Edge &other) const {
            if (from == other.from) {
                return to < other.to;
            }
            return from < other.from;
        }
    };
    struct CoordRange {
        float minX;
        float maxX;
        float minY;
        float maxY;
    };
    CoordRange coordRange;
    std::array<std::vector<std::string>, 4> levelIndex;
    std::array<std::vector<Node>, 4> levelNodeIndex; // 不同层级的nodes
    std::unordered_set<std::string> currentLevelLabels; // 当前层级全部标签ID（快速存在性检查）
    std::unordered_set<std::string> visibleLabels; // 当前可见标签ID
    std::vector<Node> visileNodes;
    std::vector<Node> currentNodes;
    std::unordered_set<std::string> sceneLabels; // 场景中已存在的标签ID
    // 需要新增的标签ID列表
    std::vector<std::string> newAddList;

    // 需要移除的标签ID列表
    std::vector<std::string> removeList;
    // 经纬度网格的分区信息
    struct Grid {
        std::vector<std::string> node_ids; // 存储在该网格内的节点ID
    };

    // 地球网格分区管理（结构体）
    struct EarthGridPartition {
        int latitude_cells;                  // 纬度分区数
        int longitude_cells;                 // 经度分区数
        std::vector<std::vector<Grid>> grid; // 存储所有网格的二维数组
        // 初始化网格分区
        EarthGridPartition(int lat_cells, int lon_cells)
            : latitude_cells(lat_cells), longitude_cells(lon_cells) {
            grid.resize(latitude_cells);
            for (auto &g : grid) {
                g.resize(longitude_cells);
            }
        }

        // 将节点按经纬度坐标插入到网格中
        void insertNodeIntoGrid(const Node &node) {
            // 假设 node.pos.x() 是 纬度 (Latitude)
            // 假设 node.pos.y() 是 经度 (Longitude)

            // 正确的纬度索引计算 (使用 node.pos.x())
            int lat_idx = static_cast<int>((node.pos.x() + 90.0f) * latitude_cells / 180.0f); 

            // 正确的经度索引计算 (使用 node.pos.y())
            int lon_idx = static_cast<int>((node.pos.y() + 180.0f) * longitude_cells / 360.0f); 

            // 防止越界
            lat_idx = std::min(std::max(lat_idx, 0), latitude_cells - 1);
            lon_idx = std::min(std::max(lon_idx, 0), longitude_cells - 1);

            // 将节点ID插入到对应网格
            grid[lat_idx][lon_idx].node_ids.push_back(node.id);
        }

        // 获取指定经纬度范围内的所有节点ID
        std::vector<std::string> getNodesInFrustum(double lat_min, double lat_max, double lon_min,
                                                   double lon_max) {
            std::vector<std::string> visibleNodes;

            int lat_start = static_cast<int>((lat_min + 90.0f) * latitude_cells / 180.0f);
            int lat_end = static_cast<int>((lat_max + 90.0f) * latitude_cells / 180.0f);
            int lon_start = static_cast<int>((lon_min + 180.0f) * longitude_cells / 360.0f);
            int lon_end = static_cast<int>((lon_max + 180.0f) * longitude_cells / 360.0f);

            // 防止越界
            lat_start = std::max(lat_start, 0);
            lat_end = std::min(lat_end, latitude_cells - 1);
            lon_start = std::max(lon_start, 0);
            lon_end = std::min(lon_end, longitude_cells - 1);

            // 遍历网格，收集在范围内的节点ID
            for (int i = lat_start; i <= lat_end; ++i) {
                for (int j = lon_start; j <= lon_end; ++j) {
                    const auto &node_ids = grid[i][j].node_ids;
                    visibleNodes.insert(visibleNodes.end(), node_ids.begin(), node_ids.end());
                }
            }

            return visibleNodes;
        }
        
        // 清空所有网格
        void clearGrid() {
            for (auto &row : grid) {
                for (auto &cell : row) {
                    cell.node_ids.clear();
                }
            }
        }
    };
    EarthGridPartition earthGrid{180, 360};
    std::vector<Graph> simplifiedGraphsList;
    std::vector<std::unordered_map<std::string, std::vector<std::string>>> nodeMappingList;
    std::vector<std::unordered_map<std::string, std::vector<Edge>>>
        edgeMappingList; // 使用边的id作为键
    void simplifyGraphWithDBSCAN(const Graph &originalGraph);
    struct PerRendererParam {
        osg::ref_ptr<osg::Group> grp;
        osg::ref_ptr<osg::Camera> _camera;

        PerRendererParam() : grp(new osg::Group), _camera(nullptr) {}
        void setCamera(osg::Camera *camera) { _camera = camera; }
    };
    PerRendererParam param; // 移到public部分

    // LOD相关数据存储 (类似levelIndex的管理方式)
    std::array<std::shared_ptr<std::map<std::string, Node>>, 4>
        lodNodesData;                                               // 4个LOD层级的节点数据
    std::array<std::shared_ptr<std::vector<Edge>>, 4> lodEdgesData; // 4个LOD层级的边数据
    std::shared_ptr<std::map<std::string, Node>> satelliteNodes;
    std::shared_ptr<std::vector<Edge>> satelliteEdges;
    
    int currentActiveLODLevel = -1; // 当前活动的LOD级别，-1表示未初始化

    // 新增：区域和聚合边相关数据结构
    struct Region {
        std::vector<std::string> nodeIds; // 区域内的节点ID列表
        osg::Vec3 centroid;               // 区域质心坐标
        std::string representativeNodeId; // 区域代表节点ID
        bool isValid = false;             // 区域是否有效
    };

    struct AggregatedEdgeInfo {
        osg::Vec3 startPos;       // 聚合边起点位置
        osg::Vec3 endPos;         // 聚合边终点位置
        float totalWeight = 0.0f; // 聚合后的总权重
        int edgeCount = 0;        // 参与聚合的边数量
        osg::Vec4 color;          // 聚合边颜色
    };

    // 按LOD层级存储的区域信息
    std::array<std::map<int, Region>, 4> lodRegions; // [LOD][RegionID] -> Region

    // 视锥剔除相关数据结构
    struct SimpleFrustumBounds {
        double minLat, maxLat, minLon, maxLon;
        double cameraHeight;
        bool isValid = false;
    };

    // 视锥剔除缓存
    SimpleFrustumBounds lastFrustumBounds;
    std::map<std::string, bool> cachedNodeVisibility; // 节点可见性缓存
    std::map<std::string, bool> cachedEdgeVisibility; // 边可见性缓存（使用edge.id作为key）
    bool frustumBoundsChanged = true;

    // 视锥剔除辅助函数
    bool extractCameraBounds(osg::Camera *camera, SimpleFrustumBounds &bounds);
    bool frustumSignificantlyChanged(const SimpleFrustumBounds &current,
                                     const SimpleFrustumBounds &last);
    osg::Vec3 latLonToWorldPos(double lat, double lon);
    void performPreciseCulling(osg::Camera *camera, const std::vector<std::string> &candidateNodes,
                               std::shared_ptr<std::map<std::string, Node>> allNodes);
    void cullEdgesByVisibility(std::shared_ptr<std::vector<Edge>> allEdges,
                               std::shared_ptr<std::map<std::string, Node>> allNodes);
    void resetAllVisibility(std::shared_ptr<std::map<std::string, Node>> allNodes,
                            std::shared_ptr<std::vector<Edge>> allEdges);
    void updateVisibilityFromCache(const std::string &graphName);

    // 调试函数
    void debugEarthGridStatus();
    void debugNodeCoordinates(std::shared_ptr<std::map<std::string, Node>> nodes,
                              int maxSamples = 10);

    void generateAggregatedEdgesForLOD(int lodLevel,
                                       std::shared_ptr<std::map<std::string, Node>> lodNodes,
                                       std::shared_ptr<std::vector<Edge>> lodEdges,
                                       std::shared_ptr<std::map<std::string, Node>> allNodes,
                                       std::shared_ptr<std::vector<Edge>> allEdges);

    // std::unordered_set<std::string> currentLevelLabels; // 当前层级全部标签ID（快速存在性检查）
  private:
    struct GraphLevel {
        std::shared_ptr<std::map<std::string, Node>> nodes; // 当前层次的节点
        std::shared_ptr<std::vector<Edge>> edges;           // 当前层次的边
        std::shared_ptr<std::map<std::string, std::vector<std::string>>> nodeMapping; // 节点映射
        std::shared_ptr<std::map<Edge, std::vector<Edge>>> edgeMapping;
    };
    std::vector<std::vector<float>> heightMap;
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
        
        // 纹理流动动画相关成员变量
        bool isTextureFlowAnimating = false;
        osg::ref_ptr<osg::NodeCallback> textureFlowCallback;
        osg::ref_ptr<osg::Image> lineDataImageForGeom;
        
        std::shared_ptr<std::map<std::string, Node>> nodes;                           // 当前nodes
        std::shared_ptr<std::vector<Edge>> edges;       
        std::shared_ptr<std::map<std::string, Node>> satelliteNodes;
        std::shared_ptr<std::vector<Edge>> satelliteEdges;                            // 当前edges
        std::shared_ptr<std::map<std::string, std::vector<std::string>>> nodeMapping; // 节点映射
        std::shared_ptr<std::map<Edge, std::vector<Edge>>> edgeMapping;
        std::vector<std::vector<float>> heightMap;
        std::vector<GraphLevel> levels; // 存放多层次的图

        osg::ref_ptr<osg::Node> _satelliteModel;//卫星

        osg::ref_ptr<osg::Group> grp;            // 全部的group，包含grpWithoutSats和卫星
        osg::ref_ptr<osg::Group> sats;// 卫星
        osg::ref_ptr<osg::Group> edgeNodegrp; // 节点和线

        // 用于边绘制的顶点数据结构
        struct LineVertex {
            osg::Vec3 position;  // 顶点位置
            osg::Vec4 colorFrom; // 起点颜色
            osg::Vec4 colorTo;   // 终点颜色
            float weight;        // 边权重
        };

        // VBO相关成员
        osg::ref_ptr<osg::Geometry> mEdgeGeometry;
        osg::ref_ptr<osg::Geode> mEdgeGeode;
        osg::ref_ptr<osg::Vec3Array> mVertexArray;
        osg::ref_ptr<osg::Vec4Array> mColorFromArray;
        osg::ref_ptr<osg::Vec4Array> mColorToArray;
        osg::ref_ptr<osg::FloatArray> mWeightArray;

        // Shader相关成员
        osg::ref_ptr<osg::Program> mEdgeProgram;
        bool mUseNewRenderer = true; // 控制是否使用新的渲染方式

        // LOD数据成员变量（从GraphRenderer同步）
        std::array<std::shared_ptr<std::map<std::string, Node>>, 4>
            lodNodesData; // 4个LOD层级的节点数据
        std::array<std::shared_ptr<std::vector<Edge>>, 4> lodEdgesData; // 4个LOD层级的边数据

        // 当前LOD层级
        int currentLODLevel = 3; // 默认为最高细节层级
        void setSatellite(std::shared_ptr<std::map<std::string, Node>> nodes,
                          std::shared_ptr<std::vector<Edge>> edges) {
            satelliteNodes = nodes;
            satelliteEdges = edges;
        }
        // 初始化Shader程序
        void initEdgeShaders();
        // 更新边的VBO数据
        void updateEdgeVBO();
        // 原有的VBO更新逻辑（LOD 3使用）
        void updateEdgeVBO_Original(const std::function<osg::Vec3(const osg::Vec3 &)> &vec3ToSphere,
                                    osg::ref_ptr<osg::FloatArray> mLineIDArray);

      public:
        int graphTypeIndex;
        VIS4Earth::Area restriction;
        bool restrictionOFF = true;
        osg::ref_ptr<osg::Camera> _camera;

        PerGraphParam(std::shared_ptr<std::map<std::string, Node>> nodes,
                      std::shared_ptr<std::vector<Edge>> edges, PerRendererParam *renderer)
            : nodes(std::move(nodes)), edges(std::move(edges)), grp(new osg::Group),
              _camera(nullptr) {
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
        std::shared_ptr<std::map<std::string, std::vector<std::string>>> getNodeMapping() {
            return nodeMapping;
        }
        std::shared_ptr<std::map<Edge, std::vector<Edge>>> getEdgeMapping() { return edgeMapping; }
        osg::ref_ptr<osg::Geode> lineGeode;
        osg::ref_ptr<osg::Geometry> lineGeometry;
        osg::ref_ptr<osg::Geode> triangleGeode; // 新增用于保存三角形的 Geode
        osg::Vec3Array *segVerts;

        void setCamera(osg::Camera *camera) { _camera = camera; }
        void update();
        void createArrowAnimation(const osg::Vec3 &start, const osg::Vec3 &end,
                                  const osg::Vec4 &color, const int startIndex, const int endIndex);
        osg::Image *createLineDataTexture();
        void startArrowAnimation();
        void startHighlightAnimation();
        void startTextureAnimation();
        void startStarAnimation();
        void startTextureFlowAnimation(); // 新增纹理流动动画函数
        void setRestriction(VIS4Earth::Area res);
        bool setLongitudeRange(float minLonDeg, float maxLonDeg);

        bool setLatitudeRange(float minLatDeg, float maxLatDeg);

        bool setHeightFromCenterRange(float minH, float maxH);

        void setGraphStartFromLongitudeZero(bool flag) { volStartFromLonZero = flag; }

        void setNodeGeometrySize(float sz) { nodeGeomSize = sz; }
        void setTextGeometrySize(float sz) { textSize = sz; }
        void setLevelGraph(int level);
        void generateHierarchicalGraphs(std::shared_ptr<std::map<std::string, Node>> &initialNodes,
                                        std::shared_ptr<std::vector<Edge>> &initialEdges);
        void performClustering(const GraphLevel &previousLevel, GraphLevel &currentLevel, int p);
        void performLouvainClustering(const GraphLevel &previousLevel, GraphLevel &currentLevel,
                                      int p);
        bool isPointInPolygon(float px, float py, const std::vector<osg::Vec3> &polygon);
        float getBuildingHeightAtLatLon(float lat, float lon);

        void setUseNewRenderer(bool use) { mUseNewRenderer = use; }
        bool getUseNewRenderer() const { return mUseNewRenderer; }

        // LOD相关方法 - 只保留setActiveLODDataSource，用于接收外部设置的数据
        void setActiveLODDataSource(int targetMaxLevel);

        // 发光效果控制方法
        void setGlowIntensity(float intensity);
        void setGlobalAlpha(float alpha);
        void setLineThickness(float thickness);

        // 地理LOD处理函数
        void generateGeographicLODData(int lodLevel,
                                       std::shared_ptr<std::map<std::string, Node>> allNodes,
                                       std::shared_ptr<std::vector<Edge>> allEdges);

        // 为指定LOD级别生成聚合边
        void generateAggregatedEdgesForLOD(int lodLevel,
                                           std::shared_ptr<std::map<std::string, Node>> lodNodes,
                                           std::shared_ptr<std::vector<Edge>> lodEdges,
                                           std::shared_ptr<std::map<std::string, Node>> allNodes,
                                           std::shared_ptr<std::vector<Edge>> allEdges);

        // 生成基于地理分区的LOD数据
        void generateGeographicLODDataOld(int lodLevel,
                                          std::shared_ptr<std::map<std::string, Node>> allNodes,
                                          std::shared_ptr<std::vector<Edge>> allEdges);

        // 实用函数
        float deg2Rad(float deg) { return deg * osg::PI / 180.f; };

      private:
        friend class GraphRenderer;
    };
    std::map<std::string, PerGraphParam> graphs;

    // 添加屏幕网格结构
    struct ScreenGrid {
        int gridWidth;               // 网格的列数
        int gridHeight;              // 网格的行数
        float cellWidth;             // 单元格宽度（像素）
        float cellHeight;            // 单元格高度（像素）
        std::vector<bool> gridCells; // 每个单元格是否已被占用

        // 初始化屏幕网格
        ScreenGrid(int screenWidth = 1000, int screenHeight = 1000, float cellWidth = 20,
                   float cellHeight = 20)
            : cellWidth(cellWidth), cellHeight(cellHeight) {
            gridWidth = std::ceil(screenWidth / cellWidth);
            gridHeight = std::ceil(screenHeight / cellHeight);
            gridCells.resize(gridWidth * gridHeight, false);
        }

        // 检查某个单元格是否已被占用
        bool isOccupied(int x, int y) const {
            if (x < 0 || x >= gridWidth || y < 0 || y >= gridHeight)
                return true;
            return gridCells[y * gridWidth + x];
        }

        // 标记某个单元格为已占用
        void markOccupied(int x, int y) {
            if (x >= 0 && x < gridWidth && y >= 0 && y < gridHeight) {
                gridCells[y * gridWidth + x] = true;
            }
        }

        // 将屏幕坐标转换为网格坐标
        std::pair<int, int> screenToGrid(float screenX, float screenY) const {
            int gridX = std::floor(screenX / cellWidth);
            int gridY = std::floor(screenY / cellHeight);
            return {gridX, gridY};
        }
    };
    // 添加投影函数
    osg::Vec3 projectToScreen(const osg::Vec3 &worldPos, osg::Camera *camera) {
        osg::Matrixd viewMatrix = camera->getViewMatrix();
        osg::Matrixd projectionMatrix = camera->getProjectionMatrix();
        osg::Viewport *viewport = camera->getViewport();

        osg::Vec4 screenPos = osg::Vec4(worldPos, 1.0) * viewMatrix * projectionMatrix;

        if (screenPos.w() != 0.0) {
            screenPos.x() /= screenPos.w();
            screenPos.y() /= screenPos.w();
        }

        return osg::Vec3((screenPos.x() * 0.5 + 0.5) * viewport->width(),
                         (screenPos.y() * 0.5 + 0.5) * viewport->height(), screenPos.z());
    }

    // void adjustTextPosition(std::vector<osg::ref_ptr<osgText::Text>> &texts, float nodeGeomSize,
    //                         osg::ref_ptr<osg::Camera> camera);

  public:
    constexpr static double WGS_84_RADIUS_POLAR = 6356752.3142;
    osg::Group *getGroup() { return param.grp.get(); }
    GraphRenderer(QWidget *parent = nullptr);

    void addGraph(const std::string &name, std::shared_ptr<std::map<std::string, Node>> nodes,
                  std::shared_ptr<std::vector<Edge>> edges);
    void addGraphForBundling(const std::string &name,
                             std::shared_ptr<std::map<std::string, Node>> nodes,
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
    std::shared_ptr<std::map<std::string, std::vector<std::string>>>
    getNodeMapping(const std::string &graphName);
    std::shared_ptr<std::map<Edge, std::vector<Edge>>>
    getEdgeMapping(const std::string &graphName) {
        auto itr = graphs.find(graphName);
        if (itr != graphs.end()) {
            return itr->second.getEdgeMapping();
        }
        return nullptr;
    }
    void update(const std::string &graphName);
    void updateLabelLists(const std::string &graphName);
    void syncSceneGraph(const std::string &graphName);
    void cameraUpdate(const std::string &graphName, double cameraHeight);
    void frustumCulling(const std::string &graphName, double minLon, double maxLon, double minLat,
                        double maxLat, int currentLevel);
    int getCurrentLevel(double height);
    void setEdges(const std::string &graphName, std::shared_ptr<std::vector<Edge>> edges) {
        auto it = graphs.find(graphName);
        if (it != graphs.end()) {
            it->second.edges = edges;
        }
    }
    void loadGeoTypeGraph();
    void loadNoGeoTypeGraph();
    void LoadConfigFromTxt(const QString &filePath); // 添加此方法

    void loadMarker();

    // LOD相关方法
    void updateActiveLOD(double cameraHeight);
    void initializeLODData(std::shared_ptr<std::map<std::string, Node>> allNodes,
                           std::shared_ptr<std::vector<Edge>> allEdges);

    // 生成基于地理分区的LOD数据
    void generateGeographicLODData(int lodLevel,
                                   std::shared_ptr<std::map<std::string, Node>> allNodes,
                                   std::shared_ptr<std::vector<Edge>> allEdges);
    // 生成基于地理分区的LOD数据
    void generateGeographicLODDataOld(int lodLevel,
                                      std::shared_ptr<std::map<std::string, Node>> allNodes,
                                      std::shared_ptr<std::vector<Edge>> allEdges);

    // 渐进式LOD辅助函数
    std::string simplifyRegionName(const std::string &originalName);
    osg::Vec3 calculateRegionCentroid(const std::vector<std::string> &nodeIds,
                                      std::shared_ptr<std::map<std::string, Node>> allNodes);
    float evaluateNodeImportance(const Node &node);
    std::vector<std::string>
    selectRepresentativeNodes(const std::vector<std::string> &nodeIds,
                              std::shared_ptr<std::map<std::string, Node>> allNodes, int maxNodes,
                              int minLevel, int maxLevel);

  protected:
    Ui::GraphRenderer *ui;

    // void initOSGResource();
  private slots:
    void onComboBoxGraphTypeChanged(int index);
    void loadPointsCSV();

    std::shared_ptr<VIS4Earth::Graph>
    buildGraphFromLODData(const std::shared_ptr<std::map<std::string, Node>> &lodNodes,
                          const std::shared_ptr<std::vector<Edge>> &lodEdges);

    void saveBundledGraphToFile(const std::shared_ptr<VIS4Earth::Graph> &graph,
                                const QString &filePath);

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
    void onStarFlowButtonClicked();
    void onGlobalSpringConstantChanged(double value);

    void onNumberOfIterationsChanged(int value);
    void onRemainingIterationsChanged(int value);

    void onCyclesLeftChanged(int value);

    void onCompatibilityThresholdChanged(double value);

    void onSmoothWidthChanged(double value);

    void onDisplacementChanged(double value);

    void onEdgeDistanceChanged(double value);

    void onGravitationIsOnToggled(bool checked);

    void onGravitationCenterXChanged(double value);

    void onGravitationCenterYChanged(double value);

    void onGravitationCenterZChanged(double value);

    void onGravitationExponentChanged(double value);

    void onEdgeWeightThresholdChanged(double value);

    void onEdgePercentageThresholdChanged(double value);

    void onFontSizeSliderValueChanged(int value);

    void onResolutionSliderValueChanged(int value);

    void updateGraphParameters(PerGraphParam *graphParam);
    void copyGraphData(std::shared_ptr<std::map<std::string, Node>> &nodes,
                       std::shared_ptr<std::vector<Edge>> &edges);
};

} // namespace VIS4Earth

#endif // !VIS4EARTH_GRAPH_VISER_GRAPH_RENDERER_H
