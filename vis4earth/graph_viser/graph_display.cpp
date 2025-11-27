#include "graph_display.h"
#include "DBSCAN.h"
#include <ui_graph_layout.h>
#include "graph_draw.h"
#include "graph_animation.h"
#include "graph_gpu_edge_tables.h"
#include <algorithm>
#include <memory>
#include <osgText/Font>
#include <set>

using namespace VIS4Earth;
static std::array<float, 2> lonRng = {-90.f, 90.f};
const std::array<float, 2> latRng = {-90.f, 90.f};
const std::array<float, 2> hRng = {10000.f, 15000.f};
const float hScale = 10.f;

VIS4Earth::GraphRenderer::GraphRenderer(QWidget *parent) : QtOSGReflectableWidget(ui, parent) {

    // 连接 comboBox 的信号来记录当前选择的索引
    graphTypeIndex = 0; // 默认选择第一个（有经纬度的图）
    // ui->groupBox->hide();
    ui->fontSizeSlider->hide();
    ui->fontSizeLabel->hide();
    connect(ui->comboBoxGraphType, SIGNAL(currentIndexChanged(int)), this,
            SLOT(onComboBoxGraphTypeChanged(int)));

    // 打开文件夹
    connect(ui->loadPointsButton, &QPushButton::clicked, this, &GraphRenderer::loadPointsCSV);
    connect(ui->loadEdgesButton, &QPushButton::clicked, this, &GraphRenderer::loadEdgesCSV);
    // 加载图并绘制
    connect(ui->loadAndDrawGraphButton, &QPushButton::clicked, this,
            &GraphRenderer::loadAndDrawGraph);

    connect(ui->showGraphLayoutButton, &QPushButton::clicked, this, &GraphRenderer::showGraph);
    connect(ui->showEdgeBundlingButton, &QPushButton::clicked, this, &GraphRenderer::showBundling);

    connect(ui->fontSizeSlider, &QSlider::valueChanged, this,
            &GraphRenderer::onFontSizeSliderValueChanged);
    // 连接参数设置的信号到槽函数
    connect(ui->spinBoxAttraction, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &GraphRenderer::setAttraction);
    connect(ui->spinBoxEdgeLength, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &GraphRenderer::setEdgeLength);
    connect(ui->spinBoxRepulsion, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &GraphRenderer::setRepulsion);
    connect(ui->spinBoxSpringK, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &GraphRenderer::setSpringK);
    connect(ui->spinBoxIteration, QOverload<int>::of(&QSpinBox::valueChanged), this,
            &GraphRenderer::setIteration);

    connect(ui->regionRestrictionButton, &QPushButton::clicked, this,
            &GraphRenderer::setRegionRestriction);
    connect(ui->spinBoxMinX, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &GraphRenderer::setMinX);
    connect(ui->spinBoxMaxX, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &GraphRenderer::setMaxX);
    connect(ui->spinBoxMinY, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &GraphRenderer::setMinY);
    connect(ui->spinBoxMaxY, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &GraphRenderer::setMaxY);

    //// 连接全局弹簧常数 (K)
    connect(ui->spinBoxGlobalSpringConstant, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GraphRenderer::onGlobalSpringConstantChanged);

    // 连接兼容性阈值
    connect(ui->spinBoxCompatibilityThreshold, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GraphRenderer::onCompatibilityThresholdChanged);

    // 连接平滑宽度
    connect(ui->spinBoxSmoothWidth, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &GraphRenderer::onSmoothWidthChanged);

    // 连接位移 (S)
    connect(ui->spinBoxDisplacement, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &GraphRenderer::onDisplacementChanged);

    // 连接边距离
    connect(ui->spinBoxEdgeDistance, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &GraphRenderer::onEdgeDistanceChanged);

    // 连接边权重阈值
    connect(ui->spinBoxEdgeWeightThreshold, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GraphRenderer::onEdgeWeightThresholdChanged);

    // 连接边百分比阈值
    connect(ui->spinBoxEdgePercentageThreshold,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &GraphRenderer::onEdgePercentageThresholdChanged);
    connect(ui->highlightFlowButton, &QPushButton::clicked, this,
            &GraphRenderer::onHighlightFlowButtonClicked);
    connect(ui->textureFlowButton, &QPushButton::clicked, this,
            &GraphRenderer::onTextureFlowButtonClicked);

    connect(ui->starFlowButton, &QPushButton::clicked, this,
            &GraphRenderer::onStarFlowButtonClicked);

    // 初始化LOD数据容器
    for (int i = 0; i < 4; ++i) {
        lodNodesData[i].reset();
        lodEdgesData[i].reset();
    }

    for (auto &flag : _lodBundlingReady) {
        flag = false;
    }
}

void VIS4Earth::GraphRenderer::addGraph(const std::string &name,
                                        std::shared_ptr<std::map<std::string, Node>> nodes,
                                        std::shared_ptr<std::vector<Edge>> edges) {
    auto itr = graphs.find(name);
    if (itr != graphs.end()) {
        param.grp->removeChild(itr->second.grp);
        sceneLabels.clear();

        // 清空LOD数据
        for (int i = 0; i < 4; ++i) {
            lodNodesData[i].reset();
            lodEdgesData[i].reset();
        }
        currentActiveLODLevel = -1; // 重置LOD级别

        graphs.erase(itr);
    }
    auto opt = graphs.emplace(std::piecewise_construct, std::forward_as_tuple(name),
                              std::forward_as_tuple(nodes, edges, &param));
    opt.first->second.edgeNodegrp = new osg::Group;
    param.grp->addChild(opt.first->second.grp);
}

void VIS4Earth::GraphRenderer::addGraphForBundling(
    const std::string &name, std::shared_ptr<std::map<std::string, Node>> nodes,
    std::shared_ptr<std::vector<Edge>> edges) {
    auto itr = graphs.find(name);
    if (itr != graphs.end()) {
        param.grp->removeChild(itr->second.grp);
        sceneLabels.clear();
        graphs.erase(itr);
    }
    auto opt = graphs.emplace(std::piecewise_construct, std::forward_as_tuple(name),
                              std::forward_as_tuple(nodes, edges, &param));
    opt.first->second.edgeNodegrp = new osg::Group;
    param.grp->addChild(opt.first->second.grp);
}

std::shared_ptr<std::map<std::string, GraphRenderer::Node>>
GraphRenderer::getNodes(const std::string &graphName) {
    auto itr = graphs.find(graphName);
    if (itr != graphs.end()) {
        return itr->second.getNodes();
    }
    return nullptr;
}

std::shared_ptr<std::vector<GraphRenderer::Edge>>
GraphRenderer::getEdges(const std::string &graphName) {
    auto itr = graphs.find(graphName);
    if (itr != graphs.end()) {
        return itr->second.getEdges();
    }
    return nullptr;
}
std::shared_ptr<std::map<std::string, std::vector<std::string>>>
GraphRenderer::getNodeMapping(const std::string &graphName) {
    auto itr = graphs.find(graphName);
    if (itr != graphs.end()) {
        return itr->second.getNodeMapping();
    }
    return nullptr;
}

void GraphRenderer::update(const std::string &graphName) {
    auto itr = graphs.find(graphName);
    if (itr != graphs.end()) {
        itr->second.update();
    }
}
void GraphRenderer::updateLabelLists(const std::string &graphName) {
    // 清空新增和移除列表
    newAddList.clear();
    removeList.clear();

    // 如果场景中没有标签，将所有当前节点添加到新增列表
    if (sceneLabels.empty()) {
        for (const auto &node : currentNodes) {
            if (node.visible)
                newAddList.push_back(node.id);
        }
        return;
    }

    // 找出需要移除的标签（在场景中但不在当前层级中的标签）
    for (const auto &sceneLabel : sceneLabels) {
        if (currentLevelLabels.find(sceneLabel) == currentLevelLabels.end()) {
            removeList.push_back(sceneLabel);
        }
    }

    // 找出需要新增的标签（在当前层级中但不在场景中的标签）
    for (const auto &node : currentNodes) {
        if (sceneLabels.find(node.id) == sceneLabels.end()) {
            newAddList.push_back(node.id);
        }
    }
}
void VIS4Earth::GraphRenderer::syncSceneGraph(const std::string &graphName) {
    auto graphParam = getGraph(graphName);
    if (!graphParam)
        return;
    std::size_t maxTextCount = std::numeric_limits<std::size_t>::max();
    if (graphParam->currentLODLevel == 0 || graphParam->currentLODLevel == 1) {
        maxTextCount = 0; // LOD0 / LOD1 最多 50 个文字标签
    }
    if (graphParam->currentLODLevel == 2) {
        maxTextCount = 0; // LOD2 最多 100 个文字标签
    }
    if (graphParam->currentLODLevel == 3) {
        maxTextCount = 0; // LOD3 最多 100 个文字标签
    }
    // 移除不需要的标签
    for (const auto &labelId : removeList) {
        // 找到并移除对应的标签节点
        for (int i = 0; i < graphParam->grp->getNumChildren(); ++i) {
            osg::Node *node = graphParam->grp->getChild(i);
            osg::Geode *geode = dynamic_cast<osg::Geode *>(node);
            if (geode) {
                for (unsigned int j = 0; j < geode->getNumDrawables(); ++j) {
                    osgText::Text *text = dynamic_cast<osgText::Text *>(geode->getDrawable(j));
                    if (text && text->getText().createUTF8EncodedString() == labelId) {
                        graphParam->grp->removeChild(node);
                        sceneLabels.erase(labelId);
                        break;
                    }
                }
            }
        }
    }
    std::vector<osg::ref_ptr<osgText::Text>> textNodes;

    // 遍历并添加当前场景图中所有的文字标签
    for (int i = 0; i < graphParam->grp->getNumChildren(); ++i) {
        osg::Node *node = graphParam->grp->getChild(i);
        osg::Geode *geode = dynamic_cast<osg::Geode *>(node);
        if (geode) {
            for (unsigned int j = 0; j < geode->getNumDrawables(); ++j) {
                osgText::Text *text = dynamic_cast<osgText::Text *>(geode->getDrawable(j));
                if (text) {
                    textNodes.push_back(text);
                }
            }
        }
    }
    if (textNodes.size() > maxTextCount) {
        // 只保留前 maxTextCount 个
        for (int i = maxTextCount; i < textNodes.size(); ++i) {
            osgText::Text *text = textNodes[i].get();
            if (!text)
                continue;

            // 找到对应 geode 并删除
            for (int c = 0; c < graphParam->grp->getNumChildren(); ++c) {
                osg::Node *node = graphParam->grp->getChild(c);
                osg::Geode *geode = dynamic_cast<osg::Geode *>(node);
                if (geode) {
                    for (unsigned int j = 0; j < geode->getNumDrawables(); ++j) {
                        osgText::Text *t = dynamic_cast<osgText::Text *>(geode->getDrawable(j));
                        if (t == text) {
                            graphParam->grp->removeChild(node);
                            // 移除 sceneLabels 中的 id
                            std::string id = t->getText().createUTF8EncodedString();
                            sceneLabels.erase(id);
                            break;
                        }
                    }
                }
            }
        }

        // 截断 textNodes，只保留前 maxTextCount 个
        textNodes.resize(maxTextCount);
    }

    std::shared_ptr<std::map<std::string, Node>> nodesWithLevel = graphParam->nodes;
    // 添加新的标签
    for (const auto &labelId : newAddList) {
        if (textNodes.size() >= maxTextCount) {
            break;
        }
        // 直接从当前LOD的节点数据中查找节点信息
        auto it = nodesWithLevel->find(labelId);

        if (it != nodesWithLevel->end() && it->second.visible) {
            // 创建新的文字标签
            osg::ref_ptr<osgText::Text> text = new osgText::Text;
            text->setText(it->second.id);
            text->setFont("Fonts/simhei.ttf");
            text->setAxisAlignment(osgText::Text::SCREEN);
            text->setCharacterSize(graphParam->textSize ? graphParam->textSize * 0.25
                                                        : graphParam->nodeGeomSize * 0.25);

            // 设置标签位置
            osg::Vec3 pos = GraphUtils::vec3ToSphere(it->second.pos);
            pos.z() += it->second.size * 0.30f * graphParam->nodeGeomSize;
            text->setPosition(pos);

            // 设置标签颜色
            text->setColor(it->second.isHover ? osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)
                                              : osg::Vec4(0.8f, 0.8f, 0.8f, 1.0f));

            // 添加到场景图
            osg::ref_ptr<osg::Geode> geode = new osg::Geode;
            geode->addDrawable(text.get());
            graphParam->grp->addChild(geode.get());
            textNodes.push_back(text);
            // 更新场景标签集合
            sceneLabels.insert(labelId);
        }
    }
    VIS4Earth::GraphUtils::adjustTextPosition(textNodes, graphParam->nodeGeomSize, graphParam->_camera);
}
void VIS4Earth::GraphRenderer::cameraUpdate(const std::string &graphName, double cameraHeight) {
    /*
    * 检测当前高度

    getCurrentLevel()
    frustumCulling();
​	syncSceneGraph();
    */
    cameraHeightPresent = cameraHeight;
    int currentLevel = getCurrentLevel(cameraHeight);

    // 使用渐进式LOD数据而不是原始层级数据
    currentLevelLabels.clear();
    currentNodes.clear();

    // 获取当前LOD级别对应的节点数据
    if (currentLevel >= 0 && currentLevel < 4 && lodNodesData[currentLevel]) {
        // 遍历当前LOD级别的所有节点
        for (const auto &nodePair : *lodNodesData[currentLevel]) {
            const Node &node = nodePair.second;

            // 添加节点ID到标签集合
            currentLevelLabels.insert(node.id);

            // 添加节点到当前节点列表（直接存储Node对象）
            currentNodes.push_back(node);
        }

        std::cout << "CameraUpdate: Using LOD " << currentLevel << " with " << currentNodes.size()
                  << " nodes" << std::endl;
    } else {
        // 如果LOD数据不可用，回退到原始逻辑
        std::cout << "CameraUpdate: LOD data not available, using original logic" << std::endl;

        for (int i = 0; i <= currentLevel; ++i) {
            // 遍历 levelIndex 中的每个 vector
            for (const auto &str : levelIndex[i]) {
                currentLevelLabels.insert(str); // 将每个字符串插入到 unordered_set 中
            }
        }

        for (int i = 0; i <= currentLevel; i++) {
            // 获取level<=currentLevel的nodes
            for (int j = 0; j < levelNodeIndex[i].size(); j++) {
                auto node = levelNodeIndex[i][j];
                currentNodes.push_back(node);
            }
        }
    }
    // frustumCulling(graphName, minLon, maxLon, minLat, maxLat, currentLevel);
    updateLabelLists(graphName);
    syncSceneGraph(graphName);
}
void VIS4Earth::GraphRenderer::frustumCulling(const std::string &graphName, double minLon,
                                              double maxLon, double minLat, double maxLat,
                                              const int currentLevel) {
    // 只对LOD3进行视锥剔除，其他级别数据量较少无需剔除
    if (currentLevel != 3)
        return;

    PerGraphParam *graphParam = getGraph(graphName);
    if (!graphParam || !graphParam->_camera)
        return;

    // 1. 提取视锥参数
    SimpleFrustumBounds currentBounds;
    if (!extractCameraBounds(graphParam->_camera, currentBounds)) {
        std::cout << "Failed to extract camera bounds for frustum culling" << std::endl;
        return;
    }

    // 2. 检查视锥是否显著变化，使用缓存优化性能
    if (!frustumSignificantlyChanged(currentBounds, lastFrustumBounds)) {
        // 视锥没有显著变化，使用缓存结果
        std::cout << "Using cached frustum culling results" << std::endl;
        updateVisibilityFromCache(graphName);
        return;
    }

    std::cout << "Performing LOD3 frustum culling..." << std::endl;

    // 3. 第一阶段：基于地理网格的粗筛 (复用现有EarthGridPartition)
    std::cout << "First Stage: " << currentBounds.minLat << "/" << currentBounds.maxLat << "/"
              << currentBounds.minLon << "/" << currentBounds.maxLon << std::endl;
    std::vector<std::string> candidateNodes = earthGrid.getNodesInFrustum(
        currentBounds.minLat, currentBounds.maxLat, currentBounds.minLon, currentBounds.maxLon);

    std::cout << "Grid culling: " << candidateNodes.size() << "/" << lodNodesData[3]->size()
              << " nodes passed first stage" << std::endl;

    // 4. 第二阶段：精确视锥测试（设置节点visible属性）
    performPreciseCulling(graphParam->_camera, candidateNodes, lodNodesData[3]);

    // 5. 边剔除：基于节点可见性设置边的visible属性
    cullEdgesByVisibility(lodEdgesData[3], lodNodesData[3]);

    // 6. 输出统计信息
    int visibleNodeCount = 0, visibleEdgeCount = 0;
    for (const auto &nodePair : *lodNodesData[3]) {
        if (nodePair.second.visible)
            visibleNodeCount++;
    }
    for (const auto &edge : *lodEdgesData[3]) {
        if (edge.visible)
            visibleEdgeCount++;
    }

    std::cout << "LOD3 Frustum Culling Results:" << std::endl;
    std::cout << "  Nodes: " << visibleNodeCount << "/" << lodNodesData[3]->size() << " ("
              << (100.0 * visibleNodeCount / lodNodesData[3]->size()) << "%)" << std::endl;
    std::cout << "  Edges: " << visibleEdgeCount << "/" << lodEdgesData[3]->size() << " ("
              << (100.0 * visibleEdgeCount / lodEdgesData[3]->size()) << "%)" << std::endl;

    // 7. 更新缓存（保存visible状态）
    lastFrustumBounds = currentBounds;
    cachedNodeVisibility.clear();
    cachedEdgeVisibility.clear();

    for (const auto &nodePair : *lodNodesData[3]) {
        cachedNodeVisibility[nodePair.first] = nodePair.second.visible;
    }
    for (const auto &edge : *lodEdgesData[3]) {
        cachedEdgeVisibility[edge.id] = edge.visible;
    }

    // 8. 更新当前可见数据（基于visible属性）
    currentLevelLabels.clear();
    currentNodes.clear();

    for (const auto &nodePair : *lodNodesData[3]) {
        if (nodePair.second.visible) {
            currentLevelLabels.insert(nodePair.first);
            currentNodes.push_back(nodePair.second);
        }
    }
}
// 修改lod级别相机高度
int VIS4Earth::GraphRenderer::getCurrentLevel(double height) {
    std::cout << "height:" << height << std::endl;
//    if (height < 0)
//        return 0;
//    if (height > 2.64834e+07)
//        return 0; // 全球级
//    else if (height > 1.73736e+07)
//        return 1; // 大陆级
//    else if (height > 1.02563e+07)
//        return 2; // 国家级
//    else
//        return 3;
    if (height < 0)
        return 0;
    if (height > 2.64834e+07)
        return 0; // 全球级
    else if (height > 1.32563e+07)
        return 1; // 大陆级
    else if (height > 0.7123e+07)
        return 2; // 国家级
    else
        return 3;
}
void VIS4Earth::GraphRenderer::onComboBoxGraphTypeChanged(int index) { graphTypeIndex = index; }

void GraphRenderer::loadPointsCSV() {
    QString pointsFileName =
        QFileDialog::getOpenFileName(this, tr("Open Points CSV"), "", tr("CSV Files (*.csv)"));
    if (pointsFileName.isEmpty())
        return;

    // 设置文件路径到对应的文本框
    ui->pointsFilePath->setText(pointsFileName);
}

void VIS4Earth::GraphRenderer::loadEdgesCSV() {
    // 打开文件对话框选择边文件
    QString edgesFileName =
        QFileDialog::getOpenFileName(this, tr("Open Edges CSV"), "", tr("CSV Files (*.csv)"));
    if (edgesFileName.isEmpty())
        return;

    // 设置文件路径到对应的文本框
    ui->edgesFilePath->setText(edgesFileName);
}
void VIS4Earth::GraphRenderer::loadGeoTypeGraph() {
    //   加载城市建筑物的 OBB 数据
    VIS4Earth::CityLoader cityLoader;

    if (!cityLoader.loadBuildingsFromCSV(DATA_PATH_PREFIX "buildings_obb.csv")) {
        std::cerr << "Failed to load building data!" << std::endl;
    }

    // 设置经纬度范围（lat_min, lon_min）到（lat_max, lon_max）
    std::vector<std::pair<float, float>> latLonBounds = {
        {20.0f, -85.0f}, // 经纬度范围的左下角
        {41.0f, -74.0f}  // 经纬度范围的右上角
    };

    // 设置比例因子，将建筑物的尺寸映射到地球表面
    float scale = 1000.f; // 可调整的比例因子，根据需要调整
    int targetMaxNodeLevel = getCurrentLevel(cameraHeightPresent);
    // 绘制建筑物
    cityLoader.drawBuildings(param.grp, latLonBounds, scale, targetMaxNodeLevel);
    heightMap = cityLoader.getHeightMap();

    QString pointsFileName = ui->pointsFilePath->text();
    QString edgesFileName = ui->edgesFilePath->text();
    // QString pointsFileName, edgesFileName;
    if (pointsFileName.isEmpty()) {
        pointsFileName =
            "C:/Users/DELL/Desktop/data/graph_data/usflight/usairports_with_levels.csv";
    }
    if (edgesFileName.isEmpty()) {
        edgesFileName = "C:/Users/DELL/Desktop/data/graph_data/usflight/usroutes.csv";
    }
    QFileInfo fi(pointsFileName);
    _bundledResultDir = fi.absolutePath();

    // 清空现有的level数据
    for (int i = 0; i < 4; ++i) {
        levelIndex[i].clear();
        levelNodeIndex[i].clear();
    }

    // 读取CSV文件中的图数据
    try {
        std::string nodesFile = pointsFileName.toStdString();
        std::string edgesFile = edgesFileName.toStdString();

        auto graph = VIS4Earth::GraphLoader::LoadFromFile(nodesFile, edgesFile);
        auto nodes = std::make_shared<std::map<std::string, Node>>();
        auto edges = std::make_shared<std::vector<Edge>>();
        std::vector<osg::Vec3> colors;
        coordRange = VIS4Earth::GraphUtils::getCoordRange(graph);
        colors.resize(graph->getNodes().size());
        for (auto &col : colors) {
            col.x() = 1.f * rand() / RAND_MAX;
            col.y() = 1.f * rand() / RAND_MAX;
            col.z() = 1.f * rand() / RAND_MAX;
        }
        size_t i = 0;
        for (auto itr = graph->getNodes().begin(); itr != graph->getNodes().end(); ++itr) {
            VIS4Earth::GraphRenderer::Node node;
            node.pos = osg::Vec3(itr->second.pos.x, itr->second.pos.y, itr->second.pos.z);
            float r, g, b;
            VIS4Earth::GraphUtils::hexToRGBf(itr->second.color, r, g, b);
            node.color = osg::Vec3(r, g, b);
            node.id = itr->first;
            node.level = itr->second.level;
            if (node.level < 100) {
                levelIndex[node.level].push_back(node.id);
                levelNodeIndex[node.level].push_back(node);
            }

            nodes->emplace(std::make_pair(itr->first, node));
            earthGrid.insertNodeIntoGrid(node);
            ++i;
        }

        for (auto itr = graph->getEdges().begin(); itr != graph->getEdges().end(); ++itr) {

            edges->emplace_back();
            auto &edge = edges->back();
            edge.from = itr->sourceLabel;
            edge.to = itr->targetLabel;
            if (itr->subdivs.empty()) {
                edge.subDivs.emplace_back(osg::Vec3(itr->start.x, itr->start.y, 0.f));
                edge.subDivs.emplace_back(osg::Vec3(itr->end.x, itr->end.y, 0.f));
            } else {
                edge.subDivs.emplace_back(osg::Vec3(itr->start.x, itr->start.y, 0.f));
                for (auto &subdiv : itr->subdivs)
                    edge.subDivs.emplace_back(osg::Vec3(subdiv.x, subdiv.y, 0.f));
                edge.subDivs.emplace_back(osg::Vec3(itr->end.x, itr->end.y, 0.f));
            }
        }
        // 计算每个节点的度数
        for (const auto &edge : *edges) {
            (*nodes)[edge.from].degree++; // 增加起始节点的度数
            (*nodes)[edge.to].degree++;   // 增加结束节点的度数（如果是无向图）
        }
        // 添加图到渲染器中
        addGraph("LoadedGraph", nodes, edges);
        // 更新图渲染
        auto graphParam = getGraph("LoadedGraph");
        if (graphParam) {
            graphParam->graphTypeIndex = graphTypeIndex;
            graphParam->heightMap = heightMap;
            graphParam->setLongitudeRange(lonRng[0] * size, lonRng[1] * size);
            graphParam->setLatitudeRange(latRng[0] * size, latRng[1] * size);
            graphParam->setHeightFromCenterRange(
                static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[0],
                static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[1]);
            graphParam->setNodeGeometrySize(.02f * static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
            graphParam->setTextGeometrySize(.02f * static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
            graphParam->setCamera(param._camera);

            // 初始化LOD数据 (在GraphRenderer层面)
            this->initializeLODData(nodes, edges);
            updateActiveLOD(cameraHeightPresent);
            sceneLabels.clear();
            cameraUpdate("LoadedGraph", cameraHeightPresent);
        }
        //  初始化 UI
        QLabel *coordRangeLabel = ui->labelCurrentCoordRange; // 假设使用 ui 指针来访问 UI 元素
        QString text =
            QString("当前坐标范围:\n 最低纬度: %1, 最高纬度: %2, \n最大经度: %3, 最小经度: %4")
                .arg(coordRange.minX)
                .arg(coordRange.maxX)
                .arg(coordRange.maxY)
                .arg(coordRange.minY);
        coordRangeLabel->setText(text);
    } catch (const std::exception &e) {
        QMessageBox::critical(this, tr("Error"), tr("Failed to load graph data: %1").arg(e.what()));
    }
}

void VIS4Earth::GraphRenderer::loadNoGeoTypeGraph() {
    QString pointsFileName = ui->pointsFilePath->text();
    QString edgesFileName = ui->edgesFilePath->text();
    if (pointsFileName.isEmpty()) {
        pointsFileName = "C:/Users/DELL/Desktop/data/graph_data/grid/nodes.csv";
    }
    if (edgesFileName.isEmpty()) {
        edgesFileName = "C:/Users/DELL/Desktop/data/graph_data/grid/edges.csv";
    }

    if (pointsFileName.isEmpty() || edgesFileName.isEmpty()) {
        QMessageBox::warning(this, tr("警告"), tr("请先加载点文件和边文件"));
        return;
    }

    // 清空现有的level数据
    for (int i = 0; i < 4; ++i) {
        levelIndex[i].clear();
        levelNodeIndex[i].clear();
    }

    // 读取CSV文件中的图数据
    try {
        std::string nodesFile = pointsFileName.toStdString();
        std::string edgesFile = edgesFileName.toStdString();

        auto graph = VIS4Earth::GraphLoader::LoadFromNoGeoFile(nodesFile, edgesFile);
        auto nodes = std::make_shared<std::map<std::string, Node>>();
        auto edges = std::make_shared<std::vector<Edge>>();
        std::vector<osg::Vec3> colors;
        coordRange = VIS4Earth::GraphUtils::getCoordRange(graph);
        colors.resize(graph->getNodes().size());
        for (auto &col : colors) {
            col.x() = 1.f * rand() / RAND_MAX;
            col.y() = 1.f * rand() / RAND_MAX;
            col.z() = 1.f * rand() / RAND_MAX;
        }
        size_t i = 0;
        for (auto itr = graph->getNodes().begin(); itr != graph->getNodes().end(); ++itr) {
            VIS4Earth::GraphRenderer::Node node;
            // 初始化pos
            node.pos = osg::Vec3(itr->second.pos.x, itr->second.pos.y, 0.f);
            node.color = colors[i];
            node.id = itr->first;
            node.level = itr->second.level;
            levelIndex[node.level].push_back(node.id);
            levelNodeIndex[node.level].push_back(node);

            nodes->emplace(std::make_pair(itr->first, node));
            ++i;
        }
        for (auto itr = graph->getEdges().begin(); itr != graph->getEdges().end(); ++itr) {
            edges->emplace_back();

            auto &edge = edges->back();
            edge.from = itr->sourceLabel;
            edge.to = itr->targetLabel;
            edge.weight = itr->weight;
            if (itr->subdivs.empty()) {
                edge.subDivs.emplace_back(osg::Vec3(itr->start.x, itr->start.y, 0.f));
                edge.subDivs.emplace_back(osg::Vec3(itr->end.x, itr->end.y, 0.f));
            } else {
                edge.subDivs.emplace_back(osg::Vec3(itr->start.x, itr->start.y, 0.f));
                for (auto &subdiv : itr->subdivs)
                    edge.subDivs.emplace_back(osg::Vec3(subdiv.x, subdiv.y, 0.f));
                edge.subDivs.emplace_back(osg::Vec3(itr->end.x, itr->end.y, 0.f));
            }
        }
        for (const auto &edge : *edges) {
            (*nodes)[edge.from].degree++; // 增加起始节点的度数
            (*nodes)[edge.to].degree++;   // 增加结束节点的度数（如果是无向图）
        }
        myGraph = graph;
        myRestriction.bottomBound = 0.0;
        myRestriction.leftBound = 0.0;
        myRestriction.rightBound = 0.0;
        myRestriction.upperBound = 0.0;
        // 添加图到渲染器中
        addGraph("LoadedGraph", nodes, edges);
        auto graphParam = getGraph("LoadedGraph");
        graphParam->graphTypeIndex = graphTypeIndex;
        graphParam->heightMap = heightMap;
        graphParam->setLongitudeRange(lonRng[0] * size, lonRng[1] * size);
        graphParam->setLatitudeRange(latRng[0] * size, latRng[1] * size);
        graphParam->setHeightFromCenterRange(
            static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[0],
            static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[1]);
        graphParam->setNodeGeometrySize(.02f * static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
        graphParam->setTextGeometrySize(.02f * static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
        graphParam->setRestriction(myRestriction);
        graphParam->restrictionOFF = !restrictionOn;

        // 初始化LOD数据 (在GraphRenderer层面)
        this->initializeLODData(nodes, edges);
        graphParam->update();
        cameraUpdate("LoadedGraph", cameraHeightPresent);
        //  初始化 UI
        QLabel *coordRangeLabel = ui->labelCurrentCoordRange; // 假设使用 ui 指针来访问 UI 元素
        QString text = QString("当前坐标范围: 左: %1, 右: %2, 上: %3, 下: %4")
                           .arg(coordRange.minX)
                           .arg(coordRange.maxX)
                           .arg(coordRange.maxY)
                           .arg(coordRange.minY);
        coordRangeLabel->setText(text);

    } catch (const std::exception &e) {
        QMessageBox::critical(this, tr("Error"), tr("Failed to load graph data: %1").arg(e.what()));
    }
}

void VIS4Earth::GraphRenderer::loadAndDrawGraph() {
    if (graphTypeIndex == 0) // 假设 index 0 是 "加载固定位置的图"
    {
        loadGeoTypeGraph();         // 调用加载函数1
    } else if (graphTypeIndex == 1) // 假设 index 1 是 "无固定位置的图"
    {
        loadNoGeoTypeGraph(); // 调用加载函数2
    }
}

void VIS4Earth::GraphRenderer::applyParams() {}

void VIS4Earth::GraphRenderer::showGraph() {
    restrictionOn = false;
    myRestriction.leftBound = 00.0;
    myRestriction.rightBound = 00.0;
    myRestriction.upperBound = 00.0;
    myRestriction.bottomBound = 00.0;
    auto nodeLayouter = VIS4Earth::NodeLayouter();
    myGraph->unableNodeRestriction(myRestriction);
    nodeLayouter.setGraph(myGraph);
    nodeLayouter.setParameter(myLayoutParam);
    nodeLayouter.layout(myLayoutParam.Iteration);
    myGraph = nodeLayouter.getLayoutedGraph();
    auto existGraph = getGraph("LoadedGraph");
    if (!existGraph) {
        auto lonOffs = 1.5f * (lonRng[1] - lonRng[0]);
        lonRng[0] += lonOffs;
        lonRng[1] += lonOffs;
    }
    auto nodes = std::make_shared<std::map<std::string, Node>>();
    auto edges = std::make_shared<std::vector<Edge>>();
    std::vector<osg::Vec3> colors;
    colors.resize(myGraph->getNodes().size());
    for (auto &col : colors) {
        col.x() = 1.f * rand() / RAND_MAX;
        col.y() = 1.f * rand() / RAND_MAX;
        col.z() = 1.f * rand() / RAND_MAX;
    }
    size_t i = 0;
    for (auto itr = myGraph->getNodes().begin(); itr != myGraph->getNodes().end(); ++itr) {
        VIS4Earth::GraphRenderer::Node node;
        node.pos = osg::Vec3(itr->second.pos.x, itr->second.pos.y, 0.f);
        node.color = colors[i];
        node.id = itr->first;
        nodes->emplace(std::make_pair(itr->first, node));
        ++i;
    }

    for (auto itr = myGraph->getEdges().begin(); itr != myGraph->getEdges().end(); ++itr) {
        edges->emplace_back();

        auto &edge = edges->back();
        edge.from = itr->sourceLabel;
        edge.to = itr->targetLabel;
        if (itr->subdivs.empty()) {
            edge.subDivs.emplace_back(osg::Vec3(itr->start.x, itr->start.y, 0.f));
            edge.subDivs.emplace_back(osg::Vec3(itr->end.x, itr->end.y, 0.f));
        } else {
            edge.subDivs.emplace_back(osg::Vec3(itr->start.x, itr->start.y, 0.f));
            for (auto &subdiv : itr->subdivs)
                edge.subDivs.emplace_back(osg::Vec3(subdiv.x, subdiv.y, 0.f));
            edge.subDivs.emplace_back(osg::Vec3(itr->end.x, itr->end.y, 0.f));
        }
    }
    // 添加图到渲染器中
    addGraph("LoadedGraph", nodes, edges);
    // 更新图渲染
    auto graphParam = getGraph("LoadedGraph");
    if (graphParam) {
        graphParam->graphTypeIndex = graphTypeIndex;
        graphParam->heightMap = heightMap;
        graphParam->setLongitudeRange(lonRng[0] * size, lonRng[1] * size);
        graphParam->setLatitudeRange(latRng[0] * size, latRng[1] * size);
        graphParam->setHeightFromCenterRange(
            static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[0],
            static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[1]);
        graphParam->setNodeGeometrySize(.02f * static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
        graphParam->setTextGeometrySize(.02f * static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
        graphParam->setRestriction(myRestriction);
        graphParam->restrictionOFF = true;
        graphParam->setCamera(param._camera);
        graphParam->update();
        cameraUpdate("LoadedGraph", cameraHeightPresent);
    }
}

void VIS4Earth::GraphRenderer::showBundling() {
    // 1. 获取当前 LOD 等级
    int currentLOD = getCurrentLevel(cameraHeightPresent);

    // 2. 构造当前 LOD 对应的结果文件路径
    QString filePath = bundledFilePathForLOD(currentLOD);

    // 3. 如果标记还没 ready，先检查磁盘上是否已有该 LOD 的结果文件
    if (!_lodBundlingReady[currentLOD]) {
        if (QFile::exists(filePath)) {
            // 说明之前已经算过，直接认为 ready
            _lodBundlingReady[currentLOD] = true;
        }
    }

    // 4. 如果还没 ready 且全局 bundling 任务在运行，则只等待当前 LOD 完成
    if (!_lodBundlingReady[currentLOD] && _bundlingAllRunning && _bundlingFuture.valid()) {
        // 简单轮询等待当前 LOD 完成，不必等所有 LOD 都结束
        while (!_lodBundlingReady[currentLOD] && _bundlingAllRunning) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    if (!_lodBundlingReady[currentLOD] && !QFile::exists(filePath)) {
        if (!_bundlingAllRunning) {
            auto nodesLOD = lodNodesData[currentLOD];
            auto edgesLOD = lodEdgesData[currentLOD];
            if (nodesLOD && edgesLOD) {
                try {
                    // 1) 从当前 LOD 的 nodes/edges 构建临时 Graph
                    auto tmpGraph = buildGraphFromLODData(nodesLOD, edgesLOD);
                    if (tmpGraph) {
                        VIS4Earth::EdgeBundling edgeBundling;
                        edgeBundling.SetGraph(tmpGraph);
                        tmpGraph->buildCompatibilityListsIfNeeded();
                        edgeBundling.SetParameter(mybundlingParam);
                        edgeBundling.EdgeBundle();

                        auto bundledGraph = edgeBundling.GetLayoutedGraph();
                        saveBundledGraphToFile(bundledGraph, filePath);

                        _lodBundlingReady[currentLOD] = true;
                    }
                } catch (const std::exception &e) {
                    qDebug() << "Error in fallback bundling for LOD" << currentLOD << ":"
                             << e.what();
                }
            }
        }
    }

    // 5. 兜底检查：如果此时仍然没有 ready，且文件也不存在，就没法加载
    if (!_lodBundlingReady[currentLOD] || !QFile::exists(filePath)) {
        qDebug() << "Bundling result for LOD" << currentLOD << "is not ready. Path =" << filePath;
        return;
    }

    // 6. 拿到当前 LOD 的节点数据
    auto nodes = lodNodesData[currentLOD];
    if (!nodes) {
        qDebug() << "LOD" << currentLOD << "nodes data is null.";
        return;
    }

    // 7. 从 CSV 读取绑定后的边，转换成渲染层的 Edge（带 subDivs）
    auto edges = std::make_shared<std::vector<Edge>>();

    try {
        QFile file(filePath);
        if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QTextStream in(&file);

            while (!in.atEnd()) {
                QString line = in.readLine();
                if (line.trimmed().isEmpty())
                    continue;

                QStringList fields = line.split(",");
                if (fields.size() < 6) {
                    continue; // 格式不对，跳过
                }

                Edge e; // 这里用的是 GraphRenderer::Edge
                e.from = fields[0].toStdString();
                e.to = fields[1].toStdString();

                bool hasSubdiv = fields[2].toInt() == 1;
                int numPoints = fields[3].toInt();
                Q_UNUSED(numPoints);

                e.subDivs.clear();

                if (!hasSubdiv) {
                    // 无细分点：只有起点和终点
                    if (fields.size() >= 8) {
                        float sx = fields[4].toFloat();
                        float sy = fields[5].toFloat();
                        float ex = fields[6].toFloat();
                        float ey = fields[7].toFloat();
                        e.subDivs.emplace_back(osg::Vec3(sx, sy, 0.f));
                        e.subDivs.emplace_back(osg::Vec3(ex, ey, 0.f));
                    }
                } else {
                    // 有细分点：起点、若干 subdiv、终点，都在 CSV 里
                    // 格式：from,to,1,numPoints,x0,y0,x1,y1,...,xN,yN
                    for (int i = 4; i + 1 < fields.size(); i += 2) {
                        float x = fields[i].toFloat();
                        float y = fields[i + 1].toFloat();
                        e.subDivs.emplace_back(osg::Vec3(x, y, 0.f));
                    }
                }

                edges->push_back(e);
            }

            file.close();
        } else {
            qDebug() << "Failed to open bundled edges file:" << filePath;
            return;
        }
    } catch (const std::exception &e) {
        qDebug() << "Error loading bundled edges:" << e.what();
        return;
    }

    // 8. 为了避免和原始图重叠，我们稍微平移一下经度范围（可按需保留/修改）
    auto lonOffs = 1.5f * (lonRng[1] - lonRng[0]);
    lonRng[0] += lonOffs;
    lonRng[1] += lonOffs;

    // 添加图到渲染器中
    addGraphForBundling("LoadedGraph", nodes, edges);
    // 更新图渲染
    auto graphParam = getGraph("LoadedGraph");
    if (graphParam) {
        graphParam->currentLODLevel = getCurrentLevel(cameraHeightPresent);
        updateGraphParameters(graphParam);
        cameraUpdate("LoadedGraph", cameraHeightPresent);
    }
}

// 辅助函数：更新图形参数
void VIS4Earth::GraphRenderer::updateGraphParameters(PerGraphParam *graphParam) {
    graphParam->graphTypeIndex = graphTypeIndex;
    graphParam->heightMap = heightMap;
    graphParam->setLongitudeRange(lonRng[0] * size, lonRng[1] * size);
    graphParam->setLatitudeRange(latRng[0] * size, latRng[1] * size);
    graphParam->setHeightFromCenterRange(
        static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[0],
        static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[1]);
    graphParam->setNodeGeometrySize(.02f * static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
    graphParam->setTextGeometrySize(.02f * static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
    graphParam->setRestriction(myRestriction);
    graphParam->restrictionOFF = true;
    graphParam->setCamera(param._camera);
    graphParam->update();
}

// 辅助函数：复制图数据
void VIS4Earth::GraphRenderer::copyGraphData(std::shared_ptr<std::map<std::string, Node>> &nodes,
                                             std::shared_ptr<std::vector<Edge>> &edges) {
    std::vector<osg::Vec3> colors;
    colors.resize(myGraph->getNodes().size());
    for (auto &col : colors) {
        col.x() = 1.f * rand() / RAND_MAX;
        col.y() = 1.f * rand() / RAND_MAX;
        col.z() = 1.f * rand() / RAND_MAX;
    }

    size_t i = 0;
    for (const auto &itr : myGraph->getNodes()) {
        Node node;
        node.pos = osg::Vec3(itr.second.pos.x, itr.second.pos.y, itr.second.pos.z);
        VIS4Earth::GraphUtils::hexToRGBf(itr.second.color, node.color.x(), node.color.y(),
                                         node.color.z());
        node.id = itr.first;
        node.level = itr.second.level;
        nodes->emplace(itr.first, node);
        ++i;
    }

    for (const auto &itr : myGraph->getEdges()) {
        Edge edge;
        edge.from = itr.sourceLabel;
        edge.to = itr.targetLabel;
        if (itr.subdivs.empty()) {
            edge.subDivs.emplace_back(osg::Vec3(itr.start.x, itr.start.y, itr.start.z));
            edge.subDivs.emplace_back(osg::Vec3(itr.end.x, itr.end.y, itr.end.z));
        } else {
            edge.subDivs.emplace_back(osg::Vec3(itr.start.x, itr.start.y, itr.start.z));
            for (const auto &subdiv : itr.subdivs) {
                edge.subDivs.emplace_back(osg::Vec3(subdiv.x, subdiv.y, subdiv.z));
            }
            edge.subDivs.emplace_back(osg::Vec3(itr.end.x, itr.end.y, itr.end.z));
        }
        edges->push_back(edge);
    }
}

// 力导布局的参数
void VIS4Earth::GraphRenderer::setAttraction(double value) { myLayoutParam.attraction = value; }

void VIS4Earth::GraphRenderer::setEdgeLength(double value) { myLayoutParam.edgeLength = value; }

void VIS4Earth::GraphRenderer::setRepulsion(double value) { myLayoutParam.repulsion = value; }

void VIS4Earth::GraphRenderer::setSpringK(double value) { myLayoutParam.spring_k = value; }

void VIS4Earth::GraphRenderer::setIteration(int value) { myLayoutParam.Iteration = value; }

// 区域控制的参数
void VIS4Earth::GraphRenderer::setRegionRestriction(bool enabled) {
    restrictionOn = true;
    myRestriction.leftBound = 40.0;
    myRestriction.rightBound = 60.0;
    myRestriction.upperBound = 20.0;
    myRestriction.bottomBound = -20.0;
    auto nodeLayouter = VIS4Earth::NodeLayouter();
    nodeLayouter.setGraph(myGraph);
    nodeLayouter.setParameter(myLayoutParam);
    nodeLayouter.restrictedLayout(myRestriction, myLayoutParam.Iteration);
    myGraph = nodeLayouter.getLayoutedGraph();
    auto existGraph = getGraph("LoadedGraph");
    if (!existGraph) {
        auto lonOffs = 1.5f * (lonRng[1] - lonRng[0]);
        lonRng[0] += lonOffs;
        lonRng[1] += lonOffs;
    }
    auto nodes = std::make_shared<std::map<std::string, Node>>();
    auto edges = std::make_shared<std::vector<Edge>>();
    std::vector<osg::Vec3> colors;
    colors.resize(myGraph->getNodes().size());
    for (auto &col : colors) {
        col.x() = 1.f * rand() / RAND_MAX;
        col.y() = 1.f * rand() / RAND_MAX;
        col.z() = 1.f * rand() / RAND_MAX;
    }
    size_t i = 0;
    for (auto itr = myGraph->getNodes().begin(); itr != myGraph->getNodes().end(); ++itr) {
        VIS4Earth::GraphRenderer::Node node;
        node.pos = osg::Vec3(itr->second.pos.x, itr->second.pos.y, 0.f);
        node.color = colors[i];

        nodes->emplace(std::make_pair(itr->first, node));
        ++i;
    }

    for (auto itr = myGraph->getEdges().begin(); itr != myGraph->getEdges().end(); ++itr) {
        edges->emplace_back();

        auto &edge = edges->back();
        edge.from = itr->sourceLabel;
        edge.to = itr->targetLabel;
        if (itr->subdivs.empty()) {
            edge.subDivs.emplace_back(osg::Vec3(itr->start.x, itr->start.y, 0.f));
            edge.subDivs.emplace_back(osg::Vec3(itr->end.x, itr->end.y, 0.f));
        } else {
            edge.subDivs.emplace_back(osg::Vec3(itr->start.x, itr->start.y, 0.f));
            for (auto &subdiv : itr->subdivs)
                edge.subDivs.emplace_back(osg::Vec3(subdiv.x, subdiv.y, 0.f));
            edge.subDivs.emplace_back(osg::Vec3(itr->end.x, itr->end.y, 0.f));
        }
    }
    // 添加图到渲染器中
    addGraph("LoadedGraph", nodes, edges);
    // 更新图渲染
    auto graphParam = getGraph("LoadedGraph");
    if (graphParam) {
        graphParam->graphTypeIndex = graphTypeIndex;
        graphParam->heightMap = heightMap;
        graphParam->setLongitudeRange(lonRng[0] * size, lonRng[1] * size);
        graphParam->setLatitudeRange(latRng[0] * size, latRng[1] * size);
        graphParam->setHeightFromCenterRange(
            static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[0],
            static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[1]);
        graphParam->setNodeGeometrySize(.02f * static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
        graphParam->setTextGeometrySize(.02f * static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
        graphParam->setRestriction(myRestriction);
        graphParam->restrictionOFF = !restrictionOn;
        graphParam->update();
        cameraUpdate("LoadedGraph", cameraHeightPresent);
    }
}

void VIS4Earth::GraphRenderer::setMinX(double value) { myRestriction.leftBound = value; }

void VIS4Earth::GraphRenderer::setMaxX(double value) { myRestriction.rightBound = value; }

void VIS4Earth::GraphRenderer::setMinY(double value) { myRestriction.bottomBound = value; }

void VIS4Earth::GraphRenderer::setMaxY(double value) { myRestriction.upperBound = value; }


void VIS4Earth::GraphRenderer::onHighlightFlowButtonClicked() {
    auto graphParam = getGraph("LoadedGraph");
    graphParam->startHighlightAnimation();
}

void VIS4Earth::GraphRenderer::onTextureFlowButtonClicked() {
    auto graphParam = getGraph("LoadedGraph");
    graphParam->startTextureAnimation();
}

void VIS4Earth::GraphRenderer::onStarFlowButtonClicked() {
    auto graphParam = getGraph("LoadedGraph");
    graphParam->startStarAnimation();
}

// 边绑定的参数
void VIS4Earth::GraphRenderer::onGlobalSpringConstantChanged(double value) {
    // 处理全局弹簧常数变化的逻辑
    mybundlingParam.K = value;
}

void VIS4Earth::GraphRenderer::onNumberOfIterationsChanged(int value) {
    // 处理迭代次数变化的逻辑
    mybundlingParam.I = value;
}

void VIS4Earth::GraphRenderer::onRemainingIterationsChanged(int value) {
    // 处理剩余迭代次数变化的逻辑
    mybundlingParam.iter = value;
}

void VIS4Earth::GraphRenderer::onCyclesLeftChanged(int value) {
    // 处理剩余循环数变化的逻辑
    mybundlingParam.cycles = value;
}

void VIS4Earth::GraphRenderer::onCompatibilityThresholdChanged(double value) {
    // 处理兼容性阈值变化的逻辑
    mybundlingParam.compatibilityThreshold = value;
}

void VIS4Earth::GraphRenderer::onSmoothWidthChanged(double value) {
    // 处理平滑宽度变化的逻辑
    mybundlingParam.smoothWidth = value;
}

void VIS4Earth::GraphRenderer::onDisplacementChanged(double value) {
    // 处理位移变化的逻辑
    mybundlingParam.S = value;
}

void VIS4Earth::GraphRenderer::onEdgeDistanceChanged(double value) {
    // 处理边距离变化的逻辑
    mybundlingParam.edgeDistance = value;
}

void VIS4Earth::GraphRenderer::onGravitationIsOnToggled(bool checked) {
    // 处理引力开关变化的逻辑
    mybundlingParam.gravitationIsOn = checked;
}

void VIS4Earth::GraphRenderer::onGravitationCenterXChanged(double value) {
    // 处理引力中心X变化的逻辑
    mybundlingParam.gravitationCenter.x = value;
}

void VIS4Earth::GraphRenderer::onGravitationCenterYChanged(double value) {
    // 处理引力中心Y变化的逻辑
    mybundlingParam.gravitationCenter.y = value;
}

void VIS4Earth::GraphRenderer::onGravitationCenterZChanged(double value) {
    // 处理引力中心Z变化的逻辑
    mybundlingParam.gravitationCenter.z = value;
}

void VIS4Earth::GraphRenderer::onGravitationExponentChanged(double value) {
    // 处理引力指数变化的逻辑
    mybundlingParam.gravitationExponent = value;
}

void VIS4Earth::GraphRenderer::onEdgeWeightThresholdChanged(double value) {
    // 处理边权重阈值变化的逻辑
    mybundlingParam.edgeWeightThreshold = value;
}

void VIS4Earth::GraphRenderer::onEdgePercentageThresholdChanged(double value) {
    // 处理边百分比阈值变化的逻辑
    mybundlingParam.edgePercentageThreshold = value;
}

void VIS4Earth::GraphRenderer::onFontSizeSliderValueChanged(int value) {
    // Update the label text
    ui->fontSizeLabel->setText(QString("字体大小: %1").arg(value));
    auto graphParam = getGraph("LoadedGraph");
    if (graphParam) {
        graphParam->setTextGeometrySize(value / 12.0 * .02f *
                                        static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
        graphParam->update();
        cameraUpdate("LoadedGraph", cameraHeightPresent);
    }
}

osg::Image *VIS4Earth::GraphRenderer::PerGraphParam::createLineDataTexture() {
    int texWidth = edges->size();
    int texHeight = 4; // 使用4行存储不同参数

    osg::Image *image = new osg::Image;
    image->allocateImage(texWidth, texHeight, 1, GL_RGBA, GL_FLOAT);
    image->setInternalTextureFormat(GL_RGBA32F_ARB);

    // 初始填充0
    memset(image->data(), 0, texWidth * texHeight * 4 * sizeof(float));

    // 初始化静态数据(起点/终点)
    float *data = reinterpret_cast<float *>(image->data());
    int x = 0;
    for (auto &edge : *edges) {
        // 第0行: 动画参数 (y=0.0)
        int paramPos = (0 * texWidth + x) * 4;
        data[paramPos] = edge.highlightPos; // highlightPos
        data[paramPos + 1] = edge.speed;    // speed
        data[paramPos + 2] = 0.0f;          // 保留
        data[paramPos + 3] = 0.0f;          // 保留

        // 第1行: 起点 (y=1)
        int startPos = (1 * texWidth + x) * 4;
        auto realPos = GraphUtils::vec3ToSphere(nodes->at(edge.from).pos);
        data[startPos] = realPos.x();
        data[startPos + 1] = realPos.y();
        data[startPos + 2] = realPos.z();
        data[startPos + 3] = 1.0f;

        // 第2行: 终点 (y=2)
        int endPos = (2 * texWidth + x) * 4;
        auto realEndPos = GraphUtils::vec3ToSphere(nodes->at(edge.to).pos);
        data[endPos] = realEndPos.x();
        data[endPos + 1] = realEndPos.y();
        data[endPos + 2] = realEndPos.z();
        data[endPos + 3] = 1.0f;
        x++;
    }

    return image;
}


void VIS4Earth::GraphRenderer::PerGraphParam::startHighlightAnimation() {
    if (isAnimating) {
        // 停止动画
        if (lineGeometry) {
            lineGeometry->setUpdateCallback(nullptr);
            //// 重置几何体颜色
            // osg::Geometry *geom = dynamic_cast<osg::Geometry *>(lineGeode->getDrawable(0));
            if (lineGeometry) {
                osg::Vec4Array *colors = new osg::Vec4Array(1);
                (*colors)[0] = osg::Vec4(0.8f, 0.6f, 0.2f, 0.25f);
                lineGeometry->setColorArray(colors, osg::Array::BIND_OVERALL);
                lineGeometry->dirtyDisplayList();
            }
            // 2. 使用 ref_ptr 确保线程安全
            osg::ref_ptr<osg::StateSet> ss = lineGeometry->getOrCreateStateSet();
            if (ss.valid()) {
                // 3. 创建新的状态集合
                osg::ref_ptr<osg::StateSet> newSS = new osg::StateSet(*ss);

                // 4. 在新的状态集合上进行修改
                newSS->removeTextureAttribute(0, osg::StateAttribute::TEXTURE);
                newSS->removeUniform("uHighlightColor");
                newSS->removeAttribute(osg::StateAttribute::PROGRAM);
                newSS->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

                // 5. 原子性地替换状态集合
                lineGeometry->setStateSet(newSS);
            }
            for (auto &edge : *edges) {
                edge.highlightPos = 0.0f;
                edge.speed = 0.3f; // 重置速度
            }
        }
        isAnimating = false;
    } else {
        // 开始动画
        if (lineGeode && lineGeometry) {
            // 创建并设置数据纹理
            osg::ref_ptr<osg::Image> lineDataImage = createLineDataTexture();
            osg::ref_ptr<osg::Texture2D> lineDataTex = new osg::Texture2D;
            lineDataTex->setImage(lineDataImage);
            lineDataTex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
            lineDataTex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);
            lineDataTex->setResizeNonPowerOfTwoHint(false);

            // 设置着色器和状态
            auto arrowStates = lineGeometry->getOrCreateStateSet();
            arrowStates->setAttributeAndModes(
                VIS4Earth::GraphAnimation::createTextureBasedShaderProgram(edges->size()),
                osg::StateAttribute::ON);

            // 启用透明度混合
            osg::BlendFunc *blendFunc = new osg::BlendFunc();
//            blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE); // 加法混合
            blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // alpha混合
            arrowStates->setAttributeAndModes(blendFunc, osg::StateAttribute::ON);

            // 禁用深度写入但保留深度测试
            osg::Depth *depth = new osg::Depth();
//            depth->setWriteMask(false);
            depth->setFunction(osg::Depth::LEQUAL);
            arrowStates->setAttributeAndModes(depth, osg::StateAttribute::ON);

            // 设置渲染顺序
            arrowStates->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            arrowStates->setRenderBinDetails(10, "DepthSortedBin");

            // 绑定纹理和uniform变量
            arrowStates->setTextureAttributeAndModes(0, lineDataTex, osg::StateAttribute::ON);
            arrowStates->addUniform(new osg::Uniform("uLineDataTex", 0));
            arrowStates->addUniform(
                new osg::Uniform("uTotalLines", static_cast<float>(edges->size())));
            arrowStates->addUniform(new osg::Uniform("uHighlightWidth", 0.05f)); // 增加高光宽度
            arrowStates->addUniform(new osg::Uniform(
                "uHighlightColor", osg::Vec4(1.2f, 1.2f, 1.2f, 1.0f))); // 增强高光亮度

            // 发光和透明度控制参数
            arrowStates->addUniform(new osg::Uniform("uGlowIntensity", 0.2f)); // 适当降低发光强度
            arrowStates->addUniform(new osg::Uniform("uGlobalAlpha", 0.2f));   // 增加全局透明度
            arrowStates->addUniform(new osg::Uniform("uLineThickness", 1.5f)); // 减小线条粗细

            // 初始化每条边的动画参数
            for (auto &edge : *edges) {
                edge.highlightPos = 0.0f;
                edge.speed = 0.3f; // 降低移动速度，使高光更容易观察
            }

            // 设置动画回调
            lineGeometry->setUpdateCallback(
                new VIS4Earth::GraphAnimation::TextureBasedAnimationCallback(lineDataImage, edges));
        }
        isAnimating = true;
    }
}

void VIS4Earth::GraphRenderer::PerGraphParam::startTextureAnimation() {

    if (lineGeode && lineGeometry) {
        if (isAnimating) {
            // 当前正在动画中，结束动画
            lineGeometry->setUpdateCallback(nullptr); // 将颜色设置为初始颜色
            // 2. 直接重置几何体颜色（强制GPU更新）
            if (lineGeometry) {
                osg::Vec4Array *colors = new osg::Vec4Array(1);
                (*colors)[0] = osg::Vec4(0.8f, 0.6f, 0.2f, 0.15f);
                lineGeometry->setColorArray(colors, osg::Array::BIND_OVERALL);
                lineGeometry->dirtyDisplayList(); // 比dirtyDisplayList()更彻底
            }

            osg::ref_ptr<osg::StateSet> ss = lineGeometry->getOrCreateStateSet();
            if (ss.valid()) {
                // 3. 创建新的状态集合
                osg::ref_ptr<osg::StateSet> newSS = new osg::StateSet(*ss);

                // 4. 在新的状态集合上进行修改
                newSS->removeTextureAttribute(0, osg::StateAttribute::TEXTURE);
                newSS->removeUniform("uHighlightColor");
                newSS->removeAttribute(osg::StateAttribute::PROGRAM);
                newSS->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

                // 5. 原子性地替换状态集合
                lineGeometry->setStateSet(newSS);
            }
            isAnimating = false;
            // update(); // 重新绘制图形
        } else {
            // 当前没有动画，开始动画
            if (lineGeode && lineGeometry) {

                osg::ref_ptr<osg::Image> lineDataImage = createLineDataTexture();
                osg::ref_ptr<osg::Texture2D> lineDataTex = new osg::Texture2D;
                lineDataTex->setImage(lineDataImage);
                lineDataTex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
                lineDataTex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);
                lineDataTex->setResizeNonPowerOfTwoHint(false);

                // 禁用光照
                auto arrowStates = lineGeometry->getOrCreateStateSet();
                arrowStates->setAttributeAndModes(
                    VIS4Earth::GraphAnimation::createTextureBasedShaderProgramColorFlow(edges->size()),
                    osg::StateAttribute::ON);

                // 绑定纹理
                arrowStates->setTextureAttributeAndModes(0, lineDataTex, osg::StateAttribute::ON);
                arrowStates->addUniform(new osg::Uniform("uLineDataTex", 0));
                arrowStates->addUniform(
                    new osg::Uniform("uTotalLines", static_cast<float>(edges->size())));
                arrowStates->addUniform(new osg::Uniform("uHighlightWidth", 20.f));
                arrowStates->addUniform(
                    new osg::Uniform("uHighlightColor", osg::Vec4(246.f, 66.f, 14.f, 1.0f)));
                lineGeometry->setUpdateCallback(
                    new VIS4Earth::GraphAnimation::TextureBasedAnimationColorCallback(lineDataImage,
                                                                                      edges));
            }
            isAnimating = true;
        }
    }
}

void VIS4Earth::GraphRenderer::PerGraphParam::startStarAnimation() {

    if (lineGeode && lineGeometry) {
        if (isAnimating) {
            // 当前正在动画中，结束动画
            lineGeometry->setUpdateCallback(nullptr); // 将颜色设置为初始颜色
            // 2. 直接重置几何体颜色（强制GPU更新）
            if (lineGeometry) {
                osg::Vec4Array *colors = new osg::Vec4Array(1);
                (*colors)[0] = osg::Vec4(0.8f, 0.6f, 0.2f, 0.15f);
                lineGeometry->setColorArray(colors, osg::Array::BIND_OVERALL);
                lineGeometry->dirtyDisplayList(); // 比dirtyDisplayList()更彻底
            }

            osg::ref_ptr<osg::StateSet> ss = lineGeometry->getOrCreateStateSet();
            if (ss.valid()) {
                // 3. 创建新的状态集合
                osg::ref_ptr<osg::StateSet> newSS = new osg::StateSet(*ss);

                // 4. 在新的状态集合上进行修改
                newSS->removeTextureAttribute(0, osg::StateAttribute::TEXTURE);
                newSS->removeUniform("uHighlightColor");
                newSS->removeAttribute(osg::StateAttribute::PROGRAM);
                newSS->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

                // 5. 原子性地替换状态集合
                lineGeometry->setStateSet(newSS);
            }
            for (auto &edge : *edges) {
                edge.highlightPos = 0.0f;
                edge.speed = 0.3f; // 重置速度
            }
            isAnimating = false;
            // update(); // 重新绘制图形
        } else {
            // 当前没有动画，开始动画
            if (lineGeode && lineGeometry) {

                osg::ref_ptr<osg::Image> lineDataImage = createLineDataTexture();
                osg::ref_ptr<osg::Texture2D> lineDataTex = new osg::Texture2D;
                lineDataTex->setImage(lineDataImage);
                lineDataTex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
                lineDataTex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);
                lineDataTex->setResizeNonPowerOfTwoHint(false);

                // 禁用光照
                auto arrowStates = lineGeometry->getOrCreateStateSet();
                arrowStates->setAttributeAndModes(
                    VIS4Earth::GraphAnimation::createTextureBasedShaderProgramStarFlow(edges->size()),
                    osg::StateAttribute::ON);

                // 绑定纹理
                arrowStates->setTextureAttributeAndModes(0, lineDataTex, osg::StateAttribute::ON);
                arrowStates->addUniform(new osg::Uniform("uLineDataTex", 0));
                arrowStates->addUniform(
                    new osg::Uniform("uTotalLines", static_cast<float>(edges->size())));
                arrowStates->addUniform(new osg::Uniform("uHighlightWidth", 20.f));
                arrowStates->addUniform(
                    new osg::Uniform("uHighlightColor", osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)));
                lineGeometry->setUpdateCallback(
                    new VIS4Earth::GraphAnimation::TextureBasedAnimationCallback(lineDataImage,
                                                                                 edges));
            }
            isAnimating = true;
        }
    }
}
void VIS4Earth::GraphRenderer::PerGraphParam::update() {
    if (!_satelliteModel) {
        //AppEnv& env = AppEnv::instance();
        //String data_dir = env.getPath(AppEnv::DATA_DIR);
        //std::string str_path = data_dir.c_str();
        //str_path = str_path + "/graph/graph_data/test_data/satellite_obj.obj";
        _satelliteModel = osgDB::readNodeFile(DATA_PATH_PREFIX "satellite_obj.obj");
        //_satelliteModel = osgDB::readNodeFile(str_path);
        if (!_satelliteModel) {
            std::cout << "failed!" << std::endl;
        } else {
            std::cout << "success!" << std::endl;
        }
    }
    if (!edgeNodegrp) {
        edgeNodegrp = new osg::Group;
    }
    edgeNodegrp->removeChildren(0, edgeNodegrp->getNumChildren());
    grp->removeChild(edgeNodegrp);
    auto tessl = new osg::TessellationHints;
    tessl->setDetailRatio(1.f);
    std::map<std::string, osg::ShapeDrawable *> osgNodes;
    std::vector<osg::ref_ptr<osgText::Text>> textNodes;

    if (!sats) {
        sats = new osg::Group;
    }
    bool drawSats = false;
    if (sats->getNumChildren() == 0) {
        drawSats = true;
    }
    osg::ref_ptr<osg::Vec3Array> allNodePositions = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array> allNodeColors = new osg::Vec4Array;
    for (auto itr = nodes->begin(); itr != nodes->end(); ++itr) {
        if (!itr->second.visible)
            continue; // 只处理可见节点
        // osg::Vec4 color = generateColor(static_cast<float>(itr->second.cluster));
        osg::Vec4 color = osg::Vec4(itr->second.color, 1.0f);

        if (!restrictionOFF) {
            if (itr->second.pos.x() >= restriction.leftBound &&
                itr->second.pos.x() <= restriction.rightBound &&
                itr->second.pos.y() >= restriction.bottomBound &&
                itr->second.pos.y() <= restriction.upperBound) {
                color = osg::Vec4(1.0f, 0.0f, 0.0f, 0.5f); // 设置边框内的点为半透明白色
            }
        }
        auto p = itr->second.pos;
        if (p.z() < 100.f) {
            p.z() = getBuildingHeightAtLatLon(p.x(), p.y());
        }
        p = GraphUtils::vec3ToSphere(p);
        if (drawSats && itr->second.level == 100 && _satelliteModel.valid()) {
            osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;

            // 设置状态
            osg::ref_ptr<osg::Node> modelClone =
                dynamic_cast<osg::Node *>(_satelliteModel->clone(osg::CopyOp::DEEP_COPY_ALL));

            float modelScale = 40000.0f;
            osg::Matrix scale = osg::Matrix::scale(modelScale, modelScale, modelScale);
            osg::Matrix translate = osg::Matrix::translate(p);
            transform->setMatrix(scale * translate); // 先缩放后平移

            transform->addChild(modelClone);
            sats->addChild(transform);

        } else {
            // 原有的球体绘制逻辑
            allNodePositions->push_back(p);
            allNodeColors->push_back(color);
            /*int scale = 0.050f;
            auto sphere = new osg::ShapeDrawable(
                new osg::Sphere(p, itr->second.size * scale * nodeGeomSize), tessl);
            osg::ref_ptr<osg::Vec3Array> centerData = new osg::Vec3Array;
            centerData->push_back(itr->second.pos);
            sphere->setUserData(centerData);
            sphere->setColor(color);
            edgeNodegrp->addChild(sphere);*/
        }
    }
    grp->addChild(sats);
    auto geom = new osg::Geometry;
    geom->setVertexArray(allNodePositions);
    geom->setColorArray(allNodeColors, osg::Array::BIND_PER_VERTEX);
    geom->addPrimitiveSet(
        new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, allNodePositions->size()));

    auto geode = new osg::Geode;
    geode->addDrawable(geom);
    auto ss = geode->getOrCreateStateSet();
    osg::ref_ptr<osg::Point> pointSize = new osg::Point;
    pointSize->setSize(3.0f); // 设置点大小（像素单位）
    ss->setAttributeAndModes(pointSize, osg::StateAttribute::ON);

    edgeNodegrp->addChild(geode);
    // grp->addChild(edgeNodegrp);
    auto states = edgeNodegrp->getOrCreateStateSet();
    auto matr = new osg::Material;
    matr->setColorMode(osg::Material::DIFFUSE);
    states->setAttributeAndModes(matr, osg::StateAttribute::ON);
    states->setMode(GL_LIGHTING, osg::StateAttribute::ON);
    states->setMode(GL_BLEND, osg::StateAttribute::ON); // 开启混合模式
    if (!mUseNewRenderer) {
        // 原有的渲染逻辑

        auto segVerts = new osg::Vec3Array;
        auto segCols = new osg::Vec4Array;
        osg::ref_ptr<osg::FloatArray> lineIDs = new osg::FloatArray;
        int lineID = 0;
        int totalNum = 5;
        // 设置基准参数
        const float BASE_LENGTH = 5000.0f; // 基准长度(km)，可以根据实际情况调整
        const int BASE_SEGMENTS = 5;       // 基准长度对应的细分段数
        const int MIN_SEGMENTS = 5;        // 最小细分段数
        const int MAX_SEGMENTS = 20;       // 最大细分段数

        for (auto &edge : *edges) {
            if (!edge.visible)
                continue; // 只处理可见边

            osg::Vec4 prevColor = osg::Vec4(nodes->at(edge.from).color, 0.5f);
            auto dCol = osg::Vec4(nodes->at(edge.to).color, 1.f) - prevColor;
            dCol /= (edge.subDivs.size() == 1 ? 1 : edge.subDivs.size() - 1);

            osg::Vec3 prevPos = nodes->at(edge.from).pos;
            osg::Vec3 startPos = GraphUtils::vec3ToSphere(prevPos); // 起点
            osg::Vec3 endPos = nodes->at(edge.to).pos;
            endPos = GraphUtils::vec3ToSphere(endPos);

            // 总点数，包括起点、所有细分点和终点
            // 每段有 totalNum 个插值点，总段数是 subDivs.size() - 1
            // 再加上原始点的数量 subDivs.size()
            // 获取起点和终点的经纬度
            float lat1 = osg::DegreesToRadians(edge.subDivs.front().x()); // 起点纬度
            float lon1 = osg::DegreesToRadians(edge.subDivs.front().y()); // 起点经度
            float lat2 = osg::DegreesToRadians(edge.subDivs.back().x());  // 终点纬度
            float lon2 = osg::DegreesToRadians(edge.subDivs.back().y());  // 终点经度

            // 使用Haversine公式计算大圆距离
            float dlat = lat2 - lat1;
            float dlon = lon2 - lon1;
            float a = std::sin(dlat / 2) * std::sin(dlat / 2) +
                      std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2) * std::sin(dlon / 2);
            float c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
            float edgeLength = 6371.0f * c; // 6371km是地球平均半径，得到的距离单位是km

            // 根据实际地理距离与基准长度的比例计算细分段数
            int totalNum = static_cast<int>(BASE_SEGMENTS * (edgeLength / BASE_LENGTH));

            // 限制在合理范围内
            totalNum = std::max(MIN_SEGMENTS, std::min(MAX_SEGMENTS, totalNum));

            size_t totalPoints = (edge.subDivs.size() - 1) * (totalNum) + edge.subDivs.size();

            // 首先得到采样点的高度,进而计算这条边上的最大高度
            // 计算每个路径上的maxHeight 用Asin(pi*x)绘制
            float maxRequiredAmplitude = 0.0f;
            std::vector<float> heightArray;
            float maxHeightInArray = 0.0;
            int maxLocation = 0;
            for (int i = 1; i < totalPoints + 1; i++) {
                float t = static_cast<float>(i) / (totalPoints);
                osg::Vec3 interpolatedPos;
                interpolatedPos.x() = prevPos.x() * (1.0f - t) + edge.subDivs.back().x() * t;
                interpolatedPos.y() = prevPos.y() * (1.0f - t) + edge.subDivs.back().y() * t;
                interpolatedPos.z() =
                    getBuildingHeightAtLatLon(interpolatedPos.x(), interpolatedPos.y());
                heightArray.push_back(interpolatedPos.z());
                maxHeightInArray = std::max(maxHeightInArray, interpolatedPos.z());
                if (interpolatedPos.z() >= maxHeightInArray) {
                    maxLocation = i;
                }
            }
            {
                osg::Vec3 prevInterpolatedPos = prevPos;     // 初始插值位置
                osg::Vec4 prevInterpolatedColor = prevColor; // 初始插值颜色
                prevInterpolatedPos.z() =
                    getBuildingHeightAtLatLon(prevInterpolatedPos.x(), prevInterpolatedPos.y());

                for (int i = 1; i < (totalNum / 2) + 2; i++) {
                    float t = static_cast<float>(i) / ((totalNum / 2) + 2);
                    float sinValue = std::sin(osg::PI * t); // 计算 sin(π * x)

                    // 计算出对应位置所需的振幅 A，确保 A * sin(π * x) >= arr[i]
                    if (heightArray[i] < 1.f)
                        continue;
                    float requiredAmplitude = heightArray[i] / sinValue;
                    maxRequiredAmplitude = std::max(maxRequiredAmplitude, requiredAmplitude);
                }

                // 方法2：全局控制的最大高度,绘制sin曲线
                float maxHeight = std::max(maxRequiredAmplitude, 100000.f);
                edge.maxHeight = maxHeight;
                int count = 0;

                for (size_t i = 1; i < edge.subDivs.size(); ++i) {
                    osg::Vec3 currentPos = edge.subDivs[i];
                    osg::Vec4 currentColor = prevColor + dCol;
                    // 在 prevPos 和 currentPos 之间插入细分点
                    for (int j = 0; j <= totalNum; ++j) { // 包含起点和终点
                        float t = static_cast<float>(j) / static_cast<float>(totalNum);

                        osg::Vec3 interpolatedPos;
                        interpolatedPos.x() = prevPos.x() * (1.0f - t) + currentPos.x() * t;
                        interpolatedPos.y() = prevPos.y() * (1.0f - t) + currentPos.y() * t;
                        float globalT =
                            ((i - 1) * (totalNum + 1) + (j + 1)) / static_cast<float>(totalPoints);
                        interpolatedPos.z() = std::max(
                            getBuildingHeightAtLatLon(interpolatedPos.x(), interpolatedPos.y()),
                            float(maxHeight * sin(osg::PI * globalT))); // 平滑高度变化
                        // interpolatedPos.z() = 0.f;
                        osg::Vec4 interpolatedColor = prevColor * (1.0f - t) + currentColor * t;

                        if (j >= 0) {
                            segVerts->push_back(GraphUtils::vec3ToSphere(prevInterpolatedPos));
                            segCols->push_back(osg::Vec4(15 / 255.f, 176 / 255.0f, 1.f, 0.8f));
                            segVerts->push_back(GraphUtils::vec3ToSphere(interpolatedPos));
                            segCols->push_back(osg::Vec4(15 / 255.f, 176 / 255.0f, 1.f, 0.8f));
                            lineIDs->push_back(static_cast<float>(lineID));
                            lineIDs->push_back(static_cast<float>(lineID));
                            count++;
                        }

                        prevInterpolatedPos = interpolatedPos;
                        prevInterpolatedColor = interpolatedColor;
                    }

                    prevPos = currentPos;
                    prevColor = currentColor;
                }
                count++;
            }
            lineID++;
        }

        if (!arrowFlowEnabled) {
            auto geom = new osg::Geometry;
            geom->setVertexArray(segVerts);
            geom->setVertexAttribArray(0, segVerts, osg::Array::BIND_PER_VERTEX);
            geom->setVertexAttribArray(1, lineIDs, osg::Array::BIND_PER_VERTEX);
            this->segVerts = segVerts;
            geom->setColorArray(segCols);
            geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

            auto states = geom->getOrCreateStateSet();
            states->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
            // states->setMode(GL_BLEND, osg::StateAttribute::ON); // 开启混合模式
            geom->addPrimitiveSet(
                new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, segVerts->size()));
            auto lw = new osg::LineWidth(1.f);
            states->setAttributeAndModes(lw, osg::StateAttribute::ON);
            geom->setUseVertexBufferObjects(true);
            /*osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc();
            blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            states->setAttributeAndModes(blendFunc, osg::StateAttribute::ON);*/

            //// 启用混合（Blending）以支持透明度
            // geom->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);

            //// 设置混合函数
            // geom->getOrCreateStateSet()->setAttributeAndModes(
            //     new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
            //     osg::StateAttribute::ON);

            //// 设置渲染顺序以确保透明物体正确渲染
            // geom->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            // geom->getOrCreateStateSet()->setAttributeAndModes(
            //     new osg::Depth(osg::Depth::LESS, 0.0, 1.0, false), osg::StateAttribute::ON);

            auto geode = new osg::Geode;
            geode->addDrawable(geom);

            // 保存 Geode 和 Geometry
            lineGeode = geode;
            lineGeometry = geom;

            edgeNodegrp->addChild(geode);
            grp->addChild(edgeNodegrp);
        }

    } else {
        // 新的VBO和Shader渲染逻辑
        {
            mEdgeGeometry = new osg::Geometry;
            mEdgeGeode = new osg::Geode;
            mEdgeGeode->addDrawable(mEdgeGeometry);
            lineGeode = mEdgeGeode;
            lineGeometry = mEdgeGeometry;
            edgeNodegrp->addChild(mEdgeGeode);
            grp->addChild(edgeNodegrp);
            // 初始化着色器
            //initEdgeShaders();
            initEdgeShaders_GPUInterpolation();
        }

        // 更新VBO数据
        // LOD 3: 使用原有逻辑，完整显示所有边
        updateEdgeVBO();
    }
}

void VIS4Earth::GraphRenderer::PerGraphParam::initEdgeShaders_GPUInterpolation() {
    if (!mEdgeProgram) {
        mEdgeProgram = new osg::Program;

        // 顶点着色器
        const char *vertSource = R"(
            #version 120

            varying vec4 vColorFrom;
            varying vec4 vColorTo;
            varying float vWeight;
            varying float vLineID;
            varying float vDistanceToCamera;

            attribute vec3 vertexPosition;   // 基线位置（球面上的点）
            attribute float lineID;          // 显式绑定 slot 1（在 C++ 中 addBindAttribLocation）
            attribute float segmentParam;    // 细分参数 t \in [0,1]，绑定到 slot 3
            attribute float maxHeight;       // 当前曲线的最大高度，绑定到 slot 4

            void main() {
                // 计算弧线高度：h = sin(pi * t) * maxHeight
                float t = clamp(segmentParam, 0.0, 1.0);
                float arcH = sin(3.1415926 * t) * maxHeight;

                // 沿球心方向抬高：基线点在球面上，dir 为法向
                vec3 dir = normalize(vertexPosition);
                vec3 liftedPos = vertexPosition + dir * arcH;

                vec4 worldPos = gl_ModelViewMatrix * vec4(liftedPos, 1.0);
                gl_Position = gl_ProjectionMatrix * worldPos;

                vColorFrom = gl_MultiTexCoord0;
                vColorTo   = gl_MultiTexCoord1;
                vWeight    = gl_MultiTexCoord2.x;
                vLineID = lineID; // 将属性传给片元着色器
                
                // 计算到相机的距离用于透明度调节
                vDistanceToCamera = length(worldPos.xyz);
            }
        )";

        // 片段着色器 - 添加透明度和发光效果
        const char *fragSource = R"(
            #version 120
            varying vec4 vColorFrom;
            varying vec4 vColorTo;
            varying float vWeight;
            varying float vLineID;
            varying float vDistanceToCamera;

            uniform bool uEnableHighlight;
            uniform sampler2D uLineDataTex;
            uniform float uTotalLines;
            uniform vec4 uHighlightColor;
            uniform float uGlobalAlpha;      // 全局透明度控制
            uniform float uGlowIntensity;    // 发光强度控制
            uniform float uLineThickness;    // 线条粗细影响发光范围

            void main() {
                // 基础渐变颜色计算
                float t = fract(gl_FragCoord.x * 0.001); // 简化的插值参数
                vec4 baseColor = mix(vColorFrom, vColorTo, t);
                
                // 基于权重的亮度增强
                float weightFactor = clamp(vWeight * 0.1, 0.3, 2.0); // 权重影响亮度
                baseColor.rgb *= weightFactor;
                
                // 距离衰减透明度
                float distanceFactor = 1.0 / (1.0 + vDistanceToCamera * 0.0000001);
                float baseAlpha = clamp(distanceFactor, 0.1, 1.0);
                
                // 发光效果 - 基于线条中心的距离衰减
                float centerDistance = abs(gl_FragCoord.y - floor(gl_FragCoord.y + 0.5)); // 到线条中心的距离
                float glowRadius = uLineThickness * 2.0; // 发光半径
                float glowFactor = 1.0 - smoothstep(0.0, glowRadius, centerDistance);
                glowFactor = pow(glowFactor, 2.0); // 增强发光衰减
                
                // 组合发光效果
                vec3 glowColor = baseColor.rgb * uGlowIntensity * glowFactor;
                vec3 finalColor = baseColor.rgb + glowColor;
                
                // 高亮动画效果（保留原有功能）
                if (uEnableHighlight) {
                    float lineID = vLineID;
                    float texX = (lineID + 0.5) / uTotalLines;
                    float highlightPos = texture2D(uLineDataTex, vec2(texX, 0.0)).r;

                    float highlightWidth = 0.05;
                    float highlightIntensity = 0.0;

                    if (t >= highlightPos && t <= highlightPos + highlightWidth) {
                        float posInHighlight = (t - highlightPos) / highlightWidth;
                        highlightIntensity = smoothstep(0.0, 1.0, posInHighlight);
                    }

                    finalColor = mix(finalColor, uHighlightColor.rgb, highlightIntensity);
                }
                
                // 最终透明度计算 - 结合发光效果
                float finalAlpha = baseAlpha * uGlobalAlpha * (1.0 + glowFactor * 0.5);
                finalAlpha = clamp(finalAlpha, 0.0, 1.0);
                
                gl_FragColor = vec4(finalColor, 1.0);
            }
        )";

        mEdgeProgram->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
        mEdgeProgram->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));
        mEdgeProgram->addBindAttribLocation("lineID", 1);           // lineID -> slot 1
        mEdgeProgram->addBindAttribLocation("vertexPosition", 2);   // vertexPosition -> slot 2
        mEdgeProgram->addBindAttribLocation("segmentParam", 3);     // segmentParam(t) -> slot 3
        mEdgeProgram->addBindAttribLocation("maxHeight", 4);        // maxHeight -> slot 4
    }

    // 每次调用都为当前几何体设置StateSet和uniform参数
    if (mEdgeGeometry) {
        auto stateset = mEdgeGeometry->getOrCreateStateSet();
        stateset->setAttributeAndModes(mEdgeProgram, osg::StateAttribute::ON);

        // 启用透明度混合
        osg::BlendFunc *blendFunc = new osg::BlendFunc();
        blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE); // Additive blending
        stateset->setAttributeAndModes(blendFunc, osg::StateAttribute::OFF);

        // 禁用深度写入但保留深度测试
        osg::Depth *depth = new osg::Depth();
        depth->setFunction(osg::Depth::LEQUAL);
        stateset->setAttributeAndModes(depth, osg::StateAttribute::ON);

        // 设置渲染顺序确保透明物体正确渲染
        stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        stateset->setRenderBinDetails(10, "DepthSortedBin");

        // 添加uniform变量
        stateset->addUniform(new osg::Uniform("uEnableHighlight", false));
        stateset->addUniform(new osg::Uniform("uLineDataTex", 0));
        stateset->addUniform(new osg::Uniform("uTotalLines", static_cast<float>(edges->size())));
        stateset->addUniform(
            new osg::Uniform("uHighlightColor", osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)));
        stateset->addUniform(new osg::Uniform("uGlobalAlpha", 0.6f));   // 默认透明度
        stateset->addUniform(new osg::Uniform("uGlowIntensity", 1.5f)); // 默认发光强度
        stateset->addUniform(new osg::Uniform("uLineThickness", 2.0f)); // 默认线条粗细
    }
}
int VIS4Earth::GraphRenderer::PerGraphParam::calculateSegmentCount(const Edge &edge) {
    const float BASE_LENGTH = 1000.0f; // 基准长度(km)
    const int BASE_SEGMENTS = 5;       // 基准长度对应的细分段数
    const int MIN_SEGMENTS = 10;       // 最小细分段数
    const int MAX_SEGMENTS = 20;       // 最大细分段数

    // 计算边的总长度
    float totalLength = 0.0f;
    std::vector<float> segmentLengths;
    for (size_t i = 1; i < edge.subDivs.size(); ++i) {
        float lat1 = osg::DegreesToRadians(edge.subDivs[i - 1].x());
        float lon1 = osg::DegreesToRadians(edge.subDivs[i - 1].y());
        float lat2 = osg::DegreesToRadians(edge.subDivs[i].x());
        float lon2 = osg::DegreesToRadians(edge.subDivs[i].y());

        float dlat = lat2 - lat1;
        float dlon = lon2 - lon1;
        float a = std::sin(dlat / 2) * std::sin(dlat / 2) +
                  std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2) * std::sin(dlon / 2);
        float c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
        float length = 6371.0f * c; // 6371km是地球平均半径
        totalLength += length;
        segmentLengths.push_back(length);
    }

    // 根据边的总长度计算细分段数
    int totalSegments = static_cast<int>(BASE_SEGMENTS * (totalLength / BASE_LENGTH));
    totalSegments = std::max(MIN_SEGMENTS, std::min(MAX_SEGMENTS, totalSegments));

    return totalSegments;
}
osg::Vec3 VIS4Earth::GraphRenderer::PerGraphParam::calculateInterpolatedPosition(int segmentID,
                                                                                 const Edge &edge) {
    // 若没有细分点，直接返回原点
    if (edge.subDivs.empty()) {
        return osg::Vec3(0.0f, 0.0f, 0.0f);
    }

    // 获取边的起点和终点（经纬度）
    const osg::Vec3 &startPoint = edge.subDivs.front();
    const osg::Vec3 &endPoint = edge.subDivs.back();

    // 计算总的细分段数，用于归一化 t
    int totalSegments = calculateSegmentCount(edge);
    if (totalSegments <= 0) {
        return startPoint;
    }

    // 当前细分段在 [0, 1] 范围内的插值比例
    float t = static_cast<float>(segmentID) / static_cast<float>(totalSegments);
    if (t < 0.0f)
        t = 0.0f;
    if (t > 1.0f)
        t = 1.0f;

    // 线性插值计算“基线”位置（不做弧线抬高，留给 GPU 在 Shader 中处理）
    osg::Vec3 interpolatedPos;
    interpolatedPos.x() = startPoint.x() * (1.0f - t) + endPoint.x() * t;
    interpolatedPos.y() = startPoint.y() * (1.0f - t) + endPoint.y() * t;
    interpolatedPos.z() = 0.0f; // 高度由 Shader 根据 t 和 maxHeight 计算

    return interpolatedPos;
}

void VIS4Earth::GraphRenderer::PerGraphParam::updateEdgeVBO_GPUInterpolation(
    const std::function<osg::Vec3(const osg::Vec3 &)> &vec3ToSphere,
    osg::ref_ptr<osg::FloatArray> lineIDArray) {
    if (!edges || edges->empty() || !mEdgeGeometry)
        return;

    // 确保数组已分配
    if (!mVertexArray)
        mVertexArray = new osg::Vec3Array;
    if (!mColorFromArray)
        mColorFromArray = new osg::Vec4Array;
    if (!mColorToArray)
        mColorToArray = new osg::Vec4Array;
    if (!mWeightArray)
        mWeightArray = new osg::FloatArray;
    if (!mSegmentIDArray)
        mSegmentIDArray = new osg::FloatArray;
    if (!lineIDArray)
        lineIDArray = new osg::FloatArray;

    // 每次重建前清空数据，避免多次调用时顶点残留
    mVertexArray->clear();
    mColorFromArray->clear();
    mColorToArray->clear();
    mWeightArray->clear();
    mSegmentIDArray->clear();
    lineIDArray->clear();

    // 每个顶点对应的最大高度（每条曲线一个参数，这里先统一固定为 100000）
    osg::ref_ptr<osg::FloatArray> maxHeightArray = new osg::FloatArray;

    int lineID = 0;

    // 遍历所有边，生成顶点数据
    for (std::vector<Edge>::iterator eit = edges->begin(); eit != edges->end(); ++eit) {
        Edge &edge = *eit;
        if (!edge.visible)
            continue;

        // 计算每条边的细分段数
        const int totalSegments = calculateSegmentCount(edge);
        if (totalSegments <= 0)
            continue;

        // 本条曲线的最大高度参数（若 edge.maxHeight 尚未设置，则使用默认值 100000）
        float maxHeight = edge.maxHeight > 0.0f ? edge.maxHeight : 100000.0f;

        // 使用统一的颜色（可按需改为基于节点颜色）
        const osg::Vec4 edgeColor(0.8f, 0.6f, 0.2f, 1.0f);

        // 为该边生成一系列插值点，并用 GL_LINES 连接相邻点形成折线
        osg::Vec3 prevSpherePos;
        bool hasPrev = false;
        float prevT = 0.0f;

        for (int i = 0; i <= totalSegments; ++i) {
            // 归一化参数 t，用于在 Shader 中做弧线高度插值
            float t = static_cast<float>(i) / static_cast<float>(totalSegments);

            osg::Vec3 interpolatedPos = calculateInterpolatedPosition(i, edge);
            osg::Vec3 spherePos = vec3ToSphere(interpolatedPos);

            if (hasPrev) {
                // 每段线段写入两个顶点：前一点和当前点
                mVertexArray->push_back(prevSpherePos);
                mVertexArray->push_back(spherePos);

                // 颜色（from / to 统一为 edgeColor）
                mColorFromArray->push_back(edgeColor);
                mColorFromArray->push_back(edgeColor);
                mColorToArray->push_back(edgeColor);
                mColorToArray->push_back(edgeColor);

                // 同一条边的所有段共用一个 lineID，便于纹理高亮
                lineIDArray->push_back(static_cast<float>(lineID));
                lineIDArray->push_back(static_cast<float>(lineID));

                // 权重
                mWeightArray->push_back(edge.weight);
                mWeightArray->push_back(edge.weight);

                // 记录段内的起止参数 t（0~1），用于 Shader 中计算弧线高度
                mSegmentIDArray->push_back(prevT);
                mSegmentIDArray->push_back(t);

                // 记录每个顶点对应的最大高度参数
                maxHeightArray->push_back(maxHeight);
                maxHeightArray->push_back(maxHeight);
            }

            prevSpherePos = spherePos;
            hasPrev = true;
            prevT = t;
        }

        // 完成一条边后递增 lineID
        ++lineID;
    }

    // 设置VBO数据及 attribute 绑定（与 Shader 中的 slot 对齐）
    mEdgeGeometry->setVertexArray(mVertexArray);
    mEdgeGeometry->setVertexAttribArray(1, lineIDArray, osg::Array::BIND_PER_VERTEX);     // lineID
    mEdgeGeometry->setVertexAttribArray(2, mVertexArray, osg::Array::BIND_PER_VERTEX);    // vertexPosition（基线）
    mEdgeGeometry->setVertexAttribArray(3, mSegmentIDArray, osg::Array::BIND_PER_VERTEX); // segmentParam (t)
    mEdgeGeometry->setVertexAttribArray(4, maxHeightArray, osg::Array::BIND_PER_VERTEX);  // maxHeight
    mEdgeGeometry->setTexCoordArray(0, mColorFromArray);
    mEdgeGeometry->setTexCoordArray(1, mColorToArray);
    mEdgeGeometry->setTexCoordArray(2, mWeightArray);
    mEdgeGeometry->setUseVertexBufferObjects(true);
    mEdgeGeometry->setUseDisplayList(false);

    // 重新设置 PrimitiveSet，避免多次调用时叠加
    mEdgeGeometry->removePrimitiveSet(0, mEdgeGeometry->getNumPrimitiveSets());
    mEdgeGeometry->addPrimitiveSet(
        new osg::DrawArrays(osg::PrimitiveSet::LINES, 0,
                            static_cast<GLsizei>(mVertexArray->size())));
    lineGeode = mEdgeGeode;
    lineGeometry = mEdgeGeometry;
}


// 初始化边的着色器程序
void VIS4Earth::GraphRenderer::PerGraphParam::initEdgeShaders() {
    if (!mEdgeProgram) {
        mEdgeProgram = new osg::Program;

        // 顶点着色器
        const char *vertSource = R"(
            #version 120

            varying vec4 vColorFrom;
            varying vec4 vColorTo;
            varying float vWeight;
            varying float vLineID;
            varying float vDistanceToCamera;

            attribute vec3 vertexPosition;
            attribute float lineID; // 显式绑定 slot 1（在 C++ 中 addBindAttribLocation）

            void main() {
                vec4 worldPos = gl_ModelViewMatrix * gl_Vertex;
                gl_Position = gl_ProjectionMatrix * worldPos;

                vColorFrom = gl_MultiTexCoord0;
                vColorTo   = gl_MultiTexCoord1;
                vWeight    = gl_MultiTexCoord2.x;
                vLineID = lineID; // 将属性传给片元着色器
                
                // 计算到相机的距离用于透明度调节
                vDistanceToCamera = length(worldPos.xyz);
            }
        )";

        // 片段着色器 - 添加透明度和发光效果
        const char *fragSource = R"(
            #version 120
            varying vec4 vColorFrom;
            varying vec4 vColorTo;
            varying float vWeight;
            varying float vLineID;
            varying float vDistanceToCamera;

            uniform bool uEnableHighlight;
            uniform sampler2D uLineDataTex;
            uniform float uTotalLines;
            uniform vec4 uHighlightColor;
            uniform float uGlobalAlpha;      // 全局透明度控制
            uniform float uGlowIntensity;    // 发光强度控制
            uniform float uLineThickness;    // 线条粗细影响发光范围

            void main() {
                // 基础渐变颜色计算
                float t = fract(gl_FragCoord.x * 0.001); // 简化的插值参数
                vec4 baseColor = mix(vColorFrom, vColorTo, t);
                
                // 基于权重的亮度增强
                float weightFactor = clamp(vWeight * 0.1, 0.3, 2.0); // 权重影响亮度
                baseColor.rgb *= weightFactor;
                
                // 距离衰减透明度
                float distanceFactor = 1.0 / (1.0 + vDistanceToCamera * 0.0000001);
                float baseAlpha = clamp(distanceFactor, 0.1, 1.0);
                
                // 发光效果 - 基于线条中心的距离衰减
                float centerDistance = abs(gl_FragCoord.y - floor(gl_FragCoord.y + 0.5)); // 到线条中心的距离
                float glowRadius = uLineThickness * 2.0; // 发光半径
                float glowFactor = 1.0 - smoothstep(0.0, glowRadius, centerDistance);
                glowFactor = pow(glowFactor, 2.0); // 增强发光衰减
                
                // 组合发光效果
                vec3 glowColor = baseColor.rgb * uGlowIntensity * glowFactor;
                vec3 finalColor = baseColor.rgb + glowColor;
                
                // 高亮动画效果（保留原有功能）
                if (uEnableHighlight) {
                    float lineID = vLineID;
                float texX = (lineID + 0.5) / uTotalLines;
                float highlightPos = texture2D(uLineDataTex, vec2(texX, 0.0)).r;

                float highlightWidth = 0.05;
                float highlightIntensity = 0.0;

                if (t >= highlightPos && t <= highlightPos + highlightWidth) {
                    float posInHighlight = (t - highlightPos) / highlightWidth;
                    highlightIntensity = smoothstep(0.0, 1.0, posInHighlight);
                }

                    finalColor = mix(finalColor, uHighlightColor.rgb, highlightIntensity);
                }
                
                // 最终透明度计算 - 结合发光效果
                float finalAlpha = baseAlpha * uGlobalAlpha * (1.0 + glowFactor * 0.5);
                finalAlpha = clamp(finalAlpha, 0.0, 1.0);
                
//                gl_FragColor = vec4(finalColor, finalAlpha);
                gl_FragColor = vec4(finalColor, 1.0);
            }
        )";

        mEdgeProgram->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
        mEdgeProgram->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));
        mEdgeProgram->addBindAttribLocation("lineID", 1);         // lineID -> slot 1
        mEdgeProgram->addBindAttribLocation("vertexPosition", 2); // lineID -> slot 1
    }

    // 每次调用都为当前几何体设置StateSet和uniform参数
    if (mEdgeGeometry) {
        auto stateset = mEdgeGeometry->getOrCreateStateSet();
        stateset->setAttributeAndModes(mEdgeProgram, osg::StateAttribute::ON);

        // 启用透明度混合
        osg::BlendFunc *blendFunc = new osg::BlendFunc();
        blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE); // Additive blending
        stateset->setAttributeAndModes(blendFunc, osg::StateAttribute::OFF);

        // 禁用深度写入但保留深度测试
        osg::Depth *depth = new osg::Depth();
//        depth->setWriteMask(false);
        //  修改深度测试
        depth->setFunction(osg::Depth::LEQUAL);
        stateset->setAttributeAndModes(depth, osg::StateAttribute::ON);

        // 多边形偏移
//        osg::Polyg

        // 设置渲染顺序确保透明物体正确渲染
        stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        stateset->setRenderBinDetails(10, "DepthSortedBin");

        // 添加uniform变量（每次都重新添加，确保新的StateSet有这些参数）
        stateset->addUniform(new osg::Uniform("uEnableHighlight", false));
        stateset->addUniform(new osg::Uniform("uLineDataTex", 0));
        stateset->addUniform(new osg::Uniform("uTotalLines", static_cast<float>(edges->size())));
        stateset->addUniform(
            new osg::Uniform("uHighlightColor", osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)));

        // 发光和透明度控制参数（使用默认值，后续会通过set方法更新）
        stateset->addUniform(new osg::Uniform("uGlobalAlpha", 0.6f));   // 默认透明度
        stateset->addUniform(new osg::Uniform("uGlowIntensity", 1.5f)); // 默认发光强度
        stateset->addUniform(new osg::Uniform("uLineThickness", 2.0f)); // 默认线条粗细
    }
}

// 更新边的VBO数据
void VIS4Earth::GraphRenderer::PerGraphParam::updateEdgeVBO() {
    if (!edges || edges->empty())
        return;
    mLineIDArray = new osg::FloatArray;
    // 创建顶点数组
    mVertexArray = new osg::Vec3Array;
    mColorFromArray = new osg::Vec4Array;
    mColorToArray = new osg::Vec4Array;
    mWeightArray = new osg::FloatArray;
    mSegmentIDArray = new osg::FloatArray;
    int lineID = 0;

    std::cout << "updateEdgeVBO: Using LOD level " << currentLODLevel << std::endl;

    //// 对于 LOD0/1/2，统一走 GPU 曲线渲染路径
    //if (currentLODLevel != 3) {
    //    updateEdgeVBO_GpuInterpolated();
    //    return;
    //}

    if (currentLODLevel == 3) {
        //   加载城市建筑物的 OBB 数据
        VIS4Earth::CityLoader cityLoader;
        //AppEnv& env = AppEnv::instance();
        //String data_dir = env.getPath(AppEnv::DATA_DIR);
        //std::string str_path = data_dir.c_str();
        //str_path = str_path + "/graph/buildings_obb.csv";
        if (!cityLoader.loadBuildingsFromCSV(DATA_PATH_PREFIX "buildings_obb.csv")) {
            std::cerr << "Failed to load building data!" << std::endl;
        }
        /*if (!cityLoader.loadBuildingsFromCSV(str_path)) {
            std::cerr << "Failed to load building data!" << std::endl;
        }*/


        // 设置经纬度范围（lat_min, lon_min）到（lat_max, lon_max）
        std::vector<std::pair<float, float>> latLonBounds = {
            {20.0f, -85.0f}, // 经纬度范围的左下角
            {41.0f, -74.0f}  // 经纬度范围的右上角
        };

        // 设置比例因子，将建筑物的尺寸映射到地球表面
        float scale = 1000.f; // 可调整的比例因子，根据需要调整
        cityLoader.drawBuildings(grp, latLonBounds, scale, currentLODLevel);
        heightMap = cityLoader.getHeightMap();

        updateEdgeVBO_GPUInterpolation(GraphUtils::vec3ToSphere, mLineIDArray);
        return;
    }

    // LOD 0, 1, 2: 直接使用已生成的聚合边数据
    // 检查当前边数据是否为空
    if (edges->empty()) {
        std::cout << "No edges data available for LOD level " << currentLODLevel << std::endl;
        return;
    }

    std::cout << "Processing " << edges->size() << " aggregated edges for LOD " << currentLODLevel
              << std::endl;

    // 设置聚合边的基准参数
    const float BASE_LENGTH = 1000.0f; // 基准长度(km)
    const int BASE_SEGMENTS = 5;       // 基准长度对应的细分段数
    const int MIN_SEGMENTS = 5;        // 最小细分段数（聚合边可以更少）
    const int MAX_SEGMENTS = 15;       // 最大细分段数（聚合边不需要太多）

    // 直接遍历当前LOD级别的边数据（这些已经是聚合边）
    for (auto &edge : *edges) {
        if (!edge.visible)
            continue; // 只处理可见边

        // 获取起点和终点节点
        auto fromNodeIt = nodes->find(edge.from);
        auto toNodeIt = nodes->find(edge.to);
        if (fromNodeIt == nodes->end() || toNodeIt == nodes->end())
            continue;

        const osg::Vec3 &startPoint = fromNodeIt->second.pos;
        const osg::Vec3 &endPoint = toNodeIt->second.pos;

        if (fromNodeIt->second.level == 100 || toNodeIt->second.level == 100) {
            // 判断线条是否穿过地球
            if (fromNodeIt->second.level == 100) {
                if (GraphUtils::isLinePassingThroughEarth(
                        GraphUtils::vec3ToSphere(endPoint),
                                              GraphUtils::vec3ToSphere(startPoint))) {
                    lineID++;
                    edge.visible = false;
                    continue; // 如果穿过地球则跳过该线条
                }
            } else {
                if (VIS4Earth::GraphUtils::isLinePassingThroughEarth(
                        GraphUtils::vec3ToSphere(startPoint),
                                              GraphUtils::vec3ToSphere(endPoint))) {
                    lineID++;
                    continue; // 如果穿过地球则跳过该线条
                }
            }

            // 获取边的颜色（可以使用自定义的颜色或者节点的颜色）
            osg::Vec4 edgeColor(1.0f, 0.0f, 0.0f, 0.0f); // 默认白色，可以修改为其他颜色

            // 将边的起点和终点加入顶点数组
            mVertexArray->push_back(GraphUtils::vec3ToSphere(startPoint));
            mColorFromArray->push_back(edgeColor);
            mColorToArray->push_back(edgeColor);

            mVertexArray->push_back(GraphUtils::vec3ToSphere(endPoint));
            mColorFromArray->push_back(edgeColor);
            mColorToArray->push_back(edgeColor);

            // 记录该线段的标识符（如果需要）
            mLineIDArray->push_back(static_cast<float>(lineID));
            mLineIDArray->push_back(static_cast<float>(lineID));

            // 线段的权重（如果需要，可以根据边的属性设置权重）
            mWeightArray->push_back(edge.weight);
            mWeightArray->push_back(edge.weight);

            lineID++; // 增加线ID
        } else {
            // 获取边的颜色（使用节点颜色或默认颜色）
//            0.8 0.6 0.2
            osg::Vec4 edgeColor(0.8f, 0.6f, 0.2f, 1.0f); // 默认金色
            if (currentLODLevel < 3) {
                // 聚合边使用统一的金色
                edgeColor = osg::Vec4(0.8f, 0.6f, 0.2f, 1.0f);
            }

            // 检查是否有细分点，如果没有则使用起点和终点
            std::vector<osg::Vec3> pathPoints;
            if (!edge.subDivs.empty()) {
                pathPoints = edge.subDivs;
            } else {
                pathPoints.push_back(fromNodeIt->second.pos);
                pathPoints.push_back(toNodeIt->second.pos);
            }

            // 计算整条边的总长度
            float totalLength = 0.0f;
            std::vector<float> segmentLengths;
            for (size_t i = 1; i < pathPoints.size(); ++i) {
                float lat1 = osg::DegreesToRadians(pathPoints[i - 1].x());
                float lon1 = osg::DegreesToRadians(pathPoints[i - 1].y());
                float lat2 = osg::DegreesToRadians(pathPoints[i].x());
                float lon2 = osg::DegreesToRadians(pathPoints[i].y());

                float dlat = lat2 - lat1;
                float dlon = lon2 - lon1;
                float a = std::sin(dlat / 2) * std::sin(dlat / 2) +
                          std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2) * std::sin(dlon / 2);
                float c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
                float length = 6371.0f * c; // 6371km是地球平均半径
                totalLength += length;
                segmentLengths.push_back(length);
            }

            // 计算总的细分段数
            int totalSegments = static_cast<int>(BASE_SEGMENTS * (totalLength / BASE_LENGTH));
            totalSegments = std::max(MIN_SEGMENTS, std::min(MAX_SEGMENTS, totalSegments));

            // 根据每段长度占总长度的比例分配细分段数
            std::vector<int> segmentCounts;
            float accumulatedLength = 0.0f;
            osg::Vec3 prevPos;

            for (size_t i = 0; i < segmentLengths.size(); ++i) {
                accumulatedLength += segmentLengths[i];
                float t = accumulatedLength / totalLength;
                int currentSegments;

                if (i == segmentLengths.size() - 1) {
                    // 最后一段使用剩余的所有细分段数
                    currentSegments = totalSegments - std::accumulate(segmentCounts.begin(),
                                                                      segmentCounts.end(), 0);
                } else {
                    currentSegments =
                        static_cast<int>(totalSegments * (segmentLengths[i] / totalLength));
                }
                currentSegments = std::max(1, currentSegments); // 确保至少有一个细分段
                segmentCounts.push_back(currentSegments);

                // 在当前段内生成插值点
                const osg::Vec3 &startPoint = pathPoints[i];
                const osg::Vec3 &endPoint = pathPoints[i + 1];

                for (int j = 0; j <= currentSegments; ++j) {
                    float localT = static_cast<float>(j) / currentSegments;
                    float globalT =
                        (accumulatedLength - segmentLengths[i] + segmentLengths[i] * localT) /
                        totalLength;

                    // 线性插值位置
                    osg::Vec3 interpolatedPos;
                    interpolatedPos.x() = startPoint.x() * (1.0f - localT) + endPoint.x() * localT;
                    interpolatedPos.y() = startPoint.y() * (1.0f - localT) + endPoint.y() * localT;

                    // 计算弧线高度（使用全局t值确保整条边的弧线连续）
                    float baseHeight =
                        getBuildingHeightAtLatLon(interpolatedPos.x(), interpolatedPos.y());
                    float arcHeight = std::sin(osg::PI * globalT) * 100000.f; // 使用边的最大高度
                    interpolatedPos.z() = std::max(baseHeight, arcHeight);

                    // 转换为球面坐标
                    osg::Vec3 spherePos = GraphUtils::vec3ToSphere(interpolatedPos);

                    if (i > 0 || j > 0) {
                        // 添加线段的两个顶点
                        mVertexArray->push_back(prevPos);
                        mColorFromArray->push_back(edgeColor);
                        mColorToArray->push_back(edgeColor);
                        mWeightArray->push_back(edge.weight);

                        mVertexArray->push_back(spherePos);
                        mColorFromArray->push_back(edgeColor);
                        mColorToArray->push_back(edgeColor);
                        mWeightArray->push_back(edge.weight);

                        mLineIDArray->push_back(static_cast<float>(lineID));
                        mLineIDArray->push_back(static_cast<float>(lineID));
                    }

                    prevPos = spherePos;
                }
            }
            lineID++;
        }
    }

    // 设置VBO数据
    mEdgeGeometry->setVertexArray(mVertexArray);
    mEdgeGeometry->setVertexAttribArray(2, mVertexArray, osg::Array::BIND_PER_VERTEX);
    mEdgeGeometry->setVertexAttribArray(1, mLineIDArray, osg::Array::BIND_PER_VERTEX);
    mEdgeGeometry->setTexCoordArray(0, mColorFromArray);
    mEdgeGeometry->setTexCoordArray(1, mColorToArray);
    mEdgeGeometry->setTexCoordArray(2, mWeightArray);
    mEdgeGeometry->setUseVertexBufferObjects(true);
    mEdgeGeometry->setUseDisplayList(false);

    // 设置绘制模式
    mEdgeGeometry->addPrimitiveSet(
        new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, mVertexArray->size()));
    lineGeode = mEdgeGeode;
    lineGeometry = mEdgeGeometry;

    std::cout << "Generated " << mVertexArray->size() << " vertices from " << edges->size()
              << " aggregated edges" << std::endl;
}


// 原有的updateEdgeVBO逻辑，用于LOD 3
void VIS4Earth::GraphRenderer::PerGraphParam::updateEdgeVBO_Original(
    const std::function<osg::Vec3(const osg::Vec3 &)> &vec3ToSphere,
    osg::ref_ptr<osg::FloatArray> lineIDArray) {
    if (!edges || edges->empty())
        return;
    // 设置基准参数
    const float BASE_LENGTH = 1000.0f; // 基准长度(km)
    const int BASE_SEGMENTS = 5;       // 基准长度对应的细分段数
    const int MIN_SEGMENTS = 10;       // 最小细分段数
    const int MAX_SEGMENTS = 20;       // 最大细分段数
    int lineID = 0;
    // 遍历所有边，生成顶点数据
    for (auto &edge : *edges) {
        if (!edge.visible)
            continue; // 只处理可见边
        if (edge.subDivs.empty())
            continue; // 跳过没有细分点的边
        auto fromNodeIt = nodes->find(edge.from);
        auto toNodeIt = nodes->find(edge.to);

        if (fromNodeIt->second.level == 100 || toNodeIt->second.level == 100) {
            // 判断线条是否穿过地球
            const osg::Vec3 &startPoint = fromNodeIt->second.pos;
            const osg::Vec3 &endPoint = toNodeIt->second.pos;
            if (fromNodeIt->second.level == 100) {
                if (VIS4Earth::GraphUtils::isLinePassingThroughEarth(vec3ToSphere(endPoint),
                                                                     vec3ToSphere(startPoint))) {
                    lineID++;
                    edge.visible = false;
                    continue; // 如果穿过地球则跳过该线条
                }
            } else {
                if (VIS4Earth::GraphUtils::isLinePassingThroughEarth(vec3ToSphere(startPoint),
                                                                     vec3ToSphere(endPoint))) {
                    lineID++;
                    continue; // 如果穿过地球则跳过该线条
                }
            }

            // 获取边的颜色（可以使用自定义的颜色或者节点的颜色）
            osg::Vec4 edgeColor(1.0f, 0.0f, 0.0f, 1.0f); // 默认白色，可以修改为其他颜色

            // 将边的起点和终点加入顶点数组
            mVertexArray->push_back(vec3ToSphere(startPoint));
            mColorFromArray->push_back(edgeColor);
            mColorToArray->push_back(edgeColor);

            mVertexArray->push_back(vec3ToSphere(endPoint));
            mColorFromArray->push_back(edgeColor);
            mColorToArray->push_back(edgeColor);

            // 记录该线段的标识符（如果需要）
            lineIDArray->push_back(static_cast<float>(lineID));
            lineIDArray->push_back(static_cast<float>(lineID));

            // 线段的权重（如果需要，可以根据边的属性设置权重）
            mWeightArray->push_back(edge.weight);
            mWeightArray->push_back(edge.weight);

            lineID++; // 增加线ID
        } else {
            // osg::Vec4 startColor = osg::Vec4(fromNodeIt->second.color, 0.50f);
            // osg::Vec4 endColor = osg::Vec4(toNodeIt->second.color, 0.5f);
            // osg::Vec4 edgeColor = (startColor + endColor) * 0.5f;
            // osg::Vec4 fromColor = edgeColor;
            // osg::Vec4 toColor = edgeColor;
            //

            osg::Vec4 fromColor(0.8f, 0.6f, 0.2f, 1.0f); // 统一的金色
            osg::Vec4 toColor(0.8f, 0.6f, 0.2f, 1.0f);   // 统一的金色

            // 计算整条边的总长度
            float totalLength = 0.0f;
            std::vector<float> segmentLengths;
            for (size_t i = 1; i < edge.subDivs.size(); ++i) {
                float lat1 = osg::DegreesToRadians(edge.subDivs[i - 1].x());
                float lon1 = osg::DegreesToRadians(edge.subDivs[i - 1].y());
                float lat2 = osg::DegreesToRadians(edge.subDivs[i].x());
                float lon2 = osg::DegreesToRadians(edge.subDivs[i].y());

                float dlat = lat2 - lat1;
                float dlon = lon2 - lon1;
                float a = std::sin(dlat / 2) * std::sin(dlat / 2) +
                          std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2) * std::sin(dlon / 2);
                float c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
                float length = 6371.0f * c; // 6371km是地球平均半径
                totalLength += length;
                segmentLengths.push_back(length);
            }

            // 计算总的细分段数
            int totalSegments = static_cast<int>(BASE_SEGMENTS * (totalLength / BASE_LENGTH));
            totalSegments = std::max(MIN_SEGMENTS, std::min(MAX_SEGMENTS, totalSegments));

            // 首先得到采样点的高度,进而计算这条边上的最大高度
            // 计算每个路径上的maxHeight 用Asin(pi*x)绘制
            float maxRequiredAmplitude = 0.0f;
            std::vector<float> heightArray;
            float maxHeightInArray = 0.0;
            int maxLocation = 0;

            // 计算每个采样点的高度
            for (int i = 1; i < totalSegments + 1; i++) {
                float t = static_cast<float>(i) / (totalSegments);
                osg::Vec3 interpolatedPos;
                interpolatedPos.x() =
                    edge.subDivs.front().x() * (1.0f - t) + edge.subDivs.back().x() * t;
                interpolatedPos.y() =
                    edge.subDivs.front().y() * (1.0f - t) + edge.subDivs.back().y() * t;
                interpolatedPos.z() =
                    getBuildingHeightAtLatLon(interpolatedPos.x(), interpolatedPos.y());
                heightArray.push_back(interpolatedPos.z());
                maxHeightInArray = std::max(maxHeightInArray, interpolatedPos.z());
                if (interpolatedPos.z() >= maxHeightInArray) {
                    maxLocation = i;
                }
            }

            // 计算所需的最大振幅
            for (int i = 1; i < (totalSegments / 2) + 2; i++) {
                float t = static_cast<float>(i) / ((totalSegments / 2) + 2);
                float sinValue = std::sin(osg::PI * t); // 计算 sin(π * x)

                // 计算出对应位置所需的振幅 A，确保 A * sin(π * x) >= arr[i]
                if (heightArray[i] < 1.f)
                    continue;
                float requiredAmplitude = heightArray[i] / sinValue;
                maxRequiredAmplitude = std::max(maxRequiredAmplitude, requiredAmplitude);
            }

            // 方法2：全局控制的最大高度,绘制sin曲线
            float maxHeight = std::max(maxRequiredAmplitude, 100000.f);
            edge.maxHeight = maxHeight;

            // 根据每段长度占总长度的比例分配细分段数
            std::vector<int> segmentCounts;
            float accumulatedLength = 0.0f;
            osg::Vec3 prevPos;

            for (size_t i = 0; i < segmentLengths.size(); ++i) {
                accumulatedLength += segmentLengths[i];
                float t = accumulatedLength / totalLength;
                int currentSegments;

                if (i == segmentLengths.size() - 1) {
                    // 最后一段使用剩余的所有细分段数
                    currentSegments = totalSegments - std::accumulate(segmentCounts.begin(),
                                                                      segmentCounts.end(), 0);
                } else {
                    currentSegments =
                        static_cast<int>(totalSegments * (segmentLengths[i] / totalLength));
                }
                currentSegments = std::max(1, currentSegments); // 确保至少有一个细分段
                segmentCounts.push_back(currentSegments);

                // 在当前段内生成插值点
                const osg::Vec3 &startPoint = edge.subDivs[i];
                const osg::Vec3 &endPoint = edge.subDivs[i + 1];

                for (int j = 0; j <= currentSegments; ++j) {
                    float localT = static_cast<float>(j) / currentSegments;
                    float globalT =
                        (accumulatedLength - segmentLengths[i] + segmentLengths[i] * localT) /
                        totalLength;

                    // 线性插值位置
                    osg::Vec3 interpolatedPos;
                    interpolatedPos.x() = startPoint.x() * (1.0f - localT) + endPoint.x() * localT;
                    interpolatedPos.y() = startPoint.y() * (1.0f - localT) + endPoint.y() * localT;

                    // 使用全局t值计算高度，确保整条边的弧线连续
                    float baseHeight =
                        getBuildingHeightAtLatLon(interpolatedPos.x(), interpolatedPos.y());
                    float arcHeight = std::sin(osg::PI * globalT) * maxHeight;
                    interpolatedPos.z() = std::max(baseHeight, arcHeight);

                    // 转换为球面坐标
                    osg::Vec3 spherePos = vec3ToSphere(interpolatedPos);

                    if (i > 0 || j > 0) {
                        // 添加前一个点
                        mVertexArray->push_back(prevPos);
                        mColorFromArray->push_back(fromColor);
                        mColorToArray->push_back(toColor);
                        mWeightArray->push_back(edge.weight);

                        // 添加当前点
                        mVertexArray->push_back(spherePos);
                        mColorFromArray->push_back(fromColor);
                        mColorToArray->push_back(toColor);
                        mWeightArray->push_back(edge.weight);

                        lineIDArray->push_back(static_cast<float>(lineID));
                        lineIDArray->push_back(static_cast<float>(lineID));
                    }

                    prevPos = spherePos;
                }
            }
            lineID++;
        }
    }

    // 设置VBO数据
    mEdgeGeometry->setVertexArray(mVertexArray);
    mEdgeGeometry->setVertexAttribArray(1, lineIDArray, osg::Array::BIND_PER_VERTEX);
    mEdgeGeometry->setVertexAttribArray(2, mVertexArray, osg::Array::BIND_PER_VERTEX);
    mEdgeGeometry->setTexCoordArray(0, mColorFromArray);
    mEdgeGeometry->setTexCoordArray(1, mColorToArray);
    mEdgeGeometry->setTexCoordArray(2, mWeightArray);

    // 设置绘制模式
    mEdgeGeometry->addPrimitiveSet(
        new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, mVertexArray->size()));
    lineGeode = mEdgeGeode;
    lineGeometry = mEdgeGeometry;
}
void VIS4Earth::GraphRenderer::PerGraphParam::setRestriction(VIS4Earth::Area res) {
    restriction = res;
}

// 从当前边集合构建 GPU 曲线渲染所需的表结构
void VIS4Earth::GraphRenderer::PerGraphParam::buildTablesFromEdges(const std::vector<Edge> &edgesRef,
                                                                   bool useBundling) {
    if (!nodes || edgesRef.empty()) {
        mControlPoints.clear();
        mSegments.clear();
        mEdgeMetas.clear();
        mTablesDirty = false;
        return;
    }

    VIS4Earth::GpuEdge::buildTablesFromEdges<Node, Edge>(*nodes, edgesRef, mControlPoints,
                                                         mSegments, mEdgeMetas, useBundling);
    mTablesDirty = false;
}

// 将表上传为纹理（首版作为占位，后续根据 Shader 接口补充采样细节）
void VIS4Earth::GraphRenderer::PerGraphParam::uploadTablesToTextures() {
    if (mControlPoints.empty() || mSegments.empty()) {
        return;
    }

    // 简单的一维展开到 2D 纹理：单行存储，后续可根据规模调整为近似正方形
    const int controlCount = static_cast<int>(mControlPoints.size());
    const int controlWidth = controlCount;
    const int controlHeight = 1;

    mControlPointImage = new osg::Image;
    mControlPointImage->allocateImage(controlWidth, controlHeight, 1, GL_RGBA, GL_FLOAT);

    float *cpData = reinterpret_cast<float *>(mControlPointImage->data());
    int i = 0;
    for (std::vector<VIS4Earth::GpuEdge::ControlPoint>::const_iterator it = mControlPoints.begin();
         it != mControlPoints.end(); ++it, ++i) {
        const osg::Vec3 &p = it->pos;
        const int base = i * 4;
        cpData[base + 0] = p.x();
        cpData[base + 1] = p.y();
        cpData[base + 2] = p.z();
        cpData[base + 3] = 0.0f;
    }

    mControlPointTex = new osg::Texture2D;
    mControlPointTex->setImage(mControlPointImage.get());
    mControlPointTex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
    mControlPointTex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);
    mControlPointTex->setResizeNonPowerOfTwoHint(false);

    // 段表纹理：同样使用单行布局
    const int segmentCount = static_cast<int>(mSegments.size());
    const int segmentWidth = segmentCount;
    const int segmentHeight = 1;

    mSegmentImage = new osg::Image;
    mSegmentImage->allocateImage(segmentWidth, segmentHeight, 1, GL_RGBA, GL_FLOAT);

    float *segData = reinterpret_cast<float *>(mSegmentImage->data());
    int sIndex = 0;
    for (std::vector<VIS4Earth::GpuEdge::SegmentInfo>::const_iterator sit = mSegments.begin();
         sit != mSegments.end(); ++sit, ++sIndex) {
        const int base = sIndex * 4;
        segData[base + 0] = static_cast<float>(sit->controlStart);
        segData[base + 1] = static_cast<float>(sit->count);
        segData[base + 2] = static_cast<float>(sit->type);
        segData[base + 3] = 0.0f;
    }

    mSegmentTex = new osg::Texture2D;
    mSegmentTex->setImage(mSegmentImage.get());
    mSegmentTex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
    mSegmentTex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);
    mSegmentTex->setResizeNonPowerOfTwoHint(false);

    if (mEdgeGeometry.valid()) {
        osg::ref_ptr<osg::StateSet> ss = mEdgeGeometry->getOrCreateStateSet();
        ss->setTextureAttributeAndModes(1, mControlPointTex.get(), osg::StateAttribute::ON);
        ss->addUniform(new osg::Uniform("uControlPointTex", 1));
        ss->setTextureAttributeAndModes(2, mSegmentTex.get(), osg::StateAttribute::ON);
        ss->addUniform(new osg::Uniform("uSegmentTex", 2));
    }
}

bool VIS4Earth::GraphRenderer::PerGraphParam::setLongitudeRange(float minLonDeg, float maxLonDeg) {
    if (minLonDeg < -180.f || maxLonDeg > +180.f || minLonDeg >= maxLonDeg)
        return false;
    minLongitude = deg2Rad(minLonDeg);
    maxLongitude = deg2Rad(maxLonDeg);
    return true;
}

bool VIS4Earth::GraphRenderer::PerGraphParam::setLatitudeRange(float minLatDeg, float maxLatDeg) {
    if (minLatDeg < -90.f || maxLatDeg > +90.f || minLatDeg >= maxLatDeg)
        return false;
    minLatitude = deg2Rad(minLatDeg);
    maxLatitude = deg2Rad(maxLatDeg);
    return true;
}

bool VIS4Earth::GraphRenderer::PerGraphParam::setHeightFromCenterRange(float minH, float maxH) {
    if (minH < 0.f || minH >= maxH)
        return false;
    minHeight = minH;
    maxHeight = maxH;
    return true;
}

void GraphRenderer::PerGraphParam::performClustering(const GraphLevel &previousLevel,
                                                     GraphLevel &currentLevel, int level) {
    float threshold = static_cast<float>(level) / 10.f;
    // 从上一个层次的节点中提取位置信息，用于 DBSCAN 聚类
    std::vector<osg::Vec3> positions;
    std::vector<std::string> nodeIds;
    for (const auto &nodePair : *previousLevel.nodes) {
        positions.push_back(nodePair.second.pos);
        nodeIds.push_back(nodePair.first);
    }
    std::vector<std::pair<std::string, std::string>> dbscanedges;
    std::vector<float> weights;
    // 提取边的from和to字段
    for (const auto &edge : *previousLevel.edges) {
        dbscanedges.push_back({edge.from, edge.to});
        weights.push_back(edge.weight);
    }

    // 将节点根据簇分类
    std::map<int, std::vector<std::string>> clusters; // 簇ID -> 节点ID列表
    // 使用 DBSCAN 对节点进行聚类
    if (level == 1) {

        std::vector<int> clusterLabels;

        if (graphTypeIndex == 1) {
            // 第一次聚类
            clusterLabels = DBSCAN(positions, 4, /*minPts*/ 1, dbscanedges, nodeIds);

            // 找出噪声点
            std::vector<size_t> noisePoints;
            for (size_t i = 0; i < clusterLabels.size(); ++i) {
                if (clusterLabels[i] == -1) {
                    noisePoints.push_back(i);
                }
            }

            // 对噪声点进行特殊处理
            if (!noisePoints.empty()) {
                // 方案1：将噪声点分配给最近的非噪声簇
                for (size_t idx : noisePoints) {
                    double minDist = std::numeric_limits<double>::max();
                    int nearestCluster = -1;

                    // 找到最近的非噪声簇
                    for (size_t j = 0; j < positions.size(); ++j) {
                        if (clusterLabels[j] != -1) {
                            double dist = (positions[idx] - positions[j]).length();
                            if (dist < minDist) {
                                minDist = dist;
                                nearestCluster = clusterLabels[j];
                            }
                        }
                    }

                    // 将噪声点分配给最近的簇
                    if (nearestCluster != -1) {
                        clusterLabels[idx] = nearestCluster;
                    }
                }
            }
        } else {
            clusterLabels = DBSCAN(positions, 4, /*minPts*/ 1, dbscanedges, nodeIds);
        }
        // 处理噪声节点，随机分配给邻居节点
        // 找到最大的标签
        int maxClusterID = -1;
        for (const auto &point : clusterLabels) {
            if (point != -1) {
                maxClusterID = std::max(maxClusterID, point);
            }
        }
        for (size_t i = 0; i < clusterLabels.size(); ++i) {
            int clusterId = clusterLabels[i];
            if (clusterId == -1) {
                clusterId = maxClusterID++;
            }
            clusters[clusterId].push_back(nodeIds[i]);
        }
    } else {
        // 遍历当前层的所有节点
        for (const auto &nodePair : *previousLevel.nodes) {
            const std::string &nodeId = nodePair.first;
            int clusterId = nodePair.second.cluster; // 获取节点的 cluster 属性

            // 将节点 ID 添加到对应的簇中
            clusters[clusterId].push_back(nodeId);
        }
    }

    std::map<std::string, std::vector<std::string>> nodeMapping; // 原始节点到代表节点的映射
    std::map<Edge, std::vector<Edge>> edgeMapping;               // 边映射
    std::set<std::string> processedNodes;
    std::vector<std::string> remainingNodes; // 未处理的节点列表
    int clusterIdCounter = 0;                // 簇ID计数器

    // 遍历每个簇，选择代表节点并合并
    for (const auto &clusterPair : clusters) {
        const std::vector<std::string> &nodesInCluster = clusterPair.second;
        // 分配簇ID
        for (const std::string &nodeId : nodesInCluster) {
            previousLevel.nodes->at(nodeId).cluster = clusterIdCounter;
        }

        if (nodesInCluster.size() > 1) {
            // 如果簇中有多个节点，选择权重最高的节点作为代表节点
            std::string representativeNodeId = *std::max_element(
                nodesInCluster.begin(), nodesInCluster.end(),
                [&](const std::string &a, const std::string &b) {
                    return previousLevel.nodes->at(a).level < previousLevel.nodes->at(b).level;
                });

            // 将代表节点添加到当前层次
            currentLevel.nodes->emplace(representativeNodeId,
                                        previousLevel.nodes->at(representativeNodeId));
            currentLevel.nodes->at(representativeNodeId).isRepresent = true;

            //// 设置代表节点的大小，基于簇中节点的数量
            float representativeSize =
                (static_cast<float>(nodesInCluster.size()) * 0.15 +
                 previousLevel.nodes->at(representativeNodeId).size); // 根据节点数量设置大小
            currentLevel.nodes->at(representativeNodeId).size =
                std::min(representativeSize, currentLevel.nodes->at(representativeNodeId).size * 2);

            processedNodes.insert(representativeNodeId);

            // 更新簇中所有节点的映射关系
            for (const std::string &nodeId : nodesInCluster) {
                nodeMapping[representativeNodeId].push_back(nodeId);
            }
        } else {
            // 如果簇中只有一个节点，直接保留
            const std::string &singleNodeId = nodesInCluster[0];
            currentLevel.nodes->emplace(singleNodeId, previousLevel.nodes->at(singleNodeId));
            nodeMapping[singleNodeId].push_back(singleNodeId);
            processedNodes.insert(singleNodeId);
        }
        // 更新簇ID计数器
        ++clusterIdCounter;
    }

    // 将未处理的节点放入 remainingNodes 列表
    for (const auto &nodePair : *previousLevel.nodes) {
        const std::string &nodeId = nodePair.first;
        if (processedNodes.find(nodeId) == processedNodes.end()) {
            remainingNodes.push_back(nodeId);
        }
    }
    // 计算目标节点数 N_simplified
    int N_simplified = static_cast<int>(previousLevel.nodes->size() * (1 - threshold));

    // 如果当前层次的节点数大于 N_simplified，删除权重较低的节点
    if (currentLevel.nodes->size() > N_simplified) {
        // 获取所有节点并按权重升序排序
        std::vector<std::pair<std::string, Node>> sortedNodes(currentLevel.nodes->begin(),
                                                              currentLevel.nodes->end());
        std::sort(sortedNodes.begin(), sortedNodes.end(),
                  [](const std::pair<std::string, Node> &a, const std::pair<std::string, Node> &b) {
                      return a.second.level < b.second.level;
                  });

        // 删除权重较低的节点，直到节点数等于 N_simplified
        int nodesToRemove = static_cast<int>(currentLevel.nodes->size()) - N_simplified;
        for (int i = 0; i < nodesToRemove; ++i) {
            std::string nodeIdToRemove = sortedNodes[i].first;
            currentLevel.nodes->erase(nodeIdToRemove);
            nodeMapping.erase(nodeIdToRemove); // 删除节点映射

            // 删除与该节点相关的边
            currentLevel.edges->erase(
                std::remove_if(currentLevel.edges->begin(), currentLevel.edges->end(),
                               [&](const Edge &edge) {
                                   return edge.from == nodeIdToRemove || edge.to == nodeIdToRemove;
                               }),
                currentLevel.edges->end());

            // 删除边映射
            for (auto it = edgeMapping.begin(); it != edgeMapping.end();) {
                if (it->first.from == nodeIdToRemove || it->first.to == nodeIdToRemove) {
                    it = edgeMapping.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }

    // 添加剩余节点，直到当前层次的节点数等于 N_simplified
    if (currentLevel.nodes->size() < N_simplified && remainingNodes.size() > 0) {
        // 将剩余节点按权重降序排列
        std::sort(remainingNodes.begin(), remainingNodes.end(),
                  [&](const std::string &a, const std::string &b) {
                      return previousLevel.nodes->at(a).level > previousLevel.nodes->at(b).level;
                  });

        // 添加足够数量的剩余节点，直到达到 N_simplified
        int nodesToAdd = N_simplified - static_cast<int>(currentLevel.nodes->size());
        for (int i = 0; i < nodesToAdd; ++i) {
            const std::string &nodeIdToAdd = remainingNodes[i];
            currentLevel.nodes->emplace(nodeIdToAdd, previousLevel.nodes->at(nodeIdToAdd));

            // 更新节点映射，直接映射自己
            nodeMapping[nodeIdToAdd].push_back(nodeIdToAdd);
        }
    }
    // 处理边
    std::set<std::pair<std::string, std::string>> processedEdges;

    // 1. 保留当前层次中已经存在的边
    for (const Edge &edge : *previousLevel.edges) {
        if (currentLevel.nodes->count(edge.from) > 0 && currentLevel.nodes->count(edge.to) > 0) {
            Edge newEdge = edge;
            currentLevel.edges->push_back(newEdge);

            // 记录边映射
            edgeMapping[newEdge] = {edge};

            // 标记已处理的边
            processedEdges.insert({std::min(edge.from, edge.to), std::max(edge.from, edge.to)});
        }
    }
    // 保存节点映射和边映射到当前层次
    currentLevel.nodeMapping =
        std::make_shared<std::map<std::string, std::vector<std::string>>>(nodeMapping);
    currentLevel.edgeMapping = std::make_shared<std::map<Edge, std::vector<Edge>>>(edgeMapping);
}

float VIS4Earth::GraphRenderer::PerGraphParam::getBuildingHeightAtLatLon(float lat, float lon) {
    std::vector<std::pair<float, float>> latLonBounds = {
        {20.0f, -85.0f}, // 经纬度范围的左下角
        {41.0f, -74.0f}  // 经纬度范围的右上角
    };
    // 将经纬度映射到高度图的行列
    int row = static_cast<int>((lat - latLonBounds[0].first) /
                               (latLonBounds[1].first - latLonBounds[0].first) * (100 - 1));
    int col = static_cast<int>((lon - latLonBounds[0].second) /
                               (latLonBounds[1].second - latLonBounds[0].second) * (100 - 1));

    // 获取四邻点的索引
    int x1 = std::max(0, row - 1);
    int y1 = std::max(0, col - 1);
    int x2 = std::min(100 - 1, row + 1);
    int y2 = std::min(100 - 1, col + 1);
    if (x1 >= 100 || y1 >= 100 || x2 >= 100 || y2 >= 100 || x1 < 0 || y1 < 0 || x2 < 0 || y2 < 0) {
        return 0;
    }
    // 获取四个邻近点的高度
    float height1 = heightMap[x1][y1];
    float height2 = heightMap[x1][y2];
    float height3 = heightMap[x2][y1];
    float height4 = heightMap[x2][y2];

    // 返回最大高度
    return std::max({height1, height2, height3, height4});
}

// 根据当前 LOD 的节点 / 边数据，构建一个 VIS4Earth::Graph，供 EdgeBundling 使用
std::shared_ptr<VIS4Earth::Graph>
GraphRenderer::buildGraphFromLODData(const std::shared_ptr<std::map<std::string, Node>> &lodNodes,
                                     const std::shared_ptr<std::vector<Edge>> &lodEdges) {
    using namespace VIS4Earth;

    if (!lodNodes || !lodEdges) {
        return nullptr;
    }

    auto graph = std::make_shared<Graph>();

    // 1. 先把渲染用的节点转成 bundling Graph 里的 Node
    std::unordered_map<std::string, VIS4Earth::Node> gNodes;
    gNodes.reserve(lodNodes->size());

    for (const auto &kv : *lodNodes) {
        const std::string &id = kv.first;
        const Node &rn = kv.second; // 渲染层的节点

        // 用经纬度（x,y）和高度（z）构建 Graph::Node
        VIS4Earth::Node gn(rn.pos.x(), rn.pos.y(), rn.pos.z(), rn.level);
        gn.id = id;
        gn.name = rn.id; // 你可以按需赋值
        gn.color = VIS4Earth::GraphUtils::rgbToHex(rn.color.x(), rn.color.y(), rn.color.z());

        gNodes.emplace(id, gn);
    }

    // 2. 再把渲染用的 Edge 转成 Graph::Edge
    std::vector<VIS4Earth::Edge> gEdges;
    gEdges.reserve(lodEdges->size());

    for (const auto &re : *lodEdges) {
        // 假定 re.from / re.to 对应 lodNodes 里的 key
        auto itFrom = gNodes.find(re.from);
        auto itTo = gNodes.find(re.to);
        if (itFrom == gNodes.end() || itTo == gNodes.end()) {
            // LOD 数据不完整，跳过这条边
            continue;
        }

        const auto &fromNode = itFrom->second;
        const auto &toNode = itTo->second;

        glm::vec3 start = fromNode.pos;
        glm::vec3 end = toNode.pos;
        double width = 1.0; // 这里随便给个宽度，后续 Graph::set 会归一化

        VIS4Earth::Edge ge(re.from, re.to, start, end, width);
        // 构造函数里会调用 arrangeDirection() 和 addSubdivisions()

        gEdges.push_back(std::move(ge));
    }

    // 3. 用 Graph::set() 把节点+边塞进去（会顺便计算度数、归一化宽度等）
    graph->set(gNodes, gEdges);

    return graph;
}

void GraphRenderer::saveBundledGraphToFile(const std::shared_ptr<VIS4Earth::Graph> &graph,
                                           const QString &filePath) {
    if (!graph)
        return;

    try {
        QFile file(filePath);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            qDebug() << "Failed to open bundledEdges file for write:" << filePath;
            return;
        }

        QTextStream out(&file);

        auto &edges = graph->getEdges();
        for (const auto &edge : edges) {
            out << QString::fromStdString(edge.sourceLabel) << ","
                << QString::fromStdString(edge.targetLabel);

            const auto &subdivs = edge.subdivs;

            if (subdivs.empty()) {
                // 标记为0表示无细分点，后面是2个点（起点终点）
                out << ",0,2";
                out << "," << edge.start.x << "," << edge.start.y << "," << edge.end.x << ","
                    << edge.end.y;
            } else {
                // 标记为1表示有细分点，后面是点的总数
                out << ",1," << subdivs.size();

                // 先写起点
                out << "," << edge.start.x << "," << edge.start.y;

                // 再写所有细分点
                for (const auto &p : subdivs) {
                    out << "," << p.x << "," << p.y;
                }

                // 最后写终点
                out << "," << edge.end.x << "," << edge.end.y;
            }
            out << "\n";
        }

        file.close();
    } catch (const std::exception &e) {
        qDebug() << "Error saving bundled edges to" << filePath << ":" << e.what();
    }
}

// 初始化LOD数据 (在GraphRenderer层面管理)
void VIS4Earth::GraphRenderer::initializeLODData(
    std::shared_ptr<std::map<std::string, Node>> allNodes,
    std::shared_ptr<std::vector<Edge>> allEdges) {

    // 清空现有的LOD数据
    for (int i = 0; i < 4; ++i) {
        lodNodesData[i].reset();
        lodEdgesData[i].reset();
    }

    // 更新 LOD 3 数据
    lodNodesData[3] = allNodes;
    lodEdgesData[3] = allEdges;

    // 为LOD 0, 1, 2生成层次化的地理聚合数据
    for (int currentLOD = 0; currentLOD < 3; ++currentLOD) {
        generateGeographicLODData(currentLOD, allNodes, allEdges);
    }

    // 输出统计信息
    for (int i = 0; i < 4; ++i) {
        std::cout << "LOD Level " << i << ": " << lodNodesData[i]->size() << " nodes, "
                  << lodEdgesData[i]->size() << " edges" << std::endl;
    }

    // 初始化地理网格，使用LOD3数据（最详细级别）
    std::cout << "Initializing Earth Grid with LOD3 data..." << std::endl;
    earthGrid.clearGrid();
    if (lodNodesData[3]) {
        int insertCount = 0;
        double minLat = 90.0, maxLat = -90.0, minLon = 180.0, maxLon = -180.0;

        for (const auto &nodePair : *lodNodesData[3]) {
            earthGrid.insertNodeIntoGrid(nodePair.second);
            insertCount++;

            // 统计插入节点的坐标范围
            double lat = nodePair.second.pos.x();
            double lon = nodePair.second.pos.y();
            minLat = std::min(minLat, lat);
            maxLat = std::max(maxLat, lat);
            minLon = std::min(minLon, lon);
            maxLon = std::max(maxLon, lon);
        }

        std::cout << "Initialized Earth Grid with " << insertCount << " nodes" << std::endl;
        std::cout << "Grid data range: lat[" << minLat << ", " << maxLat << "], lon[" << minLon
                  << ", " << maxLon << "]" << std::endl;
    } else {
        std::cout << "ERROR: LOD3 data is null, cannot initialize Earth Grid" << std::endl;
    }

    bool needAnyBundling = false;
    for (int lod = 0; lod < 4; ++lod) {
        QString path = bundledFilePathForLOD(lod);
        if (QFile::exists(path)) {
            // 已经有对应 LOD 的绑定结果文件了，直接认为 ready
            _lodBundlingReady[lod] = true;
        } else {
            _lodBundlingReady[lod] = false;
            needAnyBundling = true; // 至少有一个 LOD 需要重新计算
        }
    }

    // 如果所有 LOD 都已经有绑定结果，就不需要开辟线程
    if (!needAnyBundling) {
        _bundlingAllRunning = false;
        return;
    }
    // 如果之前已经有一个 bundling 任务在跑，先不重复启动
    if (_bundlingAllRunning) {
        return;
    }

    _bundlingAllRunning = true;

    _bundlingFuture = std::async(std::launch::async, [this]() {
        try {
            // 依次计算 LOD 0~3 的 bundling，算完一个就写文件并标记 ready
            for (int lod = 0; lod < 4; ++lod) {
                auto nodes = lodNodesData[lod];
                auto edges = lodEdgesData[lod];
                if (!nodes || !edges) {
                    _lodBundlingReady[lod] = false;
                    continue;
                }

                // 这里根据你的工程实际情况，构建临时 Graph
                auto tmpGraph = buildGraphFromLODData(lodNodesData[lod], lodEdgesData[lod]);
                if (!tmpGraph) {
                    _lodBundlingReady[lod] = false;
                    continue;
                }

                VIS4Earth::EdgeBundling edgeBundling;
                edgeBundling.SetGraph(tmpGraph);

                // 如果你有兼容性预计算，可以在这里做（或确认已做）
                tmpGraph->buildCompatibilityListsIfNeeded();

                edgeBundling.SetParameter(mybundlingParam);
                edgeBundling.EdgeBundle();

                auto bundledGraph = edgeBundling.GetLayoutedGraph();
                saveBundledGraphToFile(bundledGraph, bundledFilePathForLOD(lod));
                _lodBundlingReady[lod] = true;
            }
        } catch (const std::exception &e) {
            qDebug() << "Error in background bundling:" << e.what();
        }
        _bundlingAllRunning = false;
    });
}

// 生成基于地理分区的LOD数据（重新设计为基于节点level的简化版本）
void VIS4Earth::GraphRenderer::generateGeographicLODData(
    int lodLevel, std::shared_ptr<std::map<std::string, Node>> allNodes,
    std::shared_ptr<std::vector<Edge>> allEdges) {

    auto lodNodes = std::make_shared<std::map<std::string, Node>>();
    auto lodEdges = std::make_shared<std::vector<Edge>>();

    std::cout << "Generating level-based LOD data for level " << lodLevel << std::endl;

    // 第一步：基于节点level筛选节点
    for (const auto &nodePair : *allNodes) {
        const Node &node = nodePair.second;
        // 根据LOD级别筛选节点：LOD n 包含 level <= n 的所有节点
        if (node.level <= lodLevel || node.level == 100) {
            // 直接使用原始节点，保持原始ID不变
            (*lodNodes)[nodePair.first] = node;
        }
    }

    std::cout << "Selected " << lodNodes->size() << " nodes with level <= " << lodLevel
              << std::endl;

    // 第二步：添加涉及筛选节点的直接边
    for (const auto &edge : *allEdges) {
        if (!edge.visible)
            continue;

        // 检查边的两端节点是否都在当前LOD的节点集合中
        if (lodNodes->find(edge.from) != lodNodes->end() &&
            lodNodes->find(edge.to) != lodNodes->end()) {
            lodEdges->push_back(edge);
        }
    }

    std::cout << "Added " << lodEdges->size() << " direct edges between selected nodes"
              << std::endl;

    // 第三步：为跨区域连接生成聚合边（仅对LOD0-LOD2）
    if (lodLevel <= 2) {
        generateAggregatedEdgesForLOD(lodLevel, lodNodes, lodEdges, allNodes, allEdges);
    }

    std::cout << "Final LOD " << lodLevel << ": " << lodNodes->size() << " nodes, "
              << lodEdges->size() << " edges" << std::endl;

    // 设置LOD数据
    lodNodesData[lodLevel] = lodNodes;
    lodEdgesData[lodLevel] = lodEdges;
}

// 更新活动LOD
void VIS4Earth::GraphRenderer::updateActiveLOD(double cameraHeight) {
    int targetMaxNodeLevel = getCurrentLevel(cameraHeight);

    // 性能优化：只有当LOD级别发生变化时才重新绘制
    if (targetMaxNodeLevel == currentActiveLODLevel && targetMaxNodeLevel != 3) {
        // LOD级别没有变化，只更新标签（因为相机位置可能变化）
        cameraUpdate("LoadedGraph", cameraHeight);
        return;
    }
    if (targetMaxNodeLevel == 1 && currentActiveLODLevel == 0) {
        return;
    }
    if (targetMaxNodeLevel == 0 && currentActiveLODLevel == 1) {
        return;
    }
    PerGraphParam *graphParam = getGraph("LoadedGraph");
    if (graphParam && targetMaxNodeLevel >= 0 && targetMaxNodeLevel < 4) {
        // 直接设置PerGraphParam的nodes和edges指针到对应的LOD数据
        if (lodNodesData[targetMaxNodeLevel] && lodEdgesData[targetMaxNodeLevel]) {
            graphParam->nodes = lodNodesData[targetMaxNodeLevel];
            graphParam->edges = lodEdgesData[targetMaxNodeLevel];

            // 设置PerGraphParam的当前LOD层级
            graphParam->currentLODLevel = targetMaxNodeLevel;

            std::cout << "Switched to LOD level " << targetMaxNodeLevel << " with "
                      << graphParam->nodes->size() << " nodes and " << graphParam->edges->size()
                      << " edges" << std::endl;

            // 先保存需要设置的发光参数
            float targetGlowIntensity, targetGlobalAlpha, targetLineThickness;

            // 根据LOD级别计算发光效果参数
            switch (targetMaxNodeLevel) {
            case 0:                         // LOD0 - 最粗糙级别，需要强烈发光突出聚合边
                targetGlowIntensity = 2.5f; // 高发光强度
                targetGlobalAlpha = 0.8f;   // 较高透明度
                targetLineThickness = 3.0f; // 粗线条
                break;
            case 1:                         // LOD1 - 中等级别
                targetGlowIntensity = 2.0f; // 中等发光强度
                targetGlobalAlpha = 0.7f;   // 中等透明度
                targetLineThickness = 2.5f; // 中等线条
                break;
            case 2:                         // LOD2 - 较详细级别
                targetGlowIntensity = 1.5f; // 较低发光强度
                targetGlobalAlpha = 0.6f;   // 中等透明度
                targetLineThickness = 2.0f; // 较细线条
                break;
            case 3:                         // LOD3 - 最详细级别，发光效果适中
                targetGlowIntensity = 1.0f; // 基础发光强度
                targetGlobalAlpha = 0.5f;   // 较低透明度
                targetLineThickness = 1.5f; // 细线条
                break;
            }

            // 根据边密度进一步调整发光效果
            int edgeCount = graphParam->edges->size();
            if (edgeCount > 10000) {
                // 边密度很高时，降低发光强度避免过度曝光
                float currentGlow = 2.5f - (targetMaxNodeLevel * 0.5f);
                targetGlowIntensity = currentGlow * 0.7f;
            } else if (edgeCount < 1000) {
                // 边密度较低时，增强发光效果提升可见性
                float currentGlow = 2.5f - (targetMaxNodeLevel * 0.5f);
                targetGlowIntensity = currentGlow * 1.3f;
            }

            // 更新当前活动的LOD级别
            currentActiveLODLevel = targetMaxNodeLevel;

            // 更新地理网格：清空并重新插入当前LOD的节点
            earthGrid.clearGrid();
            if (lodNodesData[targetMaxNodeLevel]) {
                for (const auto &nodePair : *lodNodesData[targetMaxNodeLevel]) {
                    earthGrid.insertNodeIntoGrid(nodePair.second);
                }
                std::cout << "Updated Earth Grid with " << lodNodesData[targetMaxNodeLevel]->size()
                          << " nodes for LOD" << targetMaxNodeLevel << std::endl;
            }
            // 更新标签（无论LOD是否变化都需要调用，因为相机位置可能变化）
            // 对LOD3进行视锥剔除优化，其他级别重置所有可见性
            if (targetMaxNodeLevel == 3) {
                if (graphParam && graphParam->_camera) {
                    // 从相机提取视锥边界并调用剔除
                    SimpleFrustumBounds bounds;
                    if (extractCameraBounds(graphParam->_camera, bounds)) {
                        frustumCulling("LoadedGraph", bounds.minLon, bounds.maxLon, bounds.minLat,
                                       bounds.maxLat, targetMaxNodeLevel);
                    }
                }
            } else {
                // 非LOD3级别重置所有可见性，确保数据正常显示
                if (lodNodesData[targetMaxNodeLevel] && lodEdgesData[targetMaxNodeLevel]) {
                    resetAllVisibility(lodNodesData[targetMaxNodeLevel],
                                       lodEdgesData[targetMaxNodeLevel]);
                }
            }

            // 重新绘制几何体
            std::unordered_map<std::string, VIS4Earth::Node> graphNodes;
            for (auto itr = graphParam->nodes->begin(); itr != graphParam->nodes->end(); ++itr) {
                graphNodes.insert(std::pair<std::string, VIS4Earth::Node>(
                    std::string(itr->first),
                    VIS4Earth::Node(itr->second.pos.x(), itr->second.pos.y(), itr->second.pos.z(),
                                    itr->second.level, itr->second.label,
                                    VIS4Earth::GraphUtils::rgbToHex(itr->second.color.x(),
                                                                    itr->second.color.y(),
                                             itr->second.color.z()))));
            }
            std::vector<VIS4Earth::Edge> garphEdges;

            for (auto itr = graphParam->edges->begin(); itr != graphParam->edges->end(); ++itr) {

                osg::Vec3 fromPos = graphParam->nodes->at(itr->from).pos;
                osg::Vec3 toPos = graphParam->nodes->at(itr->to).pos;
                if (graphParam->nodes->at(itr->from).level == 100 &&
                    VIS4Earth::GraphUtils::isLinePassingThroughEarth(GraphUtils::vec3ToSphere(toPos),
                                              GraphUtils::vec3ToSphere(toPos))) {
                    continue;
                }
                if (graphParam->nodes->at(itr->to).level == 100 &&
                    VIS4Earth::GraphUtils::isLinePassingThroughEarth(
                        GraphUtils::vec3ToSphere(fromPos),
                                              GraphUtils::vec3ToSphere(toPos))) {
                    continue;
                }
                glm::vec3 from = glm::vec3(fromPos.x(), fromPos.y(), fromPos.z());
                glm::vec3 to = glm::vec3(toPos.x(), toPos.y(), toPos.z());
                garphEdges.push_back(VIS4Earth::Edge(itr->from, itr->to, from, to, itr->weight));
            }
            auto graphSetLOD = std::make_shared<VIS4Earth::Graph>();
            graphSetLOD->set(graphNodes, garphEdges);
            myGraph = graphSetLOD;
            // compatibilityFuture = std::async(
            //     std::launch::async, [this]() { myGraph->buildCompatibilityListsIfNeeded(); });
            // showBundling();

            // 先调用update()来重新创建几何体和着色器
            graphParam->update();

            // 然后重新应用发光效果参数（在initEdgeShaders()之后）
            graphParam->setGlowIntensity(targetGlowIntensity);
            graphParam->setGlobalAlpha(targetGlobalAlpha);
            graphParam->setLineThickness(targetLineThickness);

            sceneLabels.clear();
            cameraUpdate("LoadedGraph", cameraHeight);
        }
    }
}

// 视锥剔除辅助函数实现
bool VIS4Earth::GraphRenderer::extractCameraBounds(osg::Camera *camera,
                                                   SimpleFrustumBounds &bounds) {
    if (!camera)
        return false;

    // 复用nodeClickHandler中的相机参数提取逻辑
    osg::Vec3d eyePosition, center, up;
    camera->getViewMatrixAsLookAt(eyePosition, center, up);

    double R_earth = 6371000.0; // 地球半径(米)
    double distance = eyePosition.length();
    bounds.cameraHeight = distance - R_earth;

    if (bounds.cameraHeight <= 0)
        return false;

    // 简化的地面投影计算
    double groundRadius = bounds.cameraHeight * 1.0; // 简化估算系数
    double latOffset = (groundRadius / R_earth) * 180.0 / osg::PI;
    double lonOffset = latOffset;

    // 简化的相机位置转经纬度
    double lat = std::asin(eyePosition.z() / distance) * 180.0 / osg::PI;
    double lon = std::atan2(eyePosition.y(), eyePosition.x()) * 180.0 / osg::PI;

    bounds.minLat = std::max(lat - latOffset, -90.0);
    bounds.maxLat = std::min(lat + latOffset, 90.0);
    bounds.minLon = std::max(lon - lonOffset, -180.0);
    bounds.maxLon = std::min(lon + lonOffset, 180.0);

    bounds.isValid = true;

    // 添加详细调试信息
    std::cout << "=== Camera Bounds Debug ===" << std::endl;
    std::cout << "Camera position: (" << eyePosition.x() << ", " << eyePosition.y() << ", "
              << eyePosition.z() << ")" << std::endl;
    std::cout << "Camera distance: " << distance << " meters" << std::endl;
    std::cout << "Camera height: " << bounds.cameraHeight << " meters" << std::endl;
    std::cout << "Ground radius: " << groundRadius << " meters" << std::endl;
    std::cout << "Camera lat/lon: (" << lat << ", " << lon << ")" << std::endl;
    std::cout << "Lat offset: " << latOffset << ", Lon offset: " << lonOffset << std::endl;
    std::cout << "Frustum bounds: lat[" << bounds.minLat << ", " << bounds.maxLat << "], lon["
              << bounds.minLon << ", " << bounds.maxLon << "]" << std::endl;
    std::cout << "=========================" << std::endl;

    return true;
}

bool VIS4Earth::GraphRenderer::frustumSignificantlyChanged(const SimpleFrustumBounds &current,
                                                           const SimpleFrustumBounds &last) {
    if (!last.isValid)
        return true;

    const double threshold = 0.1; // 经纬度变化阈值
    return (std::abs(current.minLat - last.minLat) > threshold ||
            std::abs(current.maxLat - last.maxLat) > threshold ||
            std::abs(current.minLon - last.minLon) > threshold ||
            std::abs(current.maxLon - last.maxLon) > threshold);
}


void VIS4Earth::GraphRenderer::performPreciseCulling(
    osg::Camera *camera, const std::vector<std::string> &candidateNodes,
    std::shared_ptr<std::map<std::string, Node>> allNodes) {
    // 使用OSG的视锥剔除
    osg::Polytope frustum;
    osg::Matrixd mvp = camera->getViewMatrix() * camera->getProjectionMatrix();
    frustum.setToUnitFrustum(true, true);
    frustum.transformProvidingInverse(osg::Matrixd::inverse(mvp));

    // 首先将所有节点设为不可见
    for (auto &nodePair : *allNodes) {
        nodePair.second.visible = false;
    }

    // 对候选节点进行精确视锥测试
    int visibleCount = 0;
    for (const std::string &nodeId : candidateNodes) {
        auto it = allNodes->find(nodeId);
        if (it == allNodes->end())
            continue;
        it->second.visible = true;
        visibleCount++;
        osg::Vec3 worldPos = VIS4Earth::GraphUtils::latLonToWorldPos(it->second.pos.x(), it->second.pos.y());
        if (frustum.contains(worldPos)) {
            it->second.visible = true; // 设置为可见
            visibleCount++;
        }
    }

    std::cout << "Precise culling: " << visibleCount << "/" << candidateNodes.size()
              << " nodes are visible" << std::endl;
}

void VIS4Earth::GraphRenderer::updateVisibilityFromCache(const std::string &graphName) {
    PerGraphParam *graphParam = getGraph(graphName);
    if (!graphParam)
        return;

    // 从缓存恢复节点可见性
    if (lodNodesData[3]) {
        for (auto &nodePair : *lodNodesData[3]) {
            auto cacheIt = cachedNodeVisibility.find(nodePair.first);
            if (cacheIt != cachedNodeVisibility.end()) {
                nodePair.second.visible = cacheIt->second;
            }
        }
    }

    // 从缓存恢复边可见性
    if (lodEdgesData[3]) {
        for (auto &edge : *lodEdgesData[3]) {
            auto cacheIt = cachedEdgeVisibility.find(edge.id);
            if (cacheIt != cachedEdgeVisibility.end()) {
                edge.visible = cacheIt->second;
            }
        }
    }

    // 更新当前显示的数据（基于visible属性）
    currentLevelLabels.clear();
    currentNodes.clear();

    if (lodNodesData[3]) {
        for (const auto &nodePair : *lodNodesData[3]) {
            if (nodePair.second.visible) {
                currentLevelLabels.insert(nodePair.first);
                currentNodes.push_back(nodePair.second);
            }
        }
    }

    std::cout << "Restored visibility from cache: " << currentNodes.size() << " visible nodes"
              << std::endl;
}

void VIS4Earth::GraphRenderer::cullEdgesByVisibility(
    std::shared_ptr<std::vector<Edge>> allEdges,
    std::shared_ptr<std::map<std::string, Node>> allNodes) {
    int visibleEdgeCount = 0;
    for (auto &edge : *allEdges) {
        // 检查边的两端节点是否可见
        auto fromIt = allNodes->find(edge.from);
        auto toIt = allNodes->find(edge.to);

        bool fromVisible = (fromIt != allNodes->end()) && fromIt->second.visible;
        bool toVisible = (toIt != allNodes->end()) && toIt->second.visible;

        edge.visible = fromVisible && toVisible;
        if (edge.visible) {
            visibleEdgeCount++;
        }
    }

    std::cout << "Edge culling: " << visibleEdgeCount << "/" << allEdges->size()
              << " edges are visible" << std::endl;
}

void VIS4Earth::GraphRenderer::resetAllVisibility(
    std::shared_ptr<std::map<std::string, Node>> allNodes,
    std::shared_ptr<std::vector<Edge>> allEdges) {
    // 重置所有节点可见性为true
    for (auto &nodePair : *allNodes) {
        nodePair.second.visible = true;
    }

    // 重置所有边可见性为true
    for (auto &edge : *allEdges) {
        edge.visible = true;
    }

    std::cout << "Reset all visibility to true" << std::endl;
}

// 为指定LOD级别生成聚合边
void VIS4Earth::GraphRenderer::generateAggregatedEdgesForLOD(
    int lodLevel, std::shared_ptr<std::map<std::string, Node>> lodNodes,
    std::shared_ptr<std::vector<Edge>> lodEdges,
    std::shared_ptr<std::map<std::string, Node>> allNodes,
    std::shared_ptr<std::vector<Edge>> allEdges) {

    std::cout << "Generating aggregated edges for LOD " << lodLevel << std::endl;

    // 根据LOD级别选择对应的地理分区
    const std::vector<GeographicRegion> *currentRegions = nullptr;
    switch (lodLevel) {
    case 0:
        currentRegions = &LOD0_REGIONS;
        break;
    case 1:
        currentRegions = &LOD1_REGIONS;
        break;
    case 2:
        currentRegions = &LOD2_REGIONS;
        break;
    default:
        std::cout << "Invalid LOD level for aggregated edges: " << lodLevel << std::endl;
        return;
    }

    // 为每个区域分配当前LOD的节点
    std::map<int, std::vector<std::string>> regionNodes;
    for (const auto &nodePair : *lodNodes) {
        const Node &node = nodePair.second;
        float nodeLat = node.pos.x();
        float nodeLon = node.pos.y();

        // 找到节点所属的地理区域
        for (const auto &region : *currentRegions) {
            if (nodeLat >= region.minLat && nodeLat <= region.maxLat && nodeLon >= region.minLon &&
                nodeLon <= region.maxLon) {
                regionNodes[region.regionId].push_back(nodePair.first);
                break; // 节点只属于第一个匹配的区域
            }
        }
    }

    // 为每个区域选择代表节点（从当前LOD的节点中选择）
    std::map<int, std::string> regionRepresentatives;
    for (const auto &regionPair : regionNodes) {
        int regionId = regionPair.first;
        const std::vector<std::string> &nodeIds = regionPair.second;

        if (nodeIds.empty())
            continue;

        // 选择区域内level最低、degree最高的节点作为代表
        std::string bestNodeId = *std::max_element(
            nodeIds.begin(), nodeIds.end(), [lodNodes](const std::string &a, const std::string &b) {
                const Node &nodeA = lodNodes->at(a);
                const Node &nodeB = lodNodes->at(b);
                if (nodeA.level != nodeB.level) {
                    return nodeA.level > nodeB.level; // level越低优先级越高
                }
                return nodeA.degree < nodeB.degree; // degree越高优先级越高
            });

        regionRepresentatives[regionId] = bestNodeId;

        std::cout << "Region " << regionId << " representative: " << bestNodeId
                  << " (level=" << lodNodes->at(bestNodeId).level
                  << ", degree=" << lodNodes->at(bestNodeId).degree << ")" << std::endl;
    }

    // 创建从节点ID到区域ID的映射（针对所有节点）
    std::map<std::string, int> nodeToRegion;
    for (const auto &nodePair : *allNodes) {
        const Node &node = nodePair.second;
        float nodeLat = node.pos.x();
        float nodeLon = node.pos.y();

        // 找到节点所属的地理区域
        for (const auto &region : *currentRegions) {
            if (nodeLat >= region.minLat && nodeLat <= region.maxLat && nodeLon >= region.minLon &&
                nodeLon <= region.maxLon) {
                nodeToRegion[nodePair.first] = region.regionId;
                break; // 节点只属于第一个匹配的区域
            }
        }
    }

    // 聚合边信息：记录从LOD节点到各区域代表节点的连接
    std::map<std::pair<std::string, std::string>, int> aggregatedConnections;
    std::map<std::pair<std::string, std::string>, float> aggregatedWeights;

    // 遍历所有原始边，查找需要聚合的连接
    for (const auto &edge : *allEdges) {
        if (!edge.visible)
            continue;

        bool fromInLOD = (lodNodes->find(edge.from) != lodNodes->end());
        bool toInLOD = (lodNodes->find(edge.to) != lodNodes->end());

        // 情况1：两端都在LOD中，已经作为直接边处理了，跳过
        if (fromInLOD && toInLOD) {
            continue;
        }

        // 情况2：一端在LOD中，一端不在LOD中，需要聚合
        if (fromInLOD && !toInLOD) {
            // from在LOD中，to不在LOD中，需要连接到to所在区域的代表节点
            auto toRegionIt = nodeToRegion.find(edge.to);
            if (toRegionIt != nodeToRegion.end()) {
                int toRegion = toRegionIt->second;
                auto repIt = regionRepresentatives.find(toRegion);
                if (repIt != regionRepresentatives.end()) {
                    std::pair<std::string, std::string> edgeKey = {edge.from, repIt->second};
                    aggregatedConnections[edgeKey]++;
                    aggregatedWeights[edgeKey] += edge.weight;
                }
            }
        } else if (!fromInLOD && toInLOD) {
            // to在LOD中，from不在LOD中，需要连接到from所在区域的代表节点
            auto fromRegionIt = nodeToRegion.find(edge.from);
            if (fromRegionIt != nodeToRegion.end()) {
                int fromRegion = fromRegionIt->second;
                auto repIt = regionRepresentatives.find(fromRegion);
                if (repIt != regionRepresentatives.end()) {
                    std::pair<std::string, std::string> edgeKey = {repIt->second, edge.to};
                    aggregatedConnections[edgeKey]++;
                    aggregatedWeights[edgeKey] += edge.weight;
                }
            }
        }
        // 情况3：两端都不在LOD中，跳过（可能是高级别LOD才有的连接）
    }

    std::cout << "Found " << aggregatedConnections.size() << " aggregated edge connections"
              << std::endl;

    // 生成聚合边
    static int aggregatedEdgeCounter = 0;
    for (const auto &connPair : aggregatedConnections) {
        const std::string &fromId = connPair.first.first;
        const std::string &toId = connPair.first.second;
        int connectionCount = connPair.second;
        float totalWeight = aggregatedWeights[connPair.first];

        // 检查聚合边的两端节点是否都存在于当前LOD中
        if (lodNodes->find(fromId) == lodNodes->end() || lodNodes->find(toId) == lodNodes->end()) {
            continue; // 如果任一端不在当前LOD中，跳过
        }

        Edge aggregatedEdge;
        aggregatedEdge.id =
            "agg_lod" + std::to_string(lodLevel) + "_" + std::to_string(aggregatedEdgeCounter++);
        aggregatedEdge.from = fromId;
        aggregatedEdge.to = toId;
        aggregatedEdge.weight = totalWeight;
        aggregatedEdge.visible = true;
        aggregatedEdge.isAdd = true; // 标记为聚合边

        // 根据LOD级别和连接数调整高度
        float baseHeight = 30000.0f + (3 - lodLevel) * 15000.0f; // LOD越低高度越高
        aggregatedEdge.maxHeight = baseHeight + connectionCount * 3000.0f;

        lodEdges->push_back(aggregatedEdge);

        std::cout << "Created aggregated edge " << aggregatedEdge.id << " between " << fromId
                  << " and " << toId << " with " << connectionCount
                  << " connections, weight=" << totalWeight << std::endl;
    }
}
// 发光效果控制方法实现
void VIS4Earth::GraphRenderer::PerGraphParam::setGlowIntensity(float intensity) {
    if (mEdgeGeometry && mEdgeGeometry->getStateSet()) {
        auto uniform = mEdgeGeometry->getStateSet()->getUniform("uGlowIntensity");
        if (uniform) {
            uniform->set(intensity);
        }
    }
}

void VIS4Earth::GraphRenderer::PerGraphParam::setGlobalAlpha(float alpha) {
    if (mEdgeGeometry && mEdgeGeometry->getStateSet()) {
        auto uniform = mEdgeGeometry->getStateSet()->getUniform("uGlobalAlpha");
        if (uniform) {
            uniform->set(alpha);
        }
    }
}

void VIS4Earth::GraphRenderer::PerGraphParam::setLineThickness(float thickness) {
    if (mEdgeGeometry && mEdgeGeometry->getStateSet()) {
        auto uniform = mEdgeGeometry->getStateSet()->getUniform("uLineThickness");
        if (uniform) {
            uniform->set(thickness);
        }
    }
}

void VIS4Earth::GraphRenderer::LoadConfigFromTxt(const QString &filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Failed to open config file:" << filePath;
        return;
    }
    int flowType = -1; // 默认无流动
    bool layoutButton = false;
    bool RestrictionButton = false;
    bool EdgeBundlingButton = false;

    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine();
        // 跳过空行和注释行
        if (line.trimmed().isEmpty() || line.trimmed().startsWith('#')) {
            continue;
        }

        // 解析键值对
        QStringList parts = line.split('=');
        if (parts.size() == 2) {
            QString key = parts[0].trimmed();
            QString value = parts[1].trimmed();

            if (key == "attraction") {
                double attraction = value.toDouble();
                ui->spinBoxAttraction->setValue(attraction);
                // setAttraction(attraction);
            } else if (key == "edgeLength") {
                double edgeLength = value.toDouble();
                ui->spinBoxEdgeLength->setValue(edgeLength);
                // setEdgeLength(edgeLength);
            } else if (key == "repulsion") {
                double repulsion = value.toDouble();
                ui->spinBoxRepulsion->setValue(repulsion);
                // setRepulsion(repulsion);
            } else if (key == "springK") {
                double springK = value.toDouble();
                ui->spinBoxSpringK->setValue(springK);
                // setSpringK(springK);
            } else if (key == "iteration") {
                int iteration = value.toInt();
                ui->spinBoxIteration->setValue(iteration);
                // setIteration(iteration);
            } else if (key == "Layout") {
                if (value == "true") {
                    layoutButton = true;
                } else if (value == "false") {
                    layoutButton = false;
                }
            } else if (key == "comboBoxGraphType") {
                int index = value.toInt();
                ui->comboBoxGraphType->setCurrentIndex(index);
                graphTypeIndex = index;
            } else if (key == "pointsFilePath") {
                ui->pointsFilePath->setText(value);
            } else if (key == "edgesFilePath") {
                ui->edgesFilePath->setText(value);
            } else if (key == "flowType") {
                flowType = value.toInt();
            } else if (key == "MinX") {
                double MinX = value.toDouble();
                ui->spinBoxMinX->setValue(MinX);
            } else if (key == "MaxX") {
                double MaxX = value.toDouble();
                ui->spinBoxMaxX->setValue(MaxX);
            } else if (key == "MinY") {
                double MinY = value.toDouble();
                ui->spinBoxMinY->setValue(MinY);
            } else if (key == "MaxY") {
                int MaxY = value.toInt();
                ui->spinBoxMaxY->setValue(MaxY);
            } else if (key == "Restriction") {
                if (value == "true") {
                    RestrictionButton = true;
                } else if (value == "false") {
                    RestrictionButton = false;
                }
            } else if (key == "EdgeBundling") {
                if (value == "true") {
                    EdgeBundlingButton = true;
                } else if (value == "false") {
                    EdgeBundlingButton = false;
                }
            } else if (key == "GlobalSpringConstant") {
                double GlobalSpringConstant = value.toDouble();
                ui->spinBoxGlobalSpringConstant->setValue(GlobalSpringConstant);
            } else if (key == "CompatibilityThreshold") {
                double CompatibilityThreshold = value.toDouble();
                ui->spinBoxCompatibilityThreshold->setValue(CompatibilityThreshold);
            } else if (key == "SmoothWidth") {
                double SmoothWidth = value.toDouble();
                ui->spinBoxSmoothWidth->setValue(SmoothWidth);
            } else if (key == "EdgeWeightThreshold") {
                double EdgeWeightThreshold = value.toDouble();
                ui->spinBoxEdgeWeightThreshold->setValue(EdgeWeightThreshold);
            } else if (key == "EdgePercentageThreshold") {
                double EdgePercentageThreshold = value.toDouble();
                ui->spinBoxEdgePercentageThreshold->setValue(EdgePercentageThreshold);
            } else if (key == "Displacement") {
                double Displacement = value.toDouble();
                ui->spinBoxDisplacement->setValue(Displacement);
            } else if (key == "EdgeDistance") {
                double EdgeDistance = value.toDouble();
                ui->spinBoxEdgeDistance->setValue(EdgeDistance);
                // 可以根据需要添加更多的配置项
            }
        }

        // 第一部分绘制
        loadAndDrawGraph();

        // 第二部分流动
        if (flowType == 0) {
            onHighlightFlowButtonClicked();
        } else if (flowType == 1) {
            onTextureFlowButtonClicked();
        } else if (flowType == 2) {
            onStarFlowButtonClicked();
        }

        // 第三部分图布局
        if (layoutButton) {
            showGraph();
        }
        if (RestrictionButton) {
            setRegionRestriction(true);
        }
        if (EdgeBundlingButton) {
            showBundling();
        }

        file.close();
    }
}
