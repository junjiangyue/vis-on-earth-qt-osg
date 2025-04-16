#include "graph_display.h"
#include "DBSCAN.h"

#include <ui_graph_layout.h>

#include "LOUVAIN.h"
#include "graph_draw.h"
#include <osgText/Font>
#include <osgText/Text>

using namespace VIS4Earth;
static std::array<float, 2> lonRng = {-90.f, 90.f};
const std::array<float, 2> latRng = {-90.f, 90.f};
const std::array<float, 2> hRng = {10000.f, 15000.f};
const float hScale = 10.f;

VIS4Earth::GraphRenderer::CoordRange getCoordRange(const std::shared_ptr<VIS4Earth::Graph> graph) {
    VIS4Earth::GraphRenderer::CoordRange range = {
        std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()};

    for (const auto &node : graph->getNodes()) {
        if (node.second.pos.x < range.minX)
            range.minX = node.second.pos.x;
        if (node.second.pos.x > range.maxX)
            range.maxX = node.second.pos.x;
        if (node.second.pos.y < range.minY)
            range.minY = node.second.pos.y;
        if (node.second.pos.y > range.maxY)
            range.maxY = node.second.pos.y;
    }

    return range;
}

VIS4Earth::GraphRenderer::GraphRenderer(QWidget *parent) : QtOSGReflectableWidget(ui, parent) {

    // 连接 comboBox 的信号来记录当前选择的索引
    graphTypeIndex = 0; // 默认选择第一个（有经纬度的图）

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
    connect(ui->resolutionSlider, &QSlider::valueChanged, this,
            &GraphRenderer::onResolutionSliderValueChanged);

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
    // connect(ui->spinBoxGlobalSpringConstant,
    // QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    //         this, &GraphRenderer::onGlobalSpringConstantChanged);

    //// 连接兼容性阈值
    // connect(ui->spinBoxCompatibilityThreshold,
    // QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    //         this, &GraphRenderer::onCompatibilityThresholdChanged);

    //// 连接平滑宽度
    // connect(ui->spinBoxSmoothWidth, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    //         &GraphRenderer::onSmoothWidthChanged);

    //// 连接位移 (S)
    // connect(ui->spinBoxDisplacement, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    //         &GraphRenderer::onDisplacementChanged);

    //// 连接边距离
    // connect(ui->spinBoxEdgeDistance, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    //         &GraphRenderer::onEdgeDistanceChanged);

    //// 连接边权重阈值
    // connect(ui->spinBoxEdgeWeightThreshold, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    //         this, &GraphRenderer::onEdgeWeightThresholdChanged);

    //// 连接边百分比阈值
    // connect(ui->spinBoxEdgePercentageThreshold,
    //         QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    //         &GraphRenderer::onEdgePercentageThresholdChanged);
    //  连接箭头流动
    /*connect(ui->arrowFlowButton, &QPushButton::clicked, this,
            &GraphRenderer::onArrowFlowButtonClicked);*/
    connect(ui->highlightFlowButton, &QPushButton::clicked, this,
            &GraphRenderer::onHighlightFlowButtonClicked);
    connect(ui->textureFlowButton, &QPushButton::clicked, this,
            &GraphRenderer::onTextureFlowButtonClicked);

    connect(ui->starFlowButton, &QPushButton::clicked, this,
            &GraphRenderer::onStarFlowButtonClicked);
}

void VIS4Earth::GraphRenderer::addGraph(const std::string &name,
                                        std::shared_ptr<std::map<std::string, Node>> nodes,
                                        std::shared_ptr<std::vector<Edge>> edges) {
    auto itr = graphs.find(name);
    if (itr != graphs.end()) {
        param.grp->removeChild(itr->second.grp);
        graphs.erase(itr);
    }
    auto opt = graphs.emplace(std::piecewise_construct, std::forward_as_tuple(name),
                              std::forward_as_tuple(nodes, edges, &param));
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

    // 绘制建筑物
    cityLoader.drawBuildings(param.grp, latLonBounds, scale);
    heightMap = cityLoader.getHeightMap();
    QString pointsFileName = ui->pointsFilePath->text();
    QString edgesFileName = ui->edgesFilePath->text();

    if (pointsFileName.isEmpty() || edgesFileName.isEmpty()) {
        QMessageBox::warning(this, tr("警告"), tr("请先加载点文件和边文件"));
        return;
    }

    // 读取CSV文件中的图数据
    try {
        std::string nodesFile = pointsFileName.toStdString();
        std::string edgesFile = edgesFileName.toStdString();

        auto graph = VIS4Earth::GraphLoader::LoadFromFile(nodesFile, edgesFile);
        auto nodes = std::make_shared<std::map<std::string, Node>>();
        auto edges = std::make_shared<std::vector<Edge>>();
        std::vector<osg::Vec3> colors;
        coordRange = getCoordRange(graph);
        colors.resize(graph->getNodes().size());
        for (auto &col : colors) {
            col.x() = 1.f * rand() / RAND_MAX;
            col.y() = 1.f * rand() / RAND_MAX;
            col.z() = 1.f * rand() / RAND_MAX;
        }
        size_t i = 0;
        for (auto itr = graph->getNodes().begin(); itr != graph->getNodes().end(); ++itr) {
            VIS4Earth::GraphRenderer::Node node;
            node.pos = osg::Vec3(itr->second.pos.x, itr->second.pos.y, 0.f);
            node.color = colors[i];
            node.id = itr->first;
            node.level = itr->second.level;

            nodes->emplace(std::make_pair(itr->first, node));
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
            // graphParam->generateHierarchicalGraphs(nodes, edges);
            // graphParam->setLevelGraph(0);
            graphParam->update();
            // loadMarker();
        }
        myGraph = graph;
        // myGraph = graph;
        compatibilityFuture = std::async(std::launch::async,
                                         [this]() { myGraph->buildCompatibilityListsIfNeeded(); });
        // myGraph.buildCompatibilityListsIfNeeded();
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

    if (pointsFileName.isEmpty() || edgesFileName.isEmpty()) {
        QMessageBox::warning(this, tr("警告"), tr("请先加载点文件和边文件"));
        return;
    }
    // 读取CSV文件中的图数据
    try {
        std::string nodesFile = pointsFileName.toStdString();
        std::string edgesFile = edgesFileName.toStdString();

        auto graph = VIS4Earth::GraphLoader::LoadFromNoGeoFile(nodesFile, edgesFile);
        auto nodes = std::make_shared<std::map<std::string, Node>>();
        auto edges = std::make_shared<std::vector<Edge>>();
        std::vector<osg::Vec3> colors;
        coordRange = getCoordRange(graph);
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
        // graphParam->generateHierarchicalGraphs(nodes, edges);
        // graphParam->setLevelGraph(0);
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
        // showGraph();
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
void sphereToLatLon(const osg::Vec3 &point, double &lon, double &lat, double offset = 60000.0) {
    double radius = osg::WGS_84_RADIUS_POLAR + offset;
    double x = point.x();
    double y = point.y();
    double z = point.z();

    // 计算纬度
    lat = std::asin(z / radius);
    // 计算经度
    lon = std::atan2(y, x);

    // 将弧度转换为角度
    lat = osg::RadiansToDegrees(lat);
    lon = osg::RadiansToDegrees(lon);
}

void VIS4Earth::GraphRenderer::loadMarker() {

    auto vec3ToSphere = [&](const osg::Vec3 &v3) -> osg::Vec3 {
        // v3.x() 是纬度，v3.y() 是经度
        float lat = osg::DegreesToRadians(v3.x()); // 纬度转换为弧度
        float lon = osg::DegreesToRadians(v3.y()); // 经度转换为弧度

        float h = osg::WGS_84_RADIUS_POLAR + v3.z(); // 固定为地球半径，单位为米

        osg::Vec3 ret;
        ret.z() = h * sinf(lat); // 根据纬度计算 Z 坐标

        h = h * cosf(lat); // 根据纬度调整水平投影的半径

        ret.y() = h * sinf(lon); // 根据经度计算 Y 坐标
        ret.x() = h * cosf(lon); // 根据经度计算 X 坐标

        return ret;
    };

    // 检查camera是否已设置
    if (!param._camera) {
        std::cerr << "Camera not set!" << std::endl;
        return;
    }

    // 创建屏幕网格系统
    ScreenGrid screenGrid(1000, 1000, 20.0f, 20.0f);

    auto graphParam = getGraph("LoadedGraph");
    if (!graphParam)
        return;
    // graphParam->grp->removeChildren(0, graphParam->grp->getNumChildren());
    auto nodes = graphParam->getNodes();
    if (!nodes)
        return;

    std::vector<osg::ref_ptr<osgText::Text>> textNodes;

    for (const auto &nodePair : *nodes) {
        const auto &node = nodePair.second;
        if (!node.visible)
            continue;

        // 获取节点在屏幕空间的位置
        osg::Vec3 worldPos = vec3ToSphere(node.pos);
        osg::Vec3 screenPos = projectToScreen(worldPos, param._camera);
        std::pair<int, int> gridCoords = screenGrid.screenToGrid(screenPos.x(), screenPos.y());

        // 创建文本标签
        osg::ref_ptr<osgText::Text> label = new osgText::Text;

        label->setText(nodePair.first);
        label->setFont("Fonts/simhei.ttf");
        label->setAxisAlignment(osgText::Text::SCREEN);
        label->setCharacterSize(graphParam->textSize ? graphParam->textSize * 0.25
                                                     : graphParam->nodeGeomSize * 0.25);
        // label->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
        auto bbx = label->getBoundingBox();
        //// 找到bbx的宽度经纬度
        double lonMin, latMin, lonMax, latMax;
        sphereToLatLon(bbx._min, lonMin, latMin);
        sphereToLatLon(bbx._max, lonMax, latMax);
        double lonDiff = abs(lonMax - lonMin);
        double latDiff = abs(latMax - latMin);
        if (node.isHover) {
            label->setColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
        } else {
            label->setColor(osg::Vec4(0.8f, 0.8f, 0.8f, 1.0f));
        }

        //// 检查网格占用情况
        if (screenGrid.isOccupied(gridCoords.first, gridCoords.second)) {
            // 寻找替代位置
            bool foundSpace = false;
            osg::Vec3 newLabelPosition;
            for (int dx = -1; dx <= 1 && !foundSpace; dx++) {
                for (int dy = -1; dy <= 1 && !foundSpace; ++dy) {
                    int newDx = dx;
                    if (dy == 0) {
                        if (dx > -2 && dx < 1)
                            newDx = -3;
                        if (dx == 1) {
                            newDx = 1;
                        }
                    }
                    int newGridX = gridCoords.first + newDx;
                    int newGridY = gridCoords.second + dy;

                    if (newGridX < 0 || newGridX >= (1000 / 20) || newGridY < 0 ||
                        newGridY >= (1000 / 20))
                        continue;

                    if (!screenGrid.isOccupied(newGridX, newGridY) &&
                        !screenGrid.isOccupied(newGridX + 1, newGridY) &&
                        !screenGrid.isOccupied(newGridX + 2, newGridY)) {

                        // 计算新的标签位置
                        osg::Vec3 newPos = node.pos;
                        newPos.x() += newDx * lonDiff * 200;
                        newPos.y() += dy * latDiff * 200;
                        label->setPosition(vec3ToSphere(newPos));
                        textNodes.push_back(label);
                        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
                        geode->addDrawable(label.get());
                        graphParam->grp->addChild(geode.get());

                        // 标记网格为已占用
                        screenGrid.markOccupied(newGridX, newGridY);
                        screenGrid.markOccupied(newGridX + 1, newGridY);
                        screenGrid.markOccupied(newGridX + 2, newGridY);
                        foundSpace = true;
                    }
                }
            }

            if (!foundSpace) {
                // 如果找不到合适位置，隐藏标签
                label->setNodeMask(0x0);
            }
        } else {
            // 如果当前位置未被占用，直接使用
            label->setPosition(worldPos);
            textNodes.push_back(label);
            osg::ref_ptr<osg::Geode> geode = new osg::Geode;
            geode->addDrawable(label.get());
            graphParam->grp->addChild(geode.get());
            for (int dx = 0; dx < 3; ++dx) {
                for (int dy = 0; dy < 1; ++dy) {
                    screenGrid.markOccupied(gridCoords.first + dx, gridCoords.second + dy);
                }
            }
        }
        /*label->setPosition(vec3ToSphere(worldPos) +
                           osg::Vec3(node.size * (-0.25f) * graphParam->nodeGeomSize,
                                     node.size * (-0.5f) * graphParam->nodeGeomSize,
                                     node.size * 0.30f * graphParam->nodeGeomSize));*/
        // text->setPosition(p +
        //                   osg::Vec3(itr->second.size * (-0.25f) * nodeGeomSize,
        //                             itr->second.size * (-0.5f) * nodeGeomSize,
        //                             itr->second.size * 0.30f *
        //                                 nodeGeomSize)); // 设置文字位置为点的位置稍微向上移动一些
        // 设置标签颜色

        // 添加到场景图
        // graphParam->grp->addChild(geode.get());
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
        // graphParam->generateHierarchicalGraphs(nodes, edges);
        // graphParam->setLevelGraph(0);
        graphParam->update();
        // loadMarker();
    }
}

void VIS4Earth::GraphRenderer::showBundling() {
    auto edgeBundling = VIS4Earth::EdgeBundling();
    edgeBundling.SetGraph(myGraph);
    glm::vec3 gravitationCenter(-75.0, 30.0, 0.0);
    // 确保兼容性计算已完成
    if (compatibilityFuture.valid()) {
        compatibilityFuture.wait(); // 阻塞直到任务完成
    }
    // 在需要边绑定效果时才计算兼容性
    // myGraph.buildCompatibilityListsIfNeeded();

    edgeBundling.SetParameter(mybundlingParam);
    edgeBundling.EdgeBundle();
    myGraph = edgeBundling.GetLayoutedGraph();
    auto lonOffs = 1.5f * (lonRng[1] - lonRng[0]);
    lonRng[0] += lonOffs;
    lonRng[1] += lonOffs;
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
        graphParam->setLongitudeRange(lonRng[0], lonRng[1]);
        graphParam->setLatitudeRange(latRng[0], latRng[1]);
        graphParam->setHeightFromCenterRange(
            static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[0],
            static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[1]);
        graphParam->setNodeGeometrySize(.02f * static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
        graphParam->setTextGeometrySize(.02f * static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
        graphParam->setRestriction(myRestriction);
        graphParam->restrictionOFF = !restrictionOn;
        graphParam->update();
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
    }
}

void VIS4Earth::GraphRenderer::setMinX(double value) { myRestriction.leftBound = value; }

void VIS4Earth::GraphRenderer::setMaxX(double value) { myRestriction.rightBound = value; }

void VIS4Earth::GraphRenderer::setMinY(double value) { myRestriction.bottomBound = value; }

void VIS4Earth::GraphRenderer::setMaxY(double value) { myRestriction.upperBound = value; }

void VIS4Earth::GraphRenderer::onArrowFlowButtonClicked() {
    auto graphParam = getGraph("LoadedGraph");
    graphParam->startArrowAnimation();
}

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
    }
}
void VIS4Earth::GraphRenderer::onResolutionSliderValueChanged(int value) {
    // 将滑块的值转换为百分比
    int percentage = (value * 10);

    // 更新resolutionLabel的文本
    ui->resolutionLabel->setText(QString("分辨率: %1%").arg(percentage));
    auto graphParam = getGraph("LoadedGraph");
    graphParam->graphTypeIndex = graphTypeIndex;
    // graphParam->setLevelGraph(10 - value);
    graphParam->update();
}
// 检查两个矩形是否重叠，并返回重叠的距离
osg::Vec3 calculateOverlapDistance(const osg::BoundingBox &bb1, const osg::BoundingBox &bb2) {
    float overlapY = std::min(bb1.yMax(), bb2.yMax()) - std::max(bb1.yMin(), bb2.yMin());
    float overlapZ = std::min(bb1.zMax(), bb2.zMax()) - std::max(bb1.zMin(), bb2.zMin());
    return osg::Vec3(0.0f, overlapY, overlapZ);
}

// 检查两个矩形是否重叠
bool checkOverlap(const osg::BoundingBox &bb1, const osg::BoundingBox &bb2) {
    return !(bb1.zMax() < bb2.zMin() || bb1.zMin() > bb2.zMax() || bb1.yMax() < bb2.yMin() ||
             bb1.yMin() > bb2.yMax());
}

// 调整文字位置以避免重叠
void adjustTextPosition(std::vector<osg::ref_ptr<osgText::Text>> &texts, float nodeGeomSize) {
    for (size_t i = 0; i < texts.size(); ++i) {
        osg::BoundingBox bb1 = texts[i]->getBoundingBox();
        for (size_t j = 0; j < i; ++j) {
            osg::BoundingBox bb2 = texts[j]->getBoundingBox();
            if (checkOverlap(bb1, bb2)) {
                osg::Vec3 overlap = calculateOverlapDistance(bb1, bb2);
                osg::Vec3 pos1 = texts[i]->getPosition();
                osg::Vec3 pos2 = texts[j]->getPosition();

                // y 代表上下，z 代表左右
                if (pos1.y() < pos2.y()) {
                    pos1.y() -= overlap.y() / 2;
                    pos2.y() += overlap.y() / 2;
                } else {
                    pos1.y() += overlap.y() / 2;
                    pos2.y() -= overlap.y() / 2;
                }

                if (pos1.z() < pos2.z()) {
                    pos1.z() -= overlap.z() / 2;
                    pos2.z() += overlap.z() / 2;
                } else {
                    pos1.z() += overlap.z() / 2;
                    pos2.z() -= overlap.z() / 2;
                }

                texts[i]->setPosition(pos1);
                texts[j]->setPosition(pos2);

                bb1 = texts[i]->getBoundingBox();
                bb2 = texts[j]->getBoundingBox();
            }
            if (checkOverlap(bb1, bb2)) {
                texts[j]->setNodeMask(0x0);
            }
        }
    }
}
class TimeController : public osg::Referenced {
  public:
    TimeController() : startTime(osg::Timer::instance()->tick()) {}

    float getTime() {
        return osg::Timer::instance()->delta_s(startTime, osg::Timer::instance()->tick());
    }

  private:
    osg::Timer_t startTime;
};

void VIS4Earth::GraphRenderer::PerGraphParam::createArrowAnimation(const osg::Vec3 &start,
                                                                   const osg::Vec3 &end,
                                                                   const osg::Vec4 &color,
                                                                   const int startIndex,
                                                                   const int endIndex) {

    auto vec3ToSphere = [&](const osg::Vec3 &v3) -> osg::Vec3 {
        // v3.x() 是纬度，v3.y() 是经度
        float lat = osg::DegreesToRadians(v3.x()); // 纬度转换为弧度
        float lon = osg::DegreesToRadians(v3.y()); // 经度转换为弧度

        float h = osg::WGS_84_RADIUS_POLAR + v3.z(); // 固定为地球半径，单位为米

        osg::Vec3 ret;
        ret.z() = h * sinf(lat); // 根据纬度计算 Z 坐标

        h = h * cosf(lat); // 根据纬度调整水平投影的半径

        ret.y() = h * sinf(lon); // 根据经度计算 Y 坐标
        ret.x() = h * cosf(lon); // 根据经度计算 X 坐标

        return ret;
    };

    auto sphereToVec3 = [&](const osg::Vec3 &sphere) -> osg::Vec3 {
        // 固定的地球半径
        float earthRadius = osg::WGS_84_RADIUS_POLAR;

        // 计算出地心到球面点的实际半径 h
        float h =
            sqrtf(sphere.x() * sphere.x() + sphere.y() * sphere.y() + sphere.z() * sphere.z());

        // 根据高度计算地球表面的距离
        float altitude = h - earthRadius;

        // 计算纬度 lat = asin(z / h)
        float lat = asinf(sphere.z() / h);

        // 计算经度 lon = atan2(y, x)
        float lon = atan2f(sphere.y(), sphere.x());

        // 将纬度和经度从弧度转换为角度
        lat = osg::RadiansToDegrees(lat);
        lon = osg::RadiansToDegrees(lon);

        // 返回值为纬度、经度和高度
        return osg::Vec3(lat, lon, altitude); // z 分量为高度
    };

    // 计算箭头方向和长度
    osg::Vec3 direction = end - start;
    float length = direction.length();
    direction.normalize();

    // 计算插值点

    osg::Vec3Array *lineVertices = new osg::Vec3Array;
    osg::Vec4Array *lineColors = new osg::Vec4Array;

    const int numSubdivisions = 84; // 细分数量
    for (int i = startIndex; i <= endIndex; i++) {
        lineVertices->push_back(segVerts->at(i));
    }
    // osg::Vec3 sstart = (start);
    // osg::Vec3 send = (end);
    // osg::Vec3 newStart = sstart - (send - sstart);
    // osg::Vec3 newEnd = sstart + (send - sstart) * 1;
    // newStart = (newStart);
    // newEnd = (newEnd);
    // osg::Vec3 temp(0.0f, 0.0f, 0.0f);
    //// 插值和渐变颜色处理
    // for (int i = 0; i <= numSubdivisions; ++i) {
    //     float t = static_cast<float>(i) / numSubdivisions;
    //     osg::Vec3 interpolatedPos = start * (1.0f - t) + end * t;

    //    float globalT = (i) / static_cast<float>(numSubdivisions);
    //    interpolatedPos.z() = maxHeight * sin(osg::PI * (globalT));

    //    if (i == numSubdivisions)
    //        interpolatedPos.z() = 0.0f;
    //    osg::Vec3 spherePos = vec3ToSphere(interpolatedPos);
    //    printf("%f,%f 。。。", interpolatedPos.z(), maxHeight);
    //    // printf("%f,%f,%f,%f", spherePos.x(), spherePos.y(), spherePos.z()-65535,maxHeight);

    //    // 添加顶点
    //    lineVertices->push_back(spherePos);
    //}

    // lineVertices->push_back(vec3ToSphere(newEnd));
    //  创建箭头的几何体
    osg::Vec3 arrowHeadBase = end - direction * 1.2f; // 箭头头部基点
    osg::Vec3 left = osg::Vec3(-direction.y(), direction.x(), 0.0f) * 0.8f;
    osg::Vec3 right = osg::Vec3(direction.y(), -direction.x(), 0.0f) * 0.8f;

    osg::Vec3Array *arrowVertices = new osg::Vec3Array;
    osg::Vec4Array *arrowColors = new osg::Vec4Array;

    // 定义箭头三角形的三个顶点
    arrowVertices->push_back(vec3ToSphere(end));                   // 箭头顶点
    arrowVertices->push_back(vec3ToSphere(arrowHeadBase + left));  // 左边
    arrowVertices->push_back(vec3ToSphere(arrowHeadBase + right)); // 右边
    // 初始化颜色数组，alpha值为0（完全透明）
    for (int i = 0; i <= 2; ++i) {
        osg::Vec4 initialColor = color;
        initialColor.a() = 0.4f; // 开始时完全透明
        arrowColors->push_back(initialColor);
    }

    class ArrowAnimationCallback : public osg::NodeCallback {
      public:
        ArrowAnimationCallback(osg::Vec4Array *colors, osg::Geometry *geometry,
                               osg::AnimationPathCallback *animationCallback)
            : _colors(colors), _geometry(geometry), _animationCallback(animationCallback) {}

        virtual void operator()(osg::Node *node, osg::NodeVisitor *nv) override {
            // 调用AnimationPathCallback来执行原来的动画路径逻辑
            if (_animationCallback) {
                (*_animationCallback)(node, nv);
            }

            // 获取当前的动画时间进度
            double currentTime = _animationCallback->getAnimationTime();
            double duration = _animationCallback->getAnimationPath()->getPeriod();

            // 计算进度百分比，确保 t 始终在 0.0 到 1.0 之间
            float t = fmod(static_cast<float>(currentTime / duration), 1.0f);

            // 根据动画进度更新颜色的 alpha 值
            for (size_t i = 0; i < _colors->size(); ++i) {
                if (t <= 0.1f) {
                    // 在 0.0 到 0.2 的范围内，alpha 值从 0 逐渐增加到 1
                    (*_colors)[i].a() = t / 0.1f;
                } else if (t >= 0.9995f) {
                    // 在 0.8 到 1.0 的范围内，alpha 值从 1 逐渐减少到 0
                    (*_colors)[i].a() = (1.0f - t) / 0.1f;
                } else {
                    // 在 0.2 到 0.8 的范围内，alpha 值保持为 1
                    (*_colors)[i].a() = 1.0f;
                }
            }

            // 标记颜色数组已修改
            _colors->dirty();
            _geometry->setColorArray(_colors, osg::Array::BIND_PER_VERTEX); // 重新绑定颜色数组
            _geometry->dirtyDisplayList(); // 标记显示列表为脏
            _geometry->dirtyBound();       // 标记边界为脏（可选）

            // 调用父类的traverse方法
            traverse(node, nv);
        }

      private:
        osg::ref_ptr<osg::Vec4Array> _colors;
        osg::ref_ptr<osg::Geometry> _geometry;
        osg::ref_ptr<osg::AnimationPathCallback> _animationCallback;
    };

    auto arrowGeom = new osg::Geometry;
    arrowGeom->setVertexArray(arrowVertices);
    arrowGeom->setColorArray(arrowColors, osg::Array::BIND_PER_VERTEX);
    arrowGeom->addPrimitiveSet(
        new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, arrowVertices->size()));

    auto arrowGeode = new osg::Geode;
    arrowGeode->addDrawable(arrowGeom);

    // 禁用光照
    auto arrowStates = arrowGeom->getOrCreateStateSet();
    arrowStates->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    arrowStates->setMode(GL_BLEND, osg::StateAttribute::ON);
    arrowStates->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    // 创建动画路径
    osg::ref_ptr<osg::AnimationPath> animationPath = new osg::AnimationPath();
    animationPath->setLoopMode(osg::AnimationPath::LOOP);

    // 插入关键帧
    const double animationDuration = 5.0; // 动画持续时间
    for (int i = 0; i < numSubdivisions; ++i) {
        double time = animationDuration * static_cast<double>(i) / numSubdivisions;
        osg::AnimationPath::ControlPoint point(lineVertices->at(i));
        animationPath->insert(time, point);
    }

    // 创建动画回调
    osg::ref_ptr<osg::AnimationPathCallback> animationCallback =
        new osg::AnimationPathCallback(animationPath);

    // 创建动画 transform
    auto transform = new osg::MatrixTransform;
    transform->addChild(arrowGeode);
    // transform->setUpdateCallback(animationCallback);

    // 创建自定义回调来更新颜色
    osg::ref_ptr<ArrowAnimationCallback> colorCallback =
        new ArrowAnimationCallback(arrowColors, arrowGeom, animationCallback);

    // 添加回调
    transform->setUpdateCallback(colorCallback);

    grp->addChild(transform);
}

osg::Image *VIS4Earth::GraphRenderer::PerGraphParam::createLineDataTexture() {
    auto vec3ToSphere = [&](const osg::Vec3 &v3) -> osg::Vec3 {
        // v3.x() 是纬度，v3.y() 是经度
        float lat = osg::DegreesToRadians(v3.x()); // 纬度转换为弧度
        float lon = osg::DegreesToRadians(v3.y()); // 经度转换为弧度

        float h = osg::WGS_84_RADIUS_POLAR + v3.z(); // 固定为地球半径，单位为米

        osg::Vec3 ret;
        ret.z() = h * sinf(lat); // 根据纬度计算 Z 坐标

        h = h * cosf(lat); // 根据纬度调整水平投影的半径

        ret.y() = h * sinf(lon); // 根据经度计算 Y 坐标
        ret.x() = h * cosf(lon); // 根据经度计算 X 坐标

        return ret;
    };
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
        // 第1行: 起点 (y=1)
        int startPos = (1 * texWidth + x) * 4;
        auto realPos = vec3ToSphere(nodes->at(edge.from).pos);
        data[startPos] = realPos.x();
        data[startPos + 1] = realPos.y();
        data[startPos + 2] = realPos.z();
        data[startPos + 3] = 1.0f;

        // 第2行: 终点 (y=2)
        int endPos = (2 * texWidth + x) * 4;
        auto realEndPos = vec3ToSphere(nodes->at(edge.to).pos);
        data[endPos] = realEndPos.x();
        data[endPos + 1] = realEndPos.y();
        data[endPos + 2] = realEndPos.z();
        data[endPos + 3] = 1.0f;
        x++;
    }

    return image;
}

void VIS4Earth::GraphRenderer::PerGraphParam::startArrowAnimation() {
    arrowFlowEnabled = !arrowFlowEnabled; // 切换箭头流动效果的启停状态

    if (arrowFlowEnabled) {
        std::cout << "Arrow flow enabled" << std::endl;
        std::vector<std::pair<int, int>> edgeRanges;
        int currentIndex = 0;
        // 开始箭头流动效果
        for (auto &edge : *edges) {
            if (!edge.visible)
                continue;                                                 // 只处理可见边
            osg::Vec4 color = osg::Vec4(nodes->at(edge.from).color, 1.f); // 边的颜色
            osg::Vec3 startPos = nodes->at(edge.from).pos;
            osg::Vec3 endPos = nodes->at(edge.to).pos;

            int numVerts = ((edge.subDivs.size() - 1) * 20 + 2) * 2;
            // float maxHeight = edge.maxHeight;
            createArrowAnimation((startPos), (endPos), color, currentIndex,
                                 currentIndex + numVerts - 1);
            currentIndex += numVerts;
        }
    } else {
        std::cout << "Arrow flow disabled" << std::endl;
        // 停止箭头流动效果
        // 可以实现清除箭头效果的逻辑，例如清除相应的节点或设置动画停止等
        grp->removeChildren(0, grp->getNumChildren());
        update(); // 重新绘制图形，移除箭头效果
    }
}

class TextureBasedAnimationCallback : public osg::NodeCallback {
  public:
    TextureBasedAnimationCallback(osg::Image *lineDataImage,
                                  std::shared_ptr<std::vector<GraphRenderer::Edge>> &lines)
        : _lineDataImage(lineDataImage), _lines(lines), _firstFrame(true) {
        // 预分配足够大小的缓存
        _paramCache.resize(lines->size() * 4); // 每个线条4个float(RGBA)
    }

    virtual void operator()(osg::Node *node, osg::NodeVisitor *nv) {
        static double lastTime = nv->getFrameStamp()->getSimulationTime();

        double currentTime = nv->getFrameStamp()->getSimulationTime();
        // 首次运行初始化时间
        if (_firstFrame) {
            lastTime = currentTime;
            _firstFrame = false;
            return; // 跳过第一帧更新
        }
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        // 更新本地缓存
        for (size_t i = 0; i < _lines->size(); ++i) {
            // 独立更新每条线的高光位置
            _lines->at(i).highlightPos =
                fmod(_lines->at(i).highlightPos + _lines->at(i).speed * deltaTime, 1.1f);

            int baseIdx = i * 4;
            _paramCache[baseIdx] = _lines->at(i).highlightPos;
            _paramCache[baseIdx + 1] = _lines->at(i).speed;
            _paramCache[baseIdx + 2] = 0.0f; // 保留
            _paramCache[baseIdx + 3] = 0.0f; // 保留
        }

        // 更新纹理（仅参数行）
        if (_lineDataImage.valid()) {
            float *data = reinterpret_cast<float *>(_lineDataImage->data());
            if (data) {
                const int rowStride = _lineDataImage->s() * 4;
                for (size_t i = 0; i < _lines->size(); ++i) {
                    int dstPos = i * 4; // 第0行参数
                    int srcPos = i * 4;
                    data[dstPos] = _paramCache[srcPos];
                    data[dstPos + 1] = _paramCache[srcPos + 1];
                    data[dstPos + 2] = _paramCache[srcPos + 2];
                    data[dstPos + 3] = _paramCache[srcPos + 3];
                }
                _lineDataImage->dirty();
            }
        }

        traverse(node, nv);
    }

  private:
    osg::ref_ptr<osg::Image> _lineDataImage;
    std::shared_ptr<std::vector<GraphRenderer::Edge>> _lines;
    std::vector<float> _paramCache; // 本地参数缓存
    bool _firstFrame = true;
};

osg::Program *createTextureBasedShaderProgram(int lineCount) {
    std::string vertSource = R"(
#version 120
attribute vec3 vertexPosition;
attribute float lineID;

varying vec3 vPosition;
varying float vLineID;
varying vec3 vLineStart;
varying vec3 vLineEnd;

uniform sampler2D uLineDataTex;
uniform float uTotalLines;
varying vec4 vColor;

void main() {
    vPosition = vertexPosition;
    vLineID = lineID;
    
    // 从纹理获取当前线段的起点终点
    float texX = (lineID + 0.5) / uTotalLines;
    vLineStart = texture2D(uLineDataTex, vec2(texX, 0.25)).rgb;
    vLineEnd = texture2D(uLineDataTex, vec2(texX, 0.5)).rgb;
    vColor = gl_Color;
    gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexPosition, 1.0);
}
)";

    std::string fragSource = R"(
#version 120
uniform sampler2D uLineDataTex;
uniform float uTotalLines;
uniform float uHighlightWidth;
uniform vec4 uHighlightColor;

varying vec3 vPosition;
varying float vLineID;
varying vec3 vLineStart;
varying vec3 vLineEnd;
varying vec4 vColor;

void main() {
    
    // 获取当前线段的高光位置
    float texX = (vLineID + 0.5) / uTotalLines;
    float highlightPos = texture2D(uLineDataTex, vec2(texX, 0.0)).r;
    
    // 计算线段方向和长度
    vec3 lineVec = vLineEnd - vLineStart;
    float lineLength = length(lineVec);
    vec3 lineDir = lineVec / lineLength;
    
    // 计算当前点在直线上的投影
    float t = dot(vPosition - vLineStart, lineDir) / lineLength;
    t = clamp(t, 0.0, 1.0);
    
    // 计算到线段的真实距离（用于线宽控制）
    vec3 projectedPos = vLineStart + t * lineVec;
    float dist = length(vPosition - projectedPos);
    float uHighlightl = lineLength/20;
    
    // 高光强度计算（仅在前向移动方向增强）
    float highlightIntensity = 0.0;
    if(t >= highlightPos - 0.10 && t <= highlightPos) {
        float falloff = 1.0 - smoothstep(highlightPos - 0.05, highlightPos, t);
        highlightIntensity = falloff * exp(-pow((highlightPos - t)/0.1, 2.0));
    }
    
    // 基础颜色
    vec4 baseColor = vec4(15 / 255.f, 176 / 255.0f, 1.f, 0.8f);
    // 最终颜色
    gl_FragColor =mix(baseColor, uHighlightColor, highlightIntensity*1);
}
)";

    osg::ref_ptr<osg::Program> program = new osg::Program;
    // 必须显式绑定属性位置
    program->addBindAttribLocation("vertexPosition", 0);
    program->addBindAttribLocation("lineID", 1);
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));
    return program.release();
}
osg::Program *createTextureBasedShaderProgramColorFlow(int lineCount) {
    std::string vertSource = R"(
#version 120
attribute vec3 vertexPosition;
attribute float lineID;

varying vec3 vPosition;
varying float vLineID;
varying vec3 vLineStart;
varying vec3 vLineEnd;

uniform sampler2D uLineDataTex;
uniform float uTotalLines;
varying vec4 vColor;

void main() {
    vPosition = vertexPosition;
    vLineID = lineID;
    
    // 从纹理获取当前线段的起点终点
    float texX = (lineID + 0.5) / uTotalLines;
    vLineStart = texture2D(uLineDataTex, vec2(texX, 0.25)).rgb;
    vLineEnd = texture2D(uLineDataTex, vec2(texX, 0.5)).rgb;
    vColor = gl_Color;
    gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexPosition, 1.0);
}
)";

    std::string fragSource = R"(
#version 120
uniform sampler2D uLineDataTex;
uniform float uTotalLines;

varying vec3 vPosition;
varying float vLineID;
varying vec3 vLineStart;
varying vec3 vLineEnd;

// 定义颜色（蓝色和黄色）
const vec3 blue = vec3(0.0, 0.0, 0.0);
const vec3 yellow = vec3(1.0, 1.0, 0.0);

void main() {
    // 获取当前线段的动画相位 [0, 1]
    float texX = (vLineID + 0.5) / uTotalLines;
    float phase = texture2D(uLineDataTex, vec2(texX, 0.0)).r;
    
    // 计算当前点在线段上的投影位置 t [0, 1]
    vec3 lineVec = vLineEnd - vLineStart;
    float lineLength = length(lineVec);
    vec3 lineDir = lineVec / lineLength;
    float t = dot(vPosition - vLineStart, lineDir) / lineLength;
    t = clamp(t, 0.0, 1.0);

    // 关键改进：计算颜色权重（使用 cos 实现平滑循环）
    float colorWeight = 0.5 + 0.5 * cos(2.0 * 3.1415926 * (t + phase));
    
    // 混合颜色（蓝色 ↔ 黄色 ↔ 蓝色...）
    vec3 color = mix(blue, yellow, colorWeight);
    
    // 输出颜色（固定透明度 1.0）
    gl_FragColor = vec4(color, 1.0);
}
)";

    osg::ref_ptr<osg::Program> program = new osg::Program;
    // 必须显式绑定属性位置
    program->addBindAttribLocation("vertexPosition", 0);
    program->addBindAttribLocation("lineID", 1);
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));
    return program.release();
}
osg::Program *createTextureBasedShaderProgramStarFlow(int lineCount) {
    std::string vertSource = R"(
#version 120
attribute vec3 vertexPosition;
attribute float lineID;

varying vec3 vPosition;
varying float vLineID;
varying vec3 vLineStart;
varying vec3 vLineEnd;

uniform sampler2D uLineDataTex;
uniform float uTotalLines;
varying vec4 vColor;

void main() {
    vPosition = vertexPosition;
    vLineID = lineID;
    
    // 从纹理获取当前线段的起点终点
    float texX = (lineID + 0.5) / uTotalLines;
    vLineStart = texture2D(uLineDataTex, vec2(texX, 0.25)).rgb;
    vLineEnd = texture2D(uLineDataTex, vec2(texX, 0.5)).rgb;
    vColor = gl_Color;
    gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexPosition, 1.0);
}
)";

    std::string fragSource = R"(
#version 120
uniform sampler2D uLineDataTex;
uniform float uTotalLines;
uniform float uHighlightWidth;
uniform vec4 uHighlightColor;

varying vec3 vPosition;
varying float vLineID;
varying vec3 vLineStart;
varying vec3 vLineEnd;
varying vec4 vColor;

void main() {
    
    // 获取当前线段的高光位置
    float texX = (vLineID + 0.5) / uTotalLines;
    float highlightPos = texture2D(uLineDataTex, vec2(texX, 0.0)).r;
    
    // 计算线段方向和长度
    vec3 lineVec = vLineEnd - vLineStart;
    float lineLength = length(lineVec);
    vec3 lineDir = lineVec / lineLength;
    
    // 计算当前点在直线上的投影
    float t = dot(vPosition - vLineStart, lineDir) / lineLength;
    t = clamp(t, 0.0, 1.0);
    
    // 计算到线段的真实距离（用于线宽控制）
    vec3 projectedPos = vLineStart + t * lineVec;
    float dist = length(vPosition - projectedPos);
    float uHighlightl = 0.05;
    
    // 高光强度计算（仅在前向移动方向增强）
    float highlightIntensity = 0.0;
    if(t >= highlightPos&& t <= highlightPos + 0.05) {
        float posInHighlight = (t - highlightPos) / uHighlightl;  // 0=前端, 1=尾端
        highlightIntensity = smoothstep(0.0, 1.0, posInHighlight);  // 线性增强（尾端最亮）
    }
    
    // 基础颜色
    vec4 baseColor = vec4(15 / 255.f, 176 / 255.0f, 1.f, 0.8f);
    // 最终颜色
    gl_FragColor =mix(baseColor, uHighlightColor, highlightIntensity*1);
}
)";

    osg::ref_ptr<osg::Program> program = new osg::Program;
    // 必须显式绑定属性位置
    program->addBindAttribLocation("vertexPosition", 0);
    program->addBindAttribLocation("lineID", 1);
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));
    return program.release();
}
void VIS4Earth::GraphRenderer::PerGraphParam::startHighlightAnimation() {
    if (isAnimating) {
        // 停止动画
        if (lineGeode) {
            lineGeode->setUpdateCallback(nullptr); // 将颜色设置为初始颜色
            // 2. 直接重置几何体颜色（强制GPU更新）
            osg::Geometry *geom = dynamic_cast<osg::Geometry *>(lineGeode->getDrawable(0));
            if (geom) {
                osg::Vec4Array *colors = new osg::Vec4Array(1);
                (*colors)[0] = osg::Vec4(15 / 255.f, 176 / 255.0f, 1.f, 0.8f); // 初始蓝色
                geom->setColorArray(colors, osg::Array::BIND_OVERALL);
                geom->dirtyDisplayList(); // 比dirtyDisplayList()更彻底
            }

            // 3. 清除所有动画相关状态（关键！）
            osg::StateSet *ss = lineGeode->getStateSet();
            if (ss) {
                ss->removeTextureAttribute(0, osg::StateAttribute::TEXTURE);
                ss->removeUniform("uHighlightColor");
                ss->removeAttribute(osg::StateAttribute::PROGRAM); // 移除着色器
                ss->setMode(GL_LIGHTING, osg::StateAttribute::ON); // 恢复光照
                for (auto &edge : *edges) {
                    edge.highlightPos = 0.0f;
                }
            }
        }
        isAnimating = false;
    } else {
        // 开始动画
        if (lineGeode && lineGeometry) {

            osg::ref_ptr<osg::Image> lineDataImage = createLineDataTexture();
            osg::ref_ptr<osg::Texture2D> lineDataTex = new osg::Texture2D;
            lineDataTex->setImage(lineDataImage);
            lineDataTex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
            lineDataTex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);
            lineDataTex->setResizeNonPowerOfTwoHint(false);

            // 禁用光照
            auto arrowStates = lineGeode->getOrCreateStateSet();
            arrowStates->setAttributeAndModes(createTextureBasedShaderProgram(edges->size()),
                                              osg::StateAttribute::ON);

            // 绑定纹理
            arrowStates->setTextureAttributeAndModes(0, lineDataTex, osg::StateAttribute::ON);
            arrowStates->addUniform(new osg::Uniform("uLineDataTex", 0));
            arrowStates->addUniform(
                new osg::Uniform("uTotalLines", static_cast<float>(edges->size())));
            arrowStates->addUniform(new osg::Uniform("uHighlightWidth", 20.f));
            arrowStates->addUniform(
                new osg::Uniform("uHighlightColor", osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)));
            lineGeode->setUpdateCallback(new TextureBasedAnimationCallback(lineDataImage, edges));
        }
        isAnimating = true;
    }
}

void VIS4Earth::GraphRenderer::PerGraphParam::startTextureAnimation() {

    if (lineGeode && lineGeometry) {
        if (isAnimating) {
            // 当前正在动画中，结束动画
            lineGeode->setUpdateCallback(nullptr); // 将颜色设置为初始颜色
            // 2. 直接重置几何体颜色（强制GPU更新）
            osg::Geometry *geom = dynamic_cast<osg::Geometry *>(lineGeode->getDrawable(0));
            if (geom) {
                osg::Vec4Array *colors = new osg::Vec4Array(1);
                (*colors)[0] = osg::Vec4(15 / 255.f, 176 / 255.0f, 1.f, 0.8f); // 初始蓝色
                geom->setColorArray(colors, osg::Array::BIND_OVERALL);
                geom->dirtyDisplayList(); // 比dirtyDisplayList()更彻底
            }

            // 3. 清除所有动画相关状态（关键！）
            osg::StateSet *ss = lineGeode->getStateSet();
            if (ss) {
                ss->removeTextureAttribute(0, osg::StateAttribute::TEXTURE);
                ss->removeUniform("uHighlightColor");
                ss->removeAttribute(osg::StateAttribute::PROGRAM); // 移除着色器
                ss->setMode(GL_LIGHTING, osg::StateAttribute::ON); // 恢复光照
                for (auto &edge : *edges) {
                    edge.highlightPos = 0.0f;
                }
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
                auto arrowStates = lineGeode->getOrCreateStateSet();
                arrowStates->setAttributeAndModes(
                    createTextureBasedShaderProgramColorFlow(edges->size()),
                    osg::StateAttribute::ON);

                // 绑定纹理
                arrowStates->setTextureAttributeAndModes(0, lineDataTex, osg::StateAttribute::ON);
                arrowStates->addUniform(new osg::Uniform("uLineDataTex", 0));
                arrowStates->addUniform(
                    new osg::Uniform("uTotalLines", static_cast<float>(edges->size())));
                arrowStates->addUniform(new osg::Uniform("uHighlightWidth", 20.f));
                arrowStates->addUniform(
                    new osg::Uniform("uHighlightColor", osg::Vec4(246.f, 66.f, 14.f, 1.0f)));
                lineGeode->setUpdateCallback(
                    new TextureBasedAnimationCallback(lineDataImage, edges));
            }
            isAnimating = true;
        }
    }
}

void VIS4Earth::GraphRenderer::PerGraphParam::startStarAnimation() {

    if (lineGeode && lineGeometry) {
        if (isAnimating) {
            // 当前正在动画中，结束动画
            lineGeode->setUpdateCallback(nullptr); // 将颜色设置为初始颜色
            // 2. 直接重置几何体颜色（强制GPU更新）
            osg::Geometry *geom = dynamic_cast<osg::Geometry *>(lineGeode->getDrawable(0));
            if (geom) {
                osg::Vec4Array *colors = new osg::Vec4Array(1);
                (*colors)[0] = osg::Vec4(15 / 255.f, 176 / 255.0f, 1.f, 0.8f); // 初始蓝色
                geom->setColorArray(colors, osg::Array::BIND_OVERALL);
                geom->dirtyDisplayList(); // 比dirtyDisplayList()更彻底
            }

            // 3. 清除所有动画相关状态（关键！）
            osg::StateSet *ss = lineGeode->getStateSet();
            if (ss) {
                ss->removeTextureAttribute(0, osg::StateAttribute::TEXTURE);
                ss->removeUniform("uHighlightColor");
                ss->removeAttribute(osg::StateAttribute::PROGRAM); // 移除着色器
                ss->setMode(GL_LIGHTING, osg::StateAttribute::ON); // 恢复光照
                for (auto &edge : *edges) {
                    edge.highlightPos = 0.0f;
                }
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
                auto arrowStates = lineGeode->getOrCreateStateSet();
                arrowStates->setAttributeAndModes(
                    createTextureBasedShaderProgramStarFlow(edges->size()),
                    osg::StateAttribute::ON);

                // 绑定纹理
                arrowStates->setTextureAttributeAndModes(0, lineDataTex, osg::StateAttribute::ON);
                arrowStates->addUniform(new osg::Uniform("uLineDataTex", 0));
                arrowStates->addUniform(
                    new osg::Uniform("uTotalLines", static_cast<float>(edges->size())));
                arrowStates->addUniform(new osg::Uniform("uHighlightWidth", 20.f));
                arrowStates->addUniform(
                    new osg::Uniform("uHighlightColor", osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)));
                lineGeode->setUpdateCallback(
                    new TextureBasedAnimationCallback(lineDataImage, edges));
            }
            isAnimating = true;
        }
    }
}
osg::Vec4 generateColor(float index) {
    std::vector<osg::Vec4> predefinedColors = {
        osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), // 红色
        osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f), // 绿色
        osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f), // 蓝色
        osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f), // 黄色
        osg::Vec4(0.0f, 1.0f, 1.0f, 1.0f), // 青色
        osg::Vec4(1.0f, 0.0f, 1.0f, 1.0f), // 品红色
        osg::Vec4(0.5f, 0.5f, 0.5f, 1.0f), // 灰色
        osg::Vec4(1.0f, 0.5f, 0.0f, 1.0f), // 橙色
        osg::Vec4(0.5f, 0.0f, 0.5f, 1.0f), // 紫色
        osg::Vec4(0.0f, 0.5f, 0.5f, 1.0f), // 深青色
        osg::Vec4(0.3f, 0.3f, 0.7f, 1.0f)  // 其他颜色
    };

    if (index < 0)
        index = 0;
    if (index >= predefinedColors.size())
        index = predefinedColors.size() - 1;
    return predefinedColors[index];
}
// 找到满足条件的最小顶点高度 h'
float findOptimalHeight(float p, float max, const std::vector<float> &heightArray,
                        float maxHeightInArray) {

    float h_max = maxHeightInArray * 1.1; // 最大顶点高度

    // 检查是否满足条件
    while (true) {
        // 计算系数 a1 和 a2
        float a1 = (heightArray[0] - h_max) / (p * p);
        float a2 = (heightArray.back() - h_max) / ((max - p) * (max - p));

        // 检查是否满足上界条件
        bool valid = true;
        for (size_t i = 0; i < heightArray.size(); ++i) {
            float x = static_cast<float>(i);
            float f_x;
            if (x <= p) {
                f_x = a1 * (x - p) * (x - p) + h_max;
            } else {
                f_x = a2 * (x - p) * (x - p) + h_max;
            }
            if (f_x < heightArray[i]) {
                valid = false;
                break;
            }
        }

        // 如果满足条件，返回 h'
        if (valid) {
            return h_max;
        }

        // 否则增加 h'
        h_max *= 1.1f; // 增加 10%
    }

    return h_max;
}
void VIS4Earth::GraphRenderer::PerGraphParam::update() {
    grp->removeChildren(0, grp->getNumChildren());

    auto vec3ToSphere = [&](const osg::Vec3 &v3) -> osg::Vec3 {
        // v3.x() 是纬度，v3.y() 是经度
        float lat = osg::DegreesToRadians(v3.x()); // 纬度转换为弧度
        float lon = osg::DegreesToRadians(v3.y()); // 经度转换为弧度

        float h = osg::WGS_84_RADIUS_POLAR + v3.z(); // 固定为地球半径，单位为米

        osg::Vec3 ret;
        ret.z() = h * sinf(lat); // 根据纬度计算 Z 坐标

        h = h * cosf(lat); // 根据纬度调整水平投影的半径

        ret.y() = h * sinf(lon); // 根据经度计算 Y 坐标
        ret.x() = h * cosf(lon); // 根据经度计算 X 坐标

        return ret;
    };

    auto tessl = new osg::TessellationHints;
    tessl->setDetailRatio(1.f);
    std::map<std::string, osg::ShapeDrawable *> osgNodes;
    std::vector<osg::ref_ptr<osgText::Text>> textNodes;

    for (auto itr = nodes->begin(); itr != nodes->end(); ++itr) {
        if (!itr->second.visible)
            continue; // 只处理可见节点
        osg::Vec4 color = generateColor(static_cast<float>(itr->second.cluster));

        if (!restrictionOFF) {
            if (itr->second.pos.x() >= restriction.leftBound &&
                itr->second.pos.x() <= restriction.rightBound &&
                itr->second.pos.y() >= restriction.bottomBound &&
                itr->second.pos.y() <= restriction.upperBound) {
                color = osg::Vec4(1.0f, 1.0f, 1.0f, 0.5f); // 设置边框内的点为半透明白色
            }
        }
        auto p = itr->second.pos;
        p.z() = getBuildingHeightAtLatLon(p.x(), p.y()) + 100.0f;
        p = vec3ToSphere(p);
        auto sphere = new osg::ShapeDrawable(
            new osg::Sphere(p, itr->second.size * .10f * nodeGeomSize), tessl);
        osg::ref_ptr<osg::Vec3Array> centerData = new osg::Vec3Array;
        centerData->push_back(itr->second.pos);
        sphere->setUserData(centerData);

        sphere->setColor(color);

        auto states = grp->getOrCreateStateSet();
        auto matr = new osg::Material;
        matr->setColorMode(osg::Material::DIFFUSE);
        states->setAttributeAndModes(matr, osg::StateAttribute::ON);
        states->setMode(GL_LIGHTING, osg::StateAttribute::ON);
        states->setMode(GL_BLEND, osg::StateAttribute::ON); // 开启混合模式
        // osg::setNotifyLevel(osg::DEBUG_INFO);

        grp->addChild(sphere);

        osg::ref_ptr<osgText::Text> text = new osgText::Text;
        text->setText(itr->first);
        text->setFont("Fonts/simhei.ttf"); // 设置字体
        text->setAxisAlignment(osgText::Text::SCREEN);
        if (textSize) {
            text->setCharacterSize(textSize * 0.25); // 设置字体大小
        } else {
            text->setCharacterSize(nodeGeomSize * 0.25);
        }
        // text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
        //  TODO: 加入文字避让
        text->setPosition(p +
                          osg::Vec3(itr->second.size * (-0.25f) * nodeGeomSize,
                                    itr->second.size * (-0.25f) * nodeGeomSize,
                                    itr->second.size * 0.30f *
                                        nodeGeomSize)); // 设置文字位置为点的位置稍微向上移动一些
                                                        // 设置文字内容为点的ID
        text->setColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)); // 设置文字颜色为白色
        text->setAxisAlignment(osgText::Text::SCREEN);     // 屏幕对齐，始终面向相机
        osg::ref_ptr<osg::Geode> textGeode = new osg::Geode;
        textGeode->addDrawable(text.get());
        textNodes.push_back(text);
        grp->addChild(textGeode.get());
        osgNodes.emplace(std::make_pair(itr->first, sphere));
    }
    // 调用新的loadMarker函数来处理标签
    // loadMarker();
    adjustTextPosition(textNodes, nodeGeomSize);

    auto segVerts = new osg::Vec3Array;
    auto segCols = new osg::Vec4Array;
    osg::ref_ptr<osg::FloatArray> lineIDs = new osg::FloatArray;
    int lineID = 0;
    for (auto &edge : *edges) {
        if (!edge.visible)
            continue; // 只处理可见边

        osg::Vec4 prevColor = osg::Vec4(nodes->at(edge.from).color, 0.5f);
        auto dCol = osg::Vec4(nodes->at(edge.to).color, 1.f) - prevColor;
        dCol /= (edge.subDivs.size() == 1 ? 1 : edge.subDivs.size() - 1);

        osg::Vec3 prevPos = nodes->at(edge.from).pos;
        osg::Vec3 startPos = vec3ToSphere(prevPos); // 起点
        osg::Vec3 endPos = nodes->at(edge.to).pos;
        endPos = vec3ToSphere(endPos);

        // 总点数，包括起点、所有细分点和终点
        size_t totalPoints = (edge.subDivs.size() - 1) * 81; // 起点 + 细分点 + 终点
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

            for (int i = 1; i < 42; i++) {
                float t = static_cast<float>(i) / (42);
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
                for (int j = 0; j <= 80; ++j) { // 包含起点和终点
                    float t = static_cast<float>(j) / 80.0f;

                    osg::Vec3 interpolatedPos;
                    interpolatedPos.x() = prevPos.x() * (1.0f - t) + currentPos.x() * t;
                    interpolatedPos.y() = prevPos.y() * (1.0f - t) + currentPos.y() * t;
                    float globalT = ((i - 1) * 81 + (j + 1)) / static_cast<float>(totalPoints);
                    interpolatedPos.z() = std::max(
                        getBuildingHeightAtLatLon(interpolatedPos.x(), interpolatedPos.y()),
                        float(maxHeight * sin(osg::PI * globalT))); // 平滑高度变化
                    // interpolatedPos.z() = 0.f;
                    osg::Vec4 interpolatedColor = prevColor * (1.0f - t) + currentColor * t;

                    if (j >= 0) {
                        segVerts->push_back(vec3ToSphere(prevInterpolatedPos));
                        segCols->push_back(osg::Vec4(15 / 255.f, 176 / 255.0f, 1.f, 0.8f));
                        segVerts->push_back(vec3ToSphere(interpolatedPos));
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
        states->setMode(GL_BLEND, osg::StateAttribute::ON); // 开启混合模式
        osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc();
        blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        states->setAttributeAndModes(blendFunc, osg::StateAttribute::ON);
        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, segVerts->size()));
        auto lw = new osg::LineWidth(1.f);
        states->setAttributeAndModes(lw, osg::StateAttribute::ON);

        // 启用混合（Blending）以支持透明度
        geom->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);

        // 设置混合函数
        geom->getOrCreateStateSet()->setAttributeAndModes(
            new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), osg::StateAttribute::ON);

        // 设置渲染顺序以确保透明物体正确渲染
        geom->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        geom->getOrCreateStateSet()->setAttributeAndModes(
            new osg::Depth(osg::Depth::LESS, 0.0, 1.0, false), osg::StateAttribute::ON);

        auto geode = new osg::Geode;
        geode->addDrawable(geom);

        // 保存 Geode 和 Geometry
        lineGeode = geode;
        lineGeometry = geom;

        grp->addChild(geode);
    }
}

void VIS4Earth::GraphRenderer::PerGraphParam::setRestriction(VIS4Earth::Area res) {
    restriction = res;
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

void VIS4Earth::GraphRenderer::PerGraphParam::setLevelGraph(int level) {
    nodes = levels[level].nodes;
    edges = levels[level].edges;
    nodeMapping = levels[level].nodeMapping;
    edgeMapping = levels[level].edgeMapping;
}

void GraphRenderer::PerGraphParam::generateHierarchicalGraphs(
    std::shared_ptr<std::map<std::string, Node>> &initialNodes,
    std::shared_ptr<std::vector<Edge>> &initialEdges) {

    int numLevels = 11; // 总共生成11层图
    std::vector<GraphLevel> mylevels(numLevels);

    // 第0层次是原始图
    mylevels[0].nodes = std::make_shared<std::map<std::string, Node>>(*initialNodes);
    mylevels[0].edges = std::make_shared<std::vector<Edge>>(*initialEdges);
    mylevels[0].nodeMapping =
        std::shared_ptr<std::map<std::string, std::vector<std::string>>>(); // 节点映射
    mylevels[0].edgeMapping = std::shared_ptr<std::map<Edge, std::vector<Edge>>>();

    // 生成其他层次的图
    for (int level = 1; level < numLevels; ++level) {
        mylevels[level].nodes = std::make_shared<std::map<std::string, Node>>();
        mylevels[level].edges = std::make_shared<std::vector<Edge>>();
        mylevels[level].nodeMapping =
            std::shared_ptr<std::map<std::string, std::vector<std::string>>>(); // 节点映射
        mylevels[level].edgeMapping = std::shared_ptr<std::map<Edge, std::vector<Edge>>>();

        // 对前一个层次的图执行 DBSCAN 聚类
        performClustering(mylevels[level - 1], mylevels[level], level);
    }

    levels = mylevels; // 将生成的层次存储到成员变量
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
            clusterLabels = DBSCAN(positions, 4, /*minPts*/ 1, dbscanedges, nodeIds);
            // clusterLabels = Louvain(dbscanedges, weights); // 全都是一个社区的 数据再改变一下
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
            // 设置代表节点的大小，基于簇中节点的数量
            float representativeSize =
                (static_cast<float>(nodesInCluster.size()) * 0.05 +
                 previousLevel.nodes->at(representativeNodeId).size); // 根据节点数量设置大小
            currentLevel.nodes->at(representativeNodeId).size = representativeSize;

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

    // 2. 为未直接连接但联通的代表节点添加新边
    for (const auto &repNodePair1 : *currentLevel.nodes) {
        if (!repNodePair1.second.isRepresent)
            continue; // 只对代表节点进行处理

        for (const auto &repNodePair2 : *currentLevel.nodes) {
            if (repNodePair1.first == repNodePair2.first || !repNodePair2.second.isRepresent)
                continue; // 跳过自己或非代表节点

            std::string from = std::min(repNodePair1.first, repNodePair2.first);
            std::string to = std::max(repNodePair1.first, repNodePair2.first);

            // 如果这条边已经处理过，则跳过
            if (processedEdges.count({from, to}) > 0)
                continue;

            // 检查这两个代表节点在上一层是否通过某种方式连接
            bool isConnected = false;
            for (const std::string &originalNode1 : nodeMapping.at(repNodePair1.first)) {
                for (const std::string &originalNode2 : nodeMapping.at(repNodePair2.first)) {
                    // 检查是否有直接连接的边
                    for (const Edge &edge : *previousLevel.edges) {
                        if ((edge.from == originalNode1 && edge.to == originalNode2) ||
                            (edge.from == originalNode2 && edge.to == originalNode1)) {
                            isConnected = true;
                            break;
                        }
                    }
                    if (isConnected)
                        break;
                }
                if (isConnected)
                    break;
            }

            // 如果两个代表节点在上一级中连接，则在当前层中添加一条直接的边
            if (isConnected) {
                Edge newEdge;
                newEdge.from = from;
                newEdge.to = to;

                // 设置细分点
                auto itFrom = currentLevel.nodes->find(from);
                auto itTo = currentLevel.nodes->find(to);
                if (itFrom != currentLevel.nodes->end() && itTo != currentLevel.nodes->end()) {
                    newEdge.subDivs.emplace_back(itFrom->second.pos);
                    newEdge.subDivs.emplace_back(itTo->second.pos);
                }

                currentLevel.edges->push_back(newEdge);

                // 记录边映射
                edgeMapping[newEdge] = {};

                // 标记已处理的边
                processedEdges.insert({from, to});
            }
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
