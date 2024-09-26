#include "graph_display.h"
#include "DBSCAN.h"

#include <ui_graph_layout.h>

#include <osgText/Font>
#include <osgText/Text>

using namespace VIS4Earth;
static std::array<float, 2> lonRng = {-60.f, -30.f};
const std::array<float, 2> latRng = {-20.f, 20.f};
const std::array<float, 2> hRng = {10000.f, 15000.f};
const float hScale = 10.f;

VIS4Earth::GraphRenderer::CoordRange getCoordRange(const VIS4Earth::Graph &graph) {
    VIS4Earth::GraphRenderer::CoordRange range = {
        std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()};

    for (const auto &node : graph.getNodes()) {
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

    // Connect the slider's valueChanged signal to the slot function
    connect(ui->sizeSlider, &QSlider::valueChanged, this, &GraphRenderer::onSizeSliderValueChanged);
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
    /*connect(ui->checkBoxRegionRestriction, &QCheckBox::toggled, this,
            &GraphRenderer::setRegionRestriction);*/
    connect(ui->spinBoxMinX, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &GraphRenderer::setMinX);
    connect(ui->spinBoxMaxX, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &GraphRenderer::setMaxX);
    connect(ui->spinBoxMinY, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &GraphRenderer::setMinY);
    connect(ui->spinBoxMaxY, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &GraphRenderer::setMaxY);

    // 连接全局弹簧常数 (K)
    connect(ui->spinBoxGlobalSpringConstant, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GraphRenderer::onGlobalSpringConstantChanged);

    // 连接迭代次数 (I)
    connect(ui->spinBoxNumberOfIterations, QOverload<int>::of(&QSpinBox::valueChanged), this,
            &GraphRenderer::onNumberOfIterationsChanged);

    // 连接剩余迭代次数 (iter)
    connect(ui->spinBoxRemainingIterations, QOverload<int>::of(&QSpinBox::valueChanged), this,
            &GraphRenderer::onRemainingIterationsChanged);

    // 连接剩余循环数
    connect(ui->spinBoxCyclesLeft, QOverload<int>::of(&QSpinBox::valueChanged), this,
            &GraphRenderer::onCyclesLeftChanged);

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

    // 连接引力开启
    connect(ui->checkBoxGravitationIsOn, &QCheckBox::toggled, this,
            &GraphRenderer::onGravitationIsOnToggled);

    // 连接引力中心
    connect(ui->spinBoxGravitationCenterX, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GraphRenderer::onGravitationCenterXChanged);
    connect(ui->spinBoxGravitationCenterY, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GraphRenderer::onGravitationCenterYChanged);
    connect(ui->spinBoxGravitationCenterZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GraphRenderer::onGravitationCenterZChanged);

    // 连接引力指数
    connect(ui->spinBoxGravitationExponent, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GraphRenderer::onGravitationExponentChanged);

    // 连接边权重阈值
    connect(ui->spinBoxEdgeWeightThreshold, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GraphRenderer::onEdgeWeightThresholdChanged);

    // 连接边百分比阈值
    connect(ui->spinBoxEdgePercentageThreshold,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            &GraphRenderer::onEdgePercentageThresholdChanged);
    // 连接箭头流动
    connect(ui->arrowFlowButton, &QPushButton::clicked, this,
            &GraphRenderer::onArrowFlowButtonClicked);
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
        colors.resize(graph.getNodes().size());
        for (auto &col : colors) {
            col.x() = 1.f * rand() / RAND_MAX;
            col.y() = 1.f * rand() / RAND_MAX;
            col.z() = 1.f * rand() / RAND_MAX;
        }
        size_t i = 0;
        for (auto itr = graph.getNodes().begin(); itr != graph.getNodes().end(); ++itr) {
            VIS4Earth::GraphRenderer::Node node;
            node.pos = osg::Vec3(itr->second.pos.x, itr->second.pos.y, 0.f);
            node.color = colors[i];
            node.id = itr->first;
            node.level = itr->second.level;

            nodes->emplace(std::make_pair(itr->first, node));
            ++i;
        }

        for (auto itr = graph.getEdges().begin(); itr != graph.getEdges().end(); ++itr) {
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

            graphParam->setLongitudeRange(lonRng[0] * size, lonRng[1] * size);
            graphParam->setLatitudeRange(latRng[0] * size, latRng[1] * size);
            graphParam->setHeightFromCenterRange(
                static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[0],
                static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[1]);
            graphParam->setNodeGeometrySize(.02f * static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
            graphParam->setTextGeometrySize(.02f * static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
            graphParam->update();
            graphParam->generateHierarchicalGraphs(nodes, edges);
        }
        myGraph = graph;
        // 初始化 UI
        QLabel *coordRangeLabel = ui->labelCurrentCoordRange; // 假设使用 ui 指针来访问 UI 元素
        QString text =
            QString("当前坐标范围: 最低纬度: %1, 最高纬度: %2, 最大经度: %3, 最小经度: %4")
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
        colors.resize(graph.getNodes().size());
        for (auto &col : colors) {
            col.x() = 1.f * rand() / RAND_MAX;
            col.y() = 1.f * rand() / RAND_MAX;
            col.z() = 1.f * rand() / RAND_MAX;
        }
        size_t i = 0;
        for (auto itr = graph.getNodes().begin(); itr != graph.getNodes().end(); ++itr) {
            VIS4Earth::GraphRenderer::Node node;
            // 初始化pos
            node.pos = osg::Vec3(itr->second.pos.x, itr->second.pos.y, 0.f);
            node.color = colors[i];
            node.id = itr->first;
            node.level = itr->second.level;

            nodes->emplace(std::make_pair(itr->first, node));
            ++i;
        }
        // pos有问题
        for (auto itr = graph.getEdges().begin(); itr != graph.getEdges().end(); ++itr) {
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
        // 添加图到渲染器中
        addGraph("LoadedGraph", nodes, edges);

        showGraph();
        // 初始化 UI
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
    auto nodeLayouter = VIS4Earth::NodeLayouter();
    nodeLayouter.setGraph(myGraph);
    nodeLayouter.setParameter(myLayoutParam);
    nodeLayouter.layout(myLayoutParam.Iteration);
    myGraph = nodeLayouter.getLayoutedGraph();
    auto existGraph = getGraph("GraphLayout");
    if (!existGraph) {
        auto lonOffs = 1.5f * (lonRng[1] - lonRng[0]);
        lonRng[0] += lonOffs;
        lonRng[1] += lonOffs;
    }
    auto nodes = std::make_shared<std::map<std::string, Node>>();
    auto edges = std::make_shared<std::vector<Edge>>();
    std::vector<osg::Vec3> colors;
    colors.resize(myGraph.getNodes().size());
    for (auto &col : colors) {
        col.x() = 1.f * rand() / RAND_MAX;
        col.y() = 1.f * rand() / RAND_MAX;
        col.z() = 1.f * rand() / RAND_MAX;
    }
    size_t i = 0;
    for (auto itr = myGraph.getNodes().begin(); itr != myGraph.getNodes().end(); ++itr) {
        VIS4Earth::GraphRenderer::Node node;
        node.pos = osg::Vec3(itr->second.pos.x, itr->second.pos.y, 0.f);
        node.color = colors[i];

        nodes->emplace(std::make_pair(itr->first, node));
        ++i;
    }

    for (auto itr = myGraph.getEdges().begin(); itr != myGraph.getEdges().end(); ++itr) {
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
    addGraph("GraphLayout", nodes, edges);
    // 更新图渲染
    auto graphParam = getGraph("GraphLayout");
    if (graphParam) {
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

void VIS4Earth::GraphRenderer::showBundling() {
    auto edgeBundling = VIS4Earth::EdgeBundling();
    edgeBundling.SetGraph(myGraph);
    glm::vec3 gravitationCenter(0.0, 0.0, 0.0);

    // VIS4Earth::EdgeBundling::BundlingParam bundlingParam = {
    //     bundlingParam.K = 0.1,
    //     bundlingParam.cycles = 5,
    //     bundlingParam.I = 90,
    //     bundlingParam.compatibilityThreshold = 0.6,
    //     bundlingParam.smoothWidth = 3,
    //     bundlingParam.edgeWeightThreshold = -1.0,
    //     bundlingParam.edgePercentageThreshold = -1.0,
    //     bundlingParam.S = 0.4,
    //     bundlingParam.edgeDistance = 1e-4,
    //     bundlingParam.gravitationCenter = gravitationCenter,
    //     bundlingParam.gravitationExponent = 1.0};
    //
    mybundlingParam.K = 0.1, mybundlingParam.cycles = 5, mybundlingParam.I = 90,
    mybundlingParam.compatibilityThreshold = 0.6, mybundlingParam.smoothWidth = 3,
    mybundlingParam.edgeWeightThreshold = -1.0, mybundlingParam.edgePercentageThreshold = -1.0,
    mybundlingParam.S = 0.4, mybundlingParam.edgeDistance = 1e-4,
    mybundlingParam.gravitationCenter = gravitationCenter,
    mybundlingParam.gravitationExponent = 1.0;

    edgeBundling.SetParameter(mybundlingParam);
    edgeBundling.EdgeBundle();
    myGraph = edgeBundling.GetLayoutedGraph();
    auto lonOffs = 1.5f * (lonRng[1] - lonRng[0]);
    lonRng[0] += lonOffs;
    lonRng[1] += lonOffs;
    auto nodes = std::make_shared<std::map<std::string, Node>>();
    auto edges = std::make_shared<std::vector<Edge>>();
    std::vector<osg::Vec3> colors;
    colors.resize(myGraph.getNodes().size());
    for (auto &col : colors) {
        col.x() = 1.f * rand() / RAND_MAX;
        col.y() = 1.f * rand() / RAND_MAX;
        col.z() = 1.f * rand() / RAND_MAX;
    }
    size_t i = 0;
    for (auto itr = myGraph.getNodes().begin(); itr != myGraph.getNodes().end(); ++itr) {
        VIS4Earth::GraphRenderer::Node node;
        node.pos = osg::Vec3(itr->second.pos.x, itr->second.pos.y, 0.f);
        node.color = colors[i];

        nodes->emplace(std::make_pair(itr->first, node));
        ++i;
    }

    for (auto itr = myGraph.getEdges().begin(); itr != myGraph.getEdges().end(); ++itr) {
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
    addGraph("edge_clustered", nodes, edges);
    // 更新图渲染
    auto graphParam = getGraph("edge_clustered");
    if (graphParam) {
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
    myRestriction.leftBound = -1000.0;
    myRestriction.rightBound = -800.0;
    myRestriction.upperBound = -300.0;
    myRestriction.bottomBound = -400.0;
    auto nodeLayouter = VIS4Earth::NodeLayouter();
    nodeLayouter.setGraph(myGraph);
    nodeLayouter.setParameter(myLayoutParam);
    nodeLayouter.restrictedLayout(myRestriction, myLayoutParam.Iteration);
    myGraph = nodeLayouter.getLayoutedGraph();
    auto existGraph = getGraph("Layout_restrict");
    if (!existGraph) {
        auto lonOffs = 1.5f * (lonRng[1] - lonRng[0]);
        lonRng[0] += lonOffs;
        lonRng[1] += lonOffs;
    }
    auto nodes = std::make_shared<std::map<std::string, Node>>();
    auto edges = std::make_shared<std::vector<Edge>>();
    std::vector<osg::Vec3> colors;
    colors.resize(myGraph.getNodes().size());
    for (auto &col : colors) {
        col.x() = 1.f * rand() / RAND_MAX;
        col.y() = 1.f * rand() / RAND_MAX;
        col.z() = 1.f * rand() / RAND_MAX;
    }
    size_t i = 0;
    for (auto itr = myGraph.getNodes().begin(); itr != myGraph.getNodes().end(); ++itr) {
        VIS4Earth::GraphRenderer::Node node;
        node.pos = osg::Vec3(itr->second.pos.x, itr->second.pos.y, 0.f);
        node.color = colors[i];

        nodes->emplace(std::make_pair(itr->first, node));
        ++i;
    }

    for (auto itr = myGraph.getEdges().begin(); itr != myGraph.getEdges().end(); ++itr) {
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
    addGraph("Layout_restrict", nodes, edges);
    // 更新图渲染
    auto graphParam = getGraph("Layout_restrict");
    if (graphParam) {
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

void VIS4Earth::GraphRenderer::onSizeSliderValueChanged(int value) {
    // 处理边百分比阈值变化的逻辑
    size = value * 0.01;
    // Update the label text
    ui->sizeLabel->setText(QString("分辨率: %1%").arg(value));
    // 更新图渲染
    auto graphParam = getGraph("LoadedGraph");
    if (graphParam) {
        graphParam->setLongitudeRange(
            (lonRng[0] + lonRng[1]) / 2 - (lonRng[1] - lonRng[0]) * size / 2,
            (lonRng[0] + lonRng[1]) / 2 + (lonRng[1] - lonRng[0]) * size / 2);
        graphParam->setLatitudeRange(
            (latRng[0] + latRng[1]) / 2 - (latRng[1] - latRng[0]) * size / 2,
            (latRng[0] + latRng[1]) / 2 + (latRng[1] - latRng[0]) * size / 2);
        graphParam->setHeightFromCenterRange(
            static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[0],
            static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[1]);
        graphParam->setNodeGeometrySize(size * .02f *
                                        static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
        graphParam->update();
    }
    // 初始化 UI
    QLabel *coordRangeLabel = ui->labelCurrentCoordRange; // 假设使用 ui 指针来访问 UI 元素
    QString text = QString("当前坐标范围: 左: %1, 右: %2, 上: %3, 下: %4")
                       .arg(coordRange.minX)
                       .arg(coordRange.maxX)
                       .arg(coordRange.maxY)
                       .arg(coordRange.minY);
    coordRangeLabel->setText(text);
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
    if (graphParam) {
        graphParam->setLevelGraph(10 - value);
        graphParam->update();
    }
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
                                                                   const osg::Vec4 &color) {

    auto vec3ToSphere = [&](const osg::Vec3 &v3) -> osg::Vec3 {
        // v3.x() 是纬度，v3.y() 是经度
        float lat = osg::DegreesToRadians(v3.x()); // 纬度转换为弧度
        float lon = osg::DegreesToRadians(v3.y()); // 经度转换为弧度

        float h = 6371000.0f / 2 + v3.z(); // 固定为地球半径，单位为米

        osg::Vec3 ret;
        ret.z() = h * sinf(lat); // 根据纬度计算 Z 坐标

        h = h * cosf(lat); // 根据纬度调整水平投影的半径

        ret.y() = h * sinf(lon); // 根据经度计算 Y 坐标
        ret.x() = h * cosf(lon); // 根据经度计算 X 坐标

        return ret;
    };

    auto sphereToVec3 = [&](const osg::Vec3 &sphere) -> osg::Vec3 {
        // 固定的地球半径
        float earthRadius = 6371000.0f / 2;

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
    const int numSubdivisions = 5; // 细分数量
    osg::Vec3Array *lineVertices = new osg::Vec3Array;
    osg::Vec4Array *lineColors = new osg::Vec4Array;

    std::cout << "length: " << length << std::endl;
    osg::Vec3 sstart = vec3ToSphere(start);
    osg::Vec3 send = vec3ToSphere(end);
    osg::Vec3 newStart = sstart - (send - sstart) * 0.9;
    newStart = sphereToVec3(newStart);
    // 插值和渐变颜色处理
    for (int i = 0; i <= numSubdivisions; ++i) {
        float t = static_cast<float>(i) / numSubdivisions;
        osg::Vec3 interpolatedPos = newStart * (1.0f - t) + end * t;

        // 计算一个缩放因子，用来调整z值
        float scaleFactor = 1.0f - (2.0f * t - 1.0f) * (2.0f * t - 1.0f); // (1 - (2t - 1)^2)
        float zOffset = 600 * std::pow(length, 2); // 你可以调整这个值来控制下沉的幅度

        // 修改 z 值
        interpolatedPos.z() += scaleFactor * zOffset;
        osg::Vec3 spherePos = vec3ToSphere(interpolatedPos);

        // 添加顶点和颜色
        lineVertices->push_back(spherePos);
    }

    // 创建箭头的几何体
    osg::Vec3 arrowHeadBase = end - direction * 1.2f; // 箭头头部基点
    osg::Vec3 left = osg::Vec3(-direction.y(), direction.x(), 0.0f) * 0.8f;
    osg::Vec3 right = osg::Vec3(direction.y(), -direction.x(), 0.0f) * 0.8f;

    osg::Vec3Array *arrowVertices = new osg::Vec3Array;
    osg::Vec4Array *arrowColors = new osg::Vec4Array;

    // 定义箭头三角形的三个顶点
    arrowVertices->push_back(vec3ToSphere(end));                  // 箭头顶点
    arrowVertices->push_back(vec3ToSphere(arrowHeadBase + left)); // 左边
    arrowVertices->push_back(
        vec3ToSphere(arrowHeadBase + right)); // 右边
                                              // 初始化颜色数组，alpha值为0（完全透明）
    for (int i = 0; i <= 2; ++i) {
        osg::Vec4 initialColor = color;
        initialColor.a() = 0.4f; // 开始时完全透明
        arrowColors->push_back(initialColor);
    }
    // arrowColors->push_back(color);
    // arrowColors->push_back(color);
    // arrowColors->push_back(color);

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
                } else if (t >= 0.9f) {
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
    for (int i = 0; i <= numSubdivisions; ++i) {
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

void VIS4Earth::GraphRenderer::PerGraphParam::startArrowAnimation() {
    arrowFlowEnabled = !arrowFlowEnabled; // 切换箭头流动效果的启停状态

    if (arrowFlowEnabled) {
        std::cout << "Arrow flow enabled" << std::endl;
        // 开始箭头流动效果
        for (auto &edge : *edges) {
            if (!edge.visible)
                continue;                                                 // 只处理可见边
            osg::Vec4 color = osg::Vec4(nodes->at(edge.from).color, 1.f); // 边的颜色
            osg::Vec3 startPos = edge.subDivs.front();
            osg::Vec3 endPos = edge.subDivs.back();
            osg::Vec3 offset(0.0f, 0.0f, -220.6f); // 定义垂直方向的偏移量
            createArrowAnimation((startPos), (endPos), color);
        }
    } else {
        std::cout << "Arrow flow disabled" << std::endl;
        // 停止箭头流动效果
        // 可以实现清除箭头效果的逻辑，例如清除相应的节点或设置动画停止等
        grp->removeChildren(0, grp->getNumChildren());
        update(); // 重新绘制图形，移除箭头效果
    }
}

class HighlightFlowCallback : public osg::NodeCallback {
  public:
    HighlightFlowCallback(osg::Geometry *geom, float speed, TimeController *timeController,
                          const std::vector<std::pair<int, int>> &edgeRanges)
        : _geom(geom), _speed(speed), _timeController(timeController), _edgeRanges(edgeRanges) {
        // 保存原始颜色数组
        _originalColors =
            new osg::Vec4Array(*dynamic_cast<osg::Vec4Array *>(geom->getColorArray()));
    }

    virtual void operator()(osg::Node *node, osg::NodeVisitor *nv) override {
        float t = _timeController->getTime() * _speed;

        osg::Vec4Array *colors = dynamic_cast<osg::Vec4Array *>(_geom->getColorArray());
        osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(_geom->getVertexArray());
        if (colors && vertices) {
            for (const auto &range : _edgeRanges) {
                int startIdx = range.first;
                int endIdx = range.second;

                // 计算高光的位置
                float highlightPos = fmod(t, 1.0f) * (endIdx - startIdx) + startIdx;

                for (int i = startIdx; i < endIdx; ++i) {
                    // 计算高光与当前顶点的距离
                    float dist = fabs(static_cast<float>(i) - highlightPos);
                    // 控制高光范围，使其更加集中
                    float intensity =
                        std::max(0.0f, 1.0f - dist / 2.0f); // 调整 2.0f 以改变高光的范围

                    osg::Vec4 &color = (*colors)[i];
                    color = osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) * intensity +
                            (*_originalColors)[i] * (1.0f - intensity);
                }
            }

            // 标记几何体更新
            _geom->dirtyDisplayList();
            _geom->dirtyBound();
        }

        traverse(node, nv);
    }

  private:
    osg::ref_ptr<osg::Geometry> _geom;
    osg::ref_ptr<osg::Vec4Array> _originalColors;
    float _speed;
    osg::ref_ptr<TimeController> _timeController;
    std::vector<std::pair<int, int>> _edgeRanges;
};

void VIS4Earth::GraphRenderer::PerGraphParam::startHighlightAnimation() {
    if (isAnimating) {
        // 停止动画
        if (lineGeode) {
            lineGeode->setUpdateCallback(nullptr);
        }
        isAnimating = false;
        update(); // 重新绘制图形
    } else {
        // 开始动画
        if (lineGeode && lineGeometry) {
            // 禁用光照
            auto arrowStates = lineGeometry->getOrCreateStateSet();
            arrowStates->setMode(GL_BLEND, osg::StateAttribute::ON);
            arrowStates->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            // 获取顶点数组
            osg::Vec3Array *vertices =
                dynamic_cast<osg::Vec3Array *>(lineGeometry->getVertexArray());
            if (!vertices) {
                std::cout << "No vertices found!" << std::endl;
                return;
            }

            std::vector<std::pair<int, int>> edgeRanges;
            int currentIndex = 0;

            for (auto &edge : *edges) {
                if (!edge.visible)
                    continue;

                int numVerts = (edge.subDivs.size()) * 10; // 每个细分段有6个顶点
                edgeRanges.push_back(std::make_pair(currentIndex, currentIndex + numVerts));
                currentIndex += numVerts;
            }

            osg::ref_ptr<TimeController> newTimeController = new TimeController();
            lineGeode->setUpdateCallback(new HighlightFlowCallback(
                lineGeometry, 0.25f, newTimeController.get(), edgeRanges));
        }
        isAnimating = true;
    }
}

class ColorFlowCallback : public osg::NodeCallback {
  public:
    ColorFlowCallback(osg::Geometry *geom, float speed)
        : _geom(geom), _speed(speed), _startTime(std::chrono::high_resolution_clock::now()) {}

    virtual void operator()(osg::Node *node, osg::NodeVisitor *nv) {
        if (nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR) {
            // 计算动画时间
            auto now = std::chrono::high_resolution_clock::now();
            double elapsedTime = std::chrono::duration<double>(now - _startTime).count();
            float t = static_cast<float>(elapsedTime * _speed);

            // 计算渐变因子，确保在前几秒钟内逐渐改变颜色
            float fadeFactor = std::min(t / 2, 1.0f); // 0 -> 1 over time

            // 更新颜色
            osg::Vec4Array *colors = dynamic_cast<osg::Vec4Array *>(_geom->getColorArray());
            if (colors) {
                for (size_t i = 0; i < colors->size(); ++i) {
                    osg::Vec4 &color = (*colors)[i];
                    float offset = static_cast<float>(i) * 0.1f; // 基于顶点索引的偏移

                    // 基于时间 t 和 offset 计算色相值（Hue）
                    float hue =
                        fmod((t + offset) * 60.0f, 360.0f); // 60度变化对应红-黄-绿-蓝-紫-红的循环
                    osg::Vec4 targetColor = hslToRgb(hue, 0.5f, 0.5f); // 使用 HSL 转 RGB

                    // 使用 fadeFactor 实现渐变效果
                    color = color * (1.0f - fadeFactor) + targetColor * fadeFactor;
                }
                _geom->dirtyDisplayList(); // 确保更新渲染
                _geom->dirtyBound();
            }
        }

        traverse(node, nv); // 继续遍历
    }

  private:
    // HSL 转 RGB 的辅助函数
    osg::Vec4 hslToRgb(float h, float s, float l) {
        float c = (1.0f - fabs(2.0f * l - 1.0f)) * s;
        float x = c * (1.0f - fabs(fmod(h / 60.0f, 2) - 1.0f));
        float m = l - c / 2.0f;

        float r, g, b;
        if (h >= 0 && h < 60) {
            r = c;
            g = x;
            b = 0;
        } else if (h >= 60 && h < 120) {
            r = x;
            g = c;
            b = 0;
        } else if (h >= 120 && h < 180) {
            r = 0;
            g = c;
            b = x;
        } else if (h >= 180 && h < 240) {
            r = 0;
            g = x;
            b = c;
        } else if (h >= 240 && h < 300) {
            r = x;
            g = 0;
            b = c;
        } else {
            r = c;
            g = 0;
            b = x;
        }

        return osg::Vec4(r + m, g + m, b + m, 1.0f);
    }

    osg::ref_ptr<osg::Geometry> _geom;
    float _speed;
    std::chrono::high_resolution_clock::time_point _startTime;
};
void VIS4Earth::GraphRenderer::PerGraphParam::startTextureAnimation() {

    if (lineGeode && lineGeometry) {
        if (isAnimating) {
            // 当前正在动画中，结束动画
            lineGeode->setUpdateCallback(nullptr);
            isAnimating = false;
            update(); // 重新绘制图形
        } else {
            // 当前没有动画，开始动画
            lineGeode->setUpdateCallback(new ColorFlowCallback(lineGeometry, 1.f));
            isAnimating = true;
        }
    }
}

class StarFlowCallback : public osg::NodeCallback {
  public:
    StarFlowCallback(osg::Geometry *geom, float speed, TimeController *timeController,
                     const std::vector<std::pair<int, int>> &edgeRanges)
        : _geom(geom), _speed(speed), _timeController(timeController), _edgeRanges(edgeRanges) {
        // 保存原始颜色数组
        _originalColors =
            new osg::Vec4Array(*dynamic_cast<osg::Vec4Array *>(geom->getColorArray()));
    }

    virtual void operator()(osg::Node *node, osg::NodeVisitor *nv) override {
        float t = _timeController->getTime() * _speed;

        osg::Vec4Array *colors = dynamic_cast<osg::Vec4Array *>(_geom->getColorArray());
        osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(_geom->getVertexArray());
        if (colors && vertices) {
            for (const auto &range : _edgeRanges) {
                int startIdx = range.first;
                int endIdx = range.second;

                // 计算高光的位置
                float highlightPos = fmod(t, 1.0f) * (endIdx + 12.0f - startIdx) + startIdx;
                // 拖尾长度的控制参数
                float tailLengthFactor = 6.0f; // 拖尾长度控制
                for (int i = startIdx; i < endIdx; ++i) {
                    // 计算高光与当前顶点的距离
                    float dist = static_cast<float>(i) - highlightPos;
                    // 控制高光范围和拖尾效果
                    float intensity = std::max(0.0f, 1.0f - fabs(dist) / tailLengthFactor);

                    osg::Vec4 &color = (*colors)[i];

                    if (dist == 0) {
                        // 前端高亮白色部分
                        color = osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
                    } else if (dist < 0) {
                        // 拖尾部分：从白色渐变到原始颜色
                        color = osg::Vec4(
                            1.0f * intensity + (*_originalColors)[i].r() * (1.0f - intensity),
                            1.0f * intensity + (*_originalColors)[i].g() * (1.0f - intensity),
                            1.0f * intensity + (*_originalColors)[i].b() * (1.0f - intensity),
                            (*_originalColors)[i].a());
                    } else {
                        color = (*_originalColors)[i];
                    }
                }
                // 处理 endIdx 之后的拖尾效果
                for (int i = endIdx + 1; i < endIdx + tailLengthFactor && i < colors->size(); ++i) {
                    // 计算超出 endIdx 的部分
                    float dist = static_cast<float>(i) - highlightPos;
                    float intensity =
                        std::max(0.0f, 1.0f - (dist + (i - endIdx)) / tailLengthFactor);

                    osg::Vec4 &color = (*colors)[i];
                    color = osg::Vec4((*_originalColors)[endIdx - 1].r() * (1.0f - intensity),
                                      (*_originalColors)[endIdx - 1].g() * (1.0f - intensity),
                                      (*_originalColors)[endIdx - 1].b() * (1.0f - intensity),
                                      (*_originalColors)[endIdx - 1].a() * intensity);
                }
            }

            // 标记几何体更新
            _geom->dirtyDisplayList();
            _geom->dirtyBound();
        }

        traverse(node, nv);
    }

  private:
    osg::ref_ptr<osg::Geometry> _geom;
    osg::ref_ptr<osg::Vec4Array> _originalColors;
    float _speed;
    osg::ref_ptr<TimeController> _timeController;
    std::vector<std::pair<int, int>> _edgeRanges;
};
void VIS4Earth::GraphRenderer::PerGraphParam::startStarAnimation() {

    if (isAnimating) {
        // 停止动画
        if (lineGeode) {
            lineGeode->setUpdateCallback(nullptr);
        }
        isAnimating = false;
        update(); // 重新绘制图形
    } else {
        // 开始动画
        if (lineGeode && lineGeometry) {
            auto arrowStates = lineGeometry->getOrCreateStateSet();
            arrowStates->setMode(GL_BLEND, osg::StateAttribute::ON);
            arrowStates->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            // 获取顶点数组
            osg::Vec3Array *vertices =
                dynamic_cast<osg::Vec3Array *>(lineGeometry->getVertexArray());
            if (!vertices) {
                std::cout << "No vertices found!" << std::endl;
                return;
            }

            std::vector<std::pair<int, int>> edgeRanges;
            int currentIndex = 0;

            for (auto &edge : *edges) {
                if (!edge.visible)
                    continue;

                int numVerts = (edge.subDivs.size()) * 10;
                edgeRanges.push_back(std::make_pair(currentIndex, currentIndex + numVerts - 1));
                currentIndex += numVerts;
            }

            osg::ref_ptr<TimeController> newTimeController = new TimeController();
            lineGeode->setUpdateCallback(
                new StarFlowCallback(lineGeometry, 0.25f, newTimeController.get(), edgeRanges));
        }
        isAnimating = true;
    }
}
void VIS4Earth::GraphRenderer::PerGraphParam::update() {
    grp->removeChildren(0, grp->getNumChildren());

    auto vec3ToSphere = [&](const osg::Vec3 &v3) -> osg::Vec3 {
        // v3.x() 是纬度，v3.y() 是经度
        float lat = osg::DegreesToRadians(v3.x()); // 纬度转换为弧度
        float lon = osg::DegreesToRadians(v3.y()); // 经度转换为弧度

        float h = 6371000.0f; // 固定为地球半径，单位为米

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
            continue;                                        // 只处理可见节点
        osg::Vec4 color = osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f); // Set color to blue
        // osg::Vec4 color = osg::Vec4(itr->second.color, 1.f);
        if (!restrictionOFF) {
            if (itr->second.pos.x() >= restriction.leftBound &&
                itr->second.pos.x() <= restriction.rightBound &&
                itr->second.pos.y() >= restriction.bottomBound &&
                itr->second.pos.y() <= restriction.upperBound) {
                color = osg::Vec4(1.0f, 1.0f, 1.0f, 0.5f); // 设置边框内的点为半透明白色
            }
        }
        auto p = itr->second.pos;

        p = vec3ToSphere(p);
        auto sphere = new osg::ShapeDrawable(new osg::Sphere(p, .25f * nodeGeomSize), tessl);
        osg::ref_ptr<osg::Vec3Array> centerData = new osg::Vec3Array;
        centerData->push_back(itr->second.pos);
        sphere->setUserData(centerData);

        sphere->setColor(color);

        auto states = grp->getOrCreateStateSet();
        auto matr = new osg::Material;
        matr->setColorMode(osg::Material::DIFFUSE);
        states->setAttributeAndModes(matr, osg::StateAttribute::ON);
        states->setMode(GL_LIGHTING, osg::StateAttribute::ON);
        // osg::setNotifyLevel(osg::DEBUG_INFO);

        grp->addChild(sphere);

        osg::ref_ptr<osgText::Text> text = new osgText::Text;
        text->setText(itr->first);
        text->setFont("Fonts/simhei.ttf"); // 设置字体
        text->setAxisAlignment(osgText::Text::SCREEN);
        if (textSize) {
            text->setCharacterSize(textSize); // 设置字体大小
        } else {
            text->setCharacterSize(nodeGeomSize);
        }

        // TODO: 加入文字避让
        text->setPosition(
            p + osg::Vec3(0.5f * nodeGeomSize, 1.0f * nodeGeomSize,
                          0.25f * nodeGeomSize)); // 设置文字位置为点的位置稍微向上移动一些
                                                  // 设置文字内容为点的ID
        text->setColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)); // 设置文字颜色为白色

        osg::ref_ptr<osg::Geode> textGeode = new osg::Geode;
        textGeode->addDrawable(text.get());
        textNodes.push_back(text);
        grp->addChild(textGeode.get());

        osgNodes.emplace(std::make_pair(itr->first, sphere));
    }
    adjustTextPosition(textNodes, nodeGeomSize);

    auto segVerts = new osg::Vec3Array;
    auto segCols = new osg::Vec4Array;

    for (auto &edge : *edges) {
        if (!edge.visible)
            continue; // 只处理可见边

        osg::Vec4 prevColor = osg::Vec4(nodes->at(edge.from).color, 1.f);
        auto dCol = osg::Vec4(nodes->at(edge.to).color, 1.f) - prevColor;
        dCol /= (edge.subDivs.size() == 1 ? 1 : edge.subDivs.size() - 1);

        osg::Vec3 prevPos = edge.subDivs.front();
        osg::Vec3 startPos = vec3ToSphere(prevPos); // 起点
        osg::Vec3 endPos = edge.subDivs.back();
        endPos = vec3ToSphere(endPos);

        // std::cout << "Edge from node " << edge.from << " to node " << edge.to << std::endl;
        // std::cout << "Start position (sphere): (" << startPos.x() << ", " << startPos.y() << ", "
        //           << startPos.z() << ")" << std::endl;
        // std::cout << "End position (sphere): (" << endPos.x() << ", " << endPos.y() << ", "
        //           << endPos.z() << ")" << std::endl;

        osg::Vec3 prevInterpolatedPos = prevPos;     // 初始插值位置
        osg::Vec4 prevInterpolatedColor = prevColor; // 初始插值颜色
        for (size_t i = 0; i < edge.subDivs.size(); ++i) {
            osg::Vec3 currentPos = edge.subDivs[i];
            osg::Vec4 currentColor = prevColor + dCol;

            // 在 prevPos 和 currentPos 之间插入 5 个细分点
            for (int j = 0; j <= 5; ++j) { // 包含起点和终点，共 6 个点
                float t = static_cast<float>(j) / 5.0f;

                osg::Vec3 interpolatedPos;
                interpolatedPos.x() = prevPos.x() * (1.0f - t) + currentPos.x() * t;
                interpolatedPos.y() = prevPos.y() * (1.0f - t) + currentPos.y() * t;
                interpolatedPos.z() = prevPos.z() * (1.0f - t) + currentPos.z() * t;

                osg::Vec4 interpolatedColor = prevColor * (1.0f - t) + currentColor * t;

                if (j > 0) { // 从第二个插值点开始创建线段
                    segVerts->push_back(vec3ToSphere(prevInterpolatedPos));
                    segCols->push_back(prevInterpolatedColor);
                    segVerts->push_back(vec3ToSphere(interpolatedPos));
                    segCols->push_back(interpolatedColor);
                }

                prevInterpolatedPos = interpolatedPos;
                prevInterpolatedColor = interpolatedColor;
            }

            prevPos = currentPos;
            prevColor = currentColor;
        }
    }
    if (!arrowFlowEnabled) {
        auto geom = new osg::Geometry;
        geom->setVertexArray(segVerts);
        geom->setColorArray(segCols);
        geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

        auto states = geom->getOrCreateStateSet();
        states->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        auto lw = new osg::LineWidth(2.f);
        states->setAttribute(lw, osg::StateAttribute::ON);

        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, segVerts->size()));

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
}

void GraphRenderer::PerGraphParam::generateHierarchicalGraphs(
    std::shared_ptr<std::map<std::string, Node>> &initialNodes,
    std::shared_ptr<std::vector<Edge>> &initialEdges) {

    int numLevels = 11; // 总共生成11层图
    std::vector<GraphLevel> mylevels(numLevels);

    // 第0层次是原始图
    mylevels[0].nodes = std::make_shared<std::map<std::string, Node>>(*initialNodes);
    mylevels[0].edges = std::make_shared<std::vector<Edge>>(*initialEdges);

    // 生成其他层次的图
    for (int level = 1; level < numLevels; ++level) {
        mylevels[level].nodes = std::make_shared<std::map<std::string, Node>>();
        mylevels[level].edges = std::make_shared<std::vector<Edge>>();

        // 对前一个层次的图执行 DBSCAN 聚类
        performClustering(mylevels[level - 1], mylevels[level], static_cast<float>(level) / 10.0f);
    }

    levels = mylevels; // 将生成的层次存储到成员变量
}

void GraphRenderer::PerGraphParam::performClustering(const GraphLevel &previousLevel,
                                                     GraphLevel &currentLevel, float threshold) {

    // 从上一个层次的节点中提取位置信息，用于 DBSCAN 聚类
    std::vector<osg::Vec3> positions;
    std::vector<std::string> nodeIds;
    for (const auto &nodePair : *previousLevel.nodes) {
        positions.push_back(nodePair.second.pos);
        nodeIds.push_back(nodePair.first);
    }

    // 使用 DBSCAN 对节点进行聚类
    std::vector<int> clusterLabels = DBSCAN(positions, 5, /*minPts*/ 2);

    // 将节点根据簇分类
    std::map<int, std::vector<std::string>> clusters; // 簇ID -> 节点ID列表
    for (size_t i = 0; i < clusterLabels.size(); ++i) {
        int clusterId = clusterLabels[i];
        clusters[clusterId].push_back(nodeIds[i]);
    }

    std::map<std::string, std::vector<std::string>> nodeMapping; // 原始节点到代表节点的映射
    std::map<Edge, std::vector<Edge>> edgeMapping;               // 边映射
    std::set<std::string> processedNodes;
    std::vector<std::string> remainingNodes; // 未处理的节点列表

    // 遍历每个簇，选择代表节点并合并
    for (const auto &clusterPair : clusters) {
        const std::vector<std::string> &nodesInCluster = clusterPair.second;

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

    // 2. 为未直接连接但联通的节点添加新边
    for (const auto &nodePair1 : *currentLevel.nodes) {
        for (const auto &nodePair2 : *currentLevel.nodes) {
            if (nodePair1.first == nodePair2.first)
                continue; // 跳过自己与自己的边

            std::string from = std::min(nodePair1.first, nodePair2.first);
            std::string to = std::max(nodePair1.first, nodePair2.first);

            // 如果这条边已经处理过，则跳过
            if (processedEdges.count({from, to}) > 0)
                continue;

            // 检查这两个节点在上一层是否通过某种方式连接
            bool isConnected = false;
            for (const std::string &originalNode1 : nodeMapping.at(nodePair1.first)) {
                for (const std::string &originalNode2 : nodeMapping.at(nodePair2.first)) {
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

            // 如果两个节点在上一级中连接，则在当前层中添加一条直接的边
            if (isConnected) {
                Edge newEdge;
                newEdge.from = from;
                newEdge.to = to;
                // 设置细分点
                auto it = nodes->find(from);
                newEdge.subDivs.emplace_back(it->second.pos);
                it = nodes->find(to);
                newEdge.subDivs.emplace_back(it->second.pos);

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

// 无固定位置的图的聚类