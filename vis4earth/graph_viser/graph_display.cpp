#include "graph_display.h"

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

    connect(ui->loadPointsButton, &QPushButton::clicked, this, &GraphRenderer::loadPointsCSV);
    connect(ui->loadEdgesButton, &QPushButton::clicked, this, &GraphRenderer::loadEdgesCSV);
    connect(ui->loadAndDrawGraphButton, &QPushButton::clicked, this,
            &GraphRenderer::loadAndDrawGraph);

    connect(ui->showGraphLayoutButton, &QPushButton::clicked, this, &GraphRenderer::showGraph);
    connect(ui->showEdgeBundlingButton, &QPushButton::clicked, this, &GraphRenderer::showBundling);

    // Connect the slider's valueChanged signal to the slot function
    connect(ui->sizeSlider, &QSlider::valueChanged, this, &GraphRenderer::onSizeSliderValueChanged);
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

void VIS4Earth::GraphRenderer::loadAndDrawGraph() {

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
        }
        myGraph = graph;
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

void VIS4Earth::GraphRenderer::PerGraphParam::createArrowAnimation(const osg::Vec3 &start,
                                                                   const osg::Vec3 &end,
                                                                   const osg::Vec4 &color) {

    auto vec3ToSphere = [&](const osg::Vec3 &v3) -> osg::Vec3 {
        float dlt = maxLongitude - minLongitude;
        float x = volStartFromLonZero == 0 ? v3.x() : v3.x() < .5f ? v3.x() + .5f : v3.x() - .5f;
        float lon = minLongitude + x * dlt;
        dlt = maxLatitude - minLatitude;
        float lat = minLatitude + v3.y() * dlt;
        dlt = maxHeight - minHeight;
        float h = minHeight + v3.z() * dlt;

        osg::Vec3 ret;
        ret.z() = h * sinf(lat);
        h = h * cosf(lat);
        ret.y() = h * sinf(lon);
        ret.x() = h * cosf(lon);

        return ret;
    };
    std::cout << "Creating arrow animation from " << start.x() << " to " << end.x() << std::endl;

    // 计算箭头方向和长度
    osg::Vec3 direction = end - start;
    float length = direction.length();
    direction.normalize();

    // 计算插值点
    const int numSubdivisions = 5; // 细分数量
    osg::Vec3Array *lineVertices = new osg::Vec3Array;
    osg::Vec4Array *lineColors = new osg::Vec4Array;
    for (int i = 0; i <= numSubdivisions; ++i) {
        float t = static_cast<float>(i) / numSubdivisions;
        osg::Vec3 interpolatedPos = start * (1.0f - t) + end * t;

        lineVertices->push_back(vec3ToSphere(interpolatedPos));
        lineColors->push_back(color);
    }

    // 绘制线段
    auto lineGeom = new osg::Geometry;
    lineGeom->setVertexArray(lineVertices);
    lineGeom->setColorArray(lineColors, osg::Array::BIND_PER_VERTEX);
    lineGeom->addPrimitiveSet(
        new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, lineVertices->size()));

    auto lineGeode = new osg::Geode;
    lineGeode->addDrawable(lineGeom);

    // 禁用光照
    auto lineStates = lineGeom->getOrCreateStateSet();
    lineStates->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    auto lw = new osg::LineWidth(2.f);
    lineStates->setAttribute(lw, osg::StateAttribute::ON);

    grp->addChild(lineGeode);

    // 创建箭头的几何体
    osg::Vec3 arrowHeadBase = end - direction * 0.05f; // 箭头头部基点
    osg::Vec3 left = osg::Vec3(-direction.y(), direction.x(), 0.0f) * 0.03f * length;
    osg::Vec3 right = osg::Vec3(direction.y(), -direction.x(), 0.0f) * 0.03f * length;

    osg::Vec3Array *arrowVertices = new osg::Vec3Array;
    osg::Vec4Array *arrowColors = new osg::Vec4Array;

    // 定义箭头三角形的三个顶点
    arrowVertices->push_back(vec3ToSphere(end));                   // 箭头顶点
    arrowVertices->push_back(vec3ToSphere(arrowHeadBase + left));  // 左边
    arrowVertices->push_back(vec3ToSphere(arrowHeadBase + right)); // 右边

    arrowColors->push_back(color);
    arrowColors->push_back(color);
    arrowColors->push_back(color);

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
    auto lwArrow = new osg::LineWidth(2.f);
    arrowStates->setAttribute(lwArrow, osg::StateAttribute::ON);

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
    transform->addChild(lineGeode);
    transform->addChild(arrowGeode);
    transform->setUpdateCallback(animationCallback);

    grp->addChild(transform);
}

void VIS4Earth::GraphRenderer::PerGraphParam::startArrowAnimation() {
    arrowFlowEnabled = !arrowFlowEnabled; // 切换箭头流动效果的启停状态
    osg::Vec3 minPos(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                     std::numeric_limits<float>::max());
    osg::Vec3 maxPos(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                     std::numeric_limits<float>::lowest());

    auto updateMinMax = [&](const osg::Vec3 &p) {
        minPos.x() = std::min(minPos.x(), p.x());
        minPos.y() = std::min(minPos.y(), p.y());
        minPos.z() = std::min(minPos.z(), p.z());

        maxPos.x() = std::max(maxPos.x(), p.x());
        maxPos.y() = std::max(maxPos.y(), p.y());
        maxPos.z() = std::max(maxPos.z(), p.z());
    };

    for (auto &node : *nodes) {
        updateMinMax(node.second.pos);
    }
    auto dltPos = maxPos - minPos;
    if (arrowFlowEnabled) {
        std::cout << "Arrow flow enabled" << std::endl;
        // 开始箭头流动效果
        for (auto &edge : *edges) {
            if (!edge.visible)
                continue; // 只处理可见边

            osg::Vec4 color = osg::Vec4(nodes->at(edge.from).color, 1.f); // 边的颜色

            osg::Vec3 startPos = nodes->at(edge.from).pos - minPos;
            osg::Vec3 endPos = nodes->at(edge.to).pos - minPos;

            startPos.x() = dltPos.x() == 0.f ? startPos.x() : startPos.x() / dltPos.x();
            startPos.y() = dltPos.y() == 0.f ? startPos.y() : startPos.y() / dltPos.y();
            startPos.z() = dltPos.z() == 0.f ? startPos.z() : startPos.z() / dltPos.z();

            endPos.x() = dltPos.x() == 0.f ? endPos.x() : endPos.x() / dltPos.x();
            endPos.y() = dltPos.y() == 0.f ? endPos.y() : endPos.y() / dltPos.y();
            endPos.z() = dltPos.z() == 0.f ? endPos.z() : endPos.z() / dltPos.z();

            osg::Vec3 offset(0.0f, 0.0f, -64.6f); // 定义垂直方向的偏移量

            std::cout << "Edge from node " << edge.from << " to node " << edge.to << std::endl;
            std::cout << "Start position: (" << startPos.x() << ", " << startPos.y() << ", "
                      << startPos.z() << ")" << std::endl;
            std::cout << "End position: (" << endPos.x() << ", " << endPos.y() << ", " << endPos.z()
                      << ")" << std::endl;

            createArrowAnimation(startPos + offset, endPos + offset, color);
        }
    } else {
        std::cout << "Arrow flow disabled" << std::endl;
        // 停止箭头流动效果
        // 可以实现清除箭头效果的逻辑，例如清除相应的节点或设置动画停止等
        grp->removeChildren(0, grp->getNumChildren());
        update(); // 重新绘制图形，移除箭头效果
    }
}

void VIS4Earth::GraphRenderer::PerGraphParam::createHighlightAnimation(
    const osg::Vec3 &start, const osg::Vec3 &end, const osg::Vec4 &baseColor,
    const osg::Vec4 &highlightColor) {

    auto vec3ToSphere = [&](const osg::Vec3 &v3) -> osg::Vec3 {
        float dlt = maxLongitude - minLongitude;
        float x = volStartFromLonZero == 0 ? v3.x() : v3.x() < .5f ? v3.x() + .5f : v3.x() - .5f;
        float lon = minLongitude + x * dlt;
        dlt = maxLatitude - minLatitude;
        float lat = minLatitude + v3.y() * dlt;
        dlt = maxHeight - minHeight;
        float h = minHeight + v3.z() * dlt;

        osg::Vec3 ret;
        ret.z() = h * sinf(lat);
        h = h * cosf(lat);
        ret.y() = h * sinf(lon);
        ret.x() = h * cosf(lon);

        return ret;
    };
    std::cout << "Creating highlight animation from " << start.x() << " to " << end.x()
              << std::endl;

    // 计算路径长度和方向
    osg::Vec3 direction = end - start;
    float length = direction.length();

    // 计算插值点
    const int numSubdivisions = 5; // 细分数量
    osg::Vec3Array *lineVertices = new osg::Vec3Array;
    osg::Vec4Array *lineColors = new osg::Vec4Array;

    osg::Vec3 offset(0.0f, 0.0f, -64.58f); // 定义垂直方向的偏移量
    for (int i = 0; i <= numSubdivisions; ++i) {
        float t = static_cast<float>(i) / numSubdivisions;
        osg::Vec3 interpolatedPos = start * (1.0f - t) + (end)*t;
        lineVertices->push_back(vec3ToSphere(interpolatedPos + offset));
        osg::Vec4 interpolatedColor = osg::Vec4(1.0, 1.0, 1.0, 1.0) * (1.0f - t) + baseColor * t;
        lineColors->push_back(interpolatedColor);
    }
    // 绘制短线条
    auto lineGeom = new osg::Geometry;
    lineGeom->setVertexArray(lineVertices);
    lineGeom->setColorArray(lineColors, osg::Array::BIND_PER_VERTEX);
    lineGeom->addPrimitiveSet(
        new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, lineVertices->size()));

    auto lineGeode = new osg::Geode;
    lineGeode->addDrawable(lineGeom);

    // 禁用光照
    auto lineStates = lineGeom->getOrCreateStateSet();
    lineStates->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    auto lw = new osg::LineWidth(2.f);
    lineStates->setAttribute(lw, osg::StateAttribute::ON);

    // 创建动画路径
    osg::ref_ptr<osg::AnimationPath> animationPath = new osg::AnimationPath();
    animationPath->setLoopMode(osg::AnimationPath::LOOP);

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
    transform->addChild(lineGeode);
    transform->setUpdateCallback(animationCallback);

    grp->addChild(transform);
}
class HighlightFlowCallback : public osg::NodeCallback {
  public:
    HighlightFlowCallback(osg::Geometry *geom, float speed)
        : _geom(geom), _speed(speed), _startTime(std::chrono::high_resolution_clock::now()) {
        // 备份原始颜色
        _originalColors =
            new osg::Vec4Array(*dynamic_cast<osg::Vec4Array *>(geom->getColorArray()));
    }

    virtual void operator()(osg::Node *node, osg::NodeVisitor *nv) {
        auto now = std::chrono::high_resolution_clock::now();
        double elapsedTime = std::chrono::duration<double>(now - _startTime).count();
        float t = static_cast<float>(elapsedTime * _speed);

        osg::Vec4Array *colors = dynamic_cast<osg::Vec4Array *>(_geom->getColorArray());
        osg::Vec3Array *vertices = dynamic_cast<osg::Vec3Array *>(_geom->getVertexArray());
        if (colors && vertices) {
            float highlightPos = fmod(t, 1.0f) * vertices->size(); // 高光点位置

            for (size_t i = 0; i < colors->size(); ++i) {
                float dist = fabs(static_cast<float>(i) - highlightPos);
                dist = std::min(dist, static_cast<float>(vertices->size()) - dist); // 环绕效果
                float intensity = std::max(0.0f, 1.0f - dist / 10.0f); // 距离越近，高光越强

                osg::Vec4 &color = (*colors)[i];
                color = osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) * intensity +
                        (*_originalColors)[i] * (1.0f - intensity);
            }
            _geom->dirtyDisplayList(); // 确保更新渲染
            _geom->dirtyBound();

            // 如果高光点到达终点，重置时间
            if (highlightPos >= vertices->size() - 1) {
                _startTime = std::chrono::high_resolution_clock::now();
            }
        }

        traverse(node, nv); // 继续遍历
    }

  private:
    osg::ref_ptr<osg::Geometry> _geom;
    osg::ref_ptr<osg::Vec4Array> _originalColors;
    float _speed;
    std::chrono::high_resolution_clock::time_point _startTime;
};
void VIS4Earth::GraphRenderer::PerGraphParam::startHighlightAnimation() {
    if (lineGeode && lineGeometry) {
        if (isAnimating) {
            // 当前正在动画中，结束动画
            lineGeode->setUpdateCallback(nullptr);
            isAnimating = false;
        } else {
            // 当前没有动画，开始动画
            lineGeode->setUpdateCallback(new HighlightFlowCallback(lineGeometry, 0.01f));
            isAnimating = true;
        }
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

            // 更新颜色
            osg::Vec4Array *colors = dynamic_cast<osg::Vec4Array *>(_geom->getColorArray());
            if (colors) {
                for (size_t i = 0; i < colors->size(); ++i) {
                    osg::Vec4 &color = (*colors)[i];
                    float offset = static_cast<float>(i) * 0.1f;  // 基于顶点索引的偏移
                    color.r() = (sinf(t + offset) + 1.0f) * 0.5f; // 动态红色分量
                    color.g() = (cosf(t + offset) + 1.0f) * 0.5f; // 动态绿色分量
                    color.b() = (sinf(t + offset + 3.14f / 2) + 1.0f) * 0.5f; // 动态蓝色分量
                }
                _geom->dirtyDisplayList(); // 确保更新渲染
                _geom->dirtyBound();
            }
        }

        traverse(node, nv); // 继续遍历
    }

  private:
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
        } else {
            // 当前没有动画，开始动画
            lineGeode->setUpdateCallback(new ColorFlowCallback(lineGeometry, 1.f));
            isAnimating = true;
        }
    }
}

void VIS4Earth::GraphRenderer::PerGraphParam::update() {
    grp->removeChildren(0, grp->getNumChildren());

    osg::Vec3 minPos(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                     std::numeric_limits<float>::max());
    osg::Vec3 maxPos(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                     std::numeric_limits<float>::lowest());

    auto updateMinMax = [&](const osg::Vec3 &p) {
        minPos.x() = std::min(minPos.x(), p.x());
        minPos.y() = std::min(minPos.y(), p.y());
        minPos.z() = std::min(minPos.z(), p.z());

        maxPos.x() = std::max(maxPos.x(), p.x());
        maxPos.y() = std::max(maxPos.y(), p.y());
        maxPos.z() = std::max(maxPos.z(), p.z());
    };

    auto vec3ToSphere = [&](const osg::Vec3 &v3) -> osg::Vec3 {
        float dlt = maxLongitude - minLongitude;
        float x = volStartFromLonZero == 0 ? v3.x() : v3.x() < .5f ? v3.x() + .5f : v3.x() - .5f;
        float lon = minLongitude + x * dlt;
        dlt = maxLatitude - minLatitude;
        float lat = minLatitude + v3.y() * dlt;
        dlt = maxHeight - minHeight;
        float h = minHeight + v3.z() * dlt;

        osg::Vec3 ret;
        ret.z() = h * sinf(lat);
        h = h * cosf(lat);
        ret.y() = h * sinf(lon);
        ret.x() = h * cosf(lon);

        return ret;
    };

    for (auto &node : *nodes) {
        updateMinMax(node.second.pos);
    }

    auto dltPos = maxPos - minPos;

    auto tessl = new osg::TessellationHints;
    tessl->setDetailRatio(1.f);
    std::map<std::string, osg::ShapeDrawable *> osgNodes;
    std::vector<osg::ref_ptr<osgText::Text>> textNodes;
    for (auto itr = nodes->begin(); itr != nodes->end(); ++itr) {
        if (!itr->second.visible)
            continue;                                        // 只处理可见节点
        osg::Vec4 color = osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f); // Set color to blue
        // osg::Vec4 color = osg::Vec4(itr->second.color, 1.f);
        if (!restrictionOFF) {
            if (itr->second.pos.x() >= restriction.leftBound &&
                itr->second.pos.x() <= restriction.rightBound &&
                itr->second.pos.y() >= restriction.bottomBound &&
                itr->second.pos.y() <= restriction.upperBound) {
                color = osg::Vec4(1.0f, 1.0f, 1.0f, 0.5f); // 设置边框内的点为半透明白色
            }
        }
        auto p = itr->second.pos - minPos;
        p.x() = dltPos.x() == 0.f ? p.x() : p.x() / dltPos.x();
        p.y() = dltPos.y() == 0.f ? p.y() : p.y() / dltPos.y();
        p.z() = dltPos.z() == 0.f ? p.z() : p.z() / dltPos.z();

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
        text->setPosition(p + osg::Vec3(0.25f * nodeGeomSize, -0.25f * nodeGeomSize,
                                        0.0f)); // 设置文字位置为点的位置稍微向上移动一些
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

        osg::Vec3 prevPos = edge.subDivs.front() - minPos;
        prevPos.x() = dltPos.x() == 0.f ? prevPos.x() : prevPos.x() / dltPos.x();
        prevPos.y() = dltPos.y() == 0.f ? prevPos.y() : prevPos.y() / dltPos.y();
        prevPos.z() = dltPos.z() == 0.f ? prevPos.z() : prevPos.z() / dltPos.z();
        // prevPos = vec3ToSphere(prevPos);

        osg::Vec3 startPos = vec3ToSphere(prevPos); // 起点
        osg::Vec3 endPos = edge.subDivs.back() - minPos;
        endPos.x() = dltPos.x() == 0.f ? endPos.x() : endPos.x() / dltPos.x();
        endPos.y() = dltPos.y() == 0.f ? endPos.y() : endPos.y() / dltPos.y();
        endPos.z() = dltPos.z() == 0.f ? endPos.z() : endPos.z() / dltPos.z(); // 终点
        endPos = vec3ToSphere(endPos);

        std::cout << "Edge from node " << edge.from << " to node " << edge.to << std::endl;
        std::cout << "Start position (sphere): (" << startPos.x() << ", " << startPos.y() << ", "
                  << startPos.z() << ")" << std::endl;
        std::cout << "End position (sphere): (" << endPos.x() << ", " << endPos.y() << ", "
                  << endPos.z() << ")" << std::endl;

        osg::Vec3 prevInterpolatedPos = prevPos;     // 初始插值位置
        osg::Vec4 prevInterpolatedColor = prevColor; // 初始插值颜色
        for (size_t i = 0; i < edge.subDivs.size(); ++i) {
            osg::Vec3 currentPos = edge.subDivs[i] - minPos;
            currentPos.x() = dltPos.x() == 0.f ? currentPos.x() : currentPos.x() / dltPos.x();
            currentPos.y() = dltPos.y() == 0.f ? currentPos.y() : currentPos.y() / dltPos.y();
            currentPos.z() = dltPos.z() == 0.f ? currentPos.z() : currentPos.z() / dltPos.z();
            // currentPos = vec3ToSphere(currentPos);

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
