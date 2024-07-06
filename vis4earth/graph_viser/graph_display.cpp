#include "graph_display.h"

#include <ui_graph_layout.h>

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

    connect(ui->checkBoxRegionRestriction, &QCheckBox::toggled, this,
            &GraphRenderer::setRegionRestriction);
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
            node.id = itr->second.id;

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
void VIS4Earth::GraphRenderer::setRegionRestriction(bool enabled) { restrictionOn = enabled; }

void VIS4Earth::GraphRenderer::setMinX(double value) { myRestriction.leftBound = value; }

void VIS4Earth::GraphRenderer::setMaxX(double value) { myRestriction.rightBound = value; }

void VIS4Earth::GraphRenderer::setMinY(double value) { myRestriction.bottomBound = value; }

void VIS4Earth::GraphRenderer::setMaxY(double value) { myRestriction.upperBound = value; }

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
        graphParam->setNodeGeometrySize(.02f * static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
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

    for (auto itr = nodes->begin(); itr != nodes->end(); ++itr) {
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
        sphere->setColor(color);

        auto states = grp->getOrCreateStateSet();
        auto matr = new osg::Material;
        matr->setColorMode(osg::Material::DIFFUSE);
        states->setAttributeAndModes(matr, osg::StateAttribute::ON);
        states->setMode(GL_LIGHTING, osg::StateAttribute::ON);

        // TODO:显示文字
        grp->addChild(sphere);
        auto text = new osgText::Text;
        // text->setFont("fonts/arial.ttf"); // 设置字体
        text->setCharacterSize(2.5f); // 设置字体大小
        text->setPosition(p +
                          osg::Vec3(0.0f, 0.0f, 1.5f)); // 设置文字位置为点的位置稍微向上移动一些
        text->setText(itr->second.id);                  // 设置文字内容为点的ID
        text->setColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)); // 设置文字颜色为白色

        auto textGeode = new osg::Geode;
        textGeode->addDrawable(text);

        grp->addChild(textGeode);

        osgNodes.emplace(std::make_pair(itr->first, sphere));
    }

    auto segVerts = new osg::Vec3Array;
    auto segCols = new osg::Vec4Array;

    for (auto &edge : *edges) {
        osg::Vec4 prevColor = osg::Vec4(nodes->at(edge.from).color, 1.f);
        auto dCol = osg::Vec4(nodes->at(edge.to).color, 1.f) - prevColor;
        dCol /= (edge.subDivs.size() == 1 ? 1 : edge.subDivs.size() - 1);

        osg::Vec3 prevPos = edge.subDivs.front() - minPos;
        prevPos.x() = dltPos.x() == 0.f ? prevPos.x() : prevPos.x() / dltPos.x();
        prevPos.y() = dltPos.y() == 0.f ? prevPos.y() : prevPos.y() / dltPos.y();
        prevPos.z() = dltPos.z() == 0.f ? prevPos.z() : prevPos.z() / dltPos.z();
        prevPos = vec3ToSphere(prevPos);

        for (size_t i = 1; i < edge.subDivs.size(); ++i) {
            segVerts->push_back(prevPos);
            segCols->push_back(prevColor);

            auto p = edge.subDivs[i] - minPos;
            p.x() = dltPos.x() == 0.f ? p.x() : p.x() / dltPos.x();
            p.y() = dltPos.y() == 0.f ? p.y() : p.y() / dltPos.y();
            p.z() = dltPos.z() == 0.f ? p.z() : p.z() / dltPos.z();
            p = vec3ToSphere(p);

            auto color = prevColor + dCol;

            segVerts->push_back(p);
            segCols->push_back(color);

            prevPos = p;
            prevColor = color;
        }
    }

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

    grp->addChild(geode);
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
