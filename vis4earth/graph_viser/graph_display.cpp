#include "graph_display.h"
#include "DBSCAN.h"

#include <ui_graph_layout.h>

#include "LOUVAIN.h"
#include "graph_draw.h"
#include <algorithm>
#include <osgText/Font>
#include <set>
#include <memory>

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
    connect(ui->resolutionSlider, &QSlider::valueChanged, this,
            &GraphRenderer::onResolutionSliderValueChanged);

    // 连接参数设置的信号到槽函数
    // connect(ui->spinBoxAttraction, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    //        &GraphRenderer::setAttraction);
    // connect(ui->spinBoxEdgeLength, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    //        &GraphRenderer::setEdgeLength);
    // connect(ui->spinBoxRepulsion, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    //        &GraphRenderer::setRepulsion);
    // connect(ui->spinBoxSpringK, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    //        &GraphRenderer::setSpringK);
    // connect(ui->spinBoxIteration, QOverload<int>::of(&QSpinBox::valueChanged), this,
    //        &GraphRenderer::setIteration);

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

    // 初始化LOD数据容器
    for (int i = 0; i < 4; ++i) {
        lodNodesData[i].reset();
        lodEdgesData[i].reset();
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
// 调整文字位置以避免重叠
void adjustTextPosition(std::vector<osg::ref_ptr<osgText::Text>> &texts, float nodeGeomSize,
                        osg::ref_ptr<osg::Camera> camera) {
    // 定义网格大小和网格数量
    const int GRID_SIZE = 10;
    const int GRID_COLS = 100;
    const int GRID_ROWS = 100;

    // 创建空间网格
    std::vector<std::vector<std::vector<int>>> spatialGrid(
        GRID_ROWS, std::vector<std::vector<int>>(GRID_COLS));

    // 检查相机
    if (!camera || !camera->getViewport())
        return;

    // 获取必要的矩阵
    osg::Matrix viewMatrix = camera->getViewMatrix();
    osg::Matrix projectionMatrix = camera->getProjectionMatrix();
    osg::Viewport *viewport = camera->getViewport();

    // 辅助函数：将世界坐标转换为屏幕坐标
    auto worldToScreen = [&](const osg::Vec3 &worldPos) -> osg::Vec3 {
        // 视图变换
        osg::Vec4 viewPos = osg::Vec4(worldPos, 1.0f) * viewMatrix;

        // 投影变换
        osg::Vec4 clipPos = viewPos * projectionMatrix;

        // 透视除法
        if (clipPos.w() != 0.0) {
            clipPos.x() /= clipPos.w();
            clipPos.y() /= clipPos.w();
            clipPos.z() /= clipPos.w();
        }

        // 视口变换
        return osg::Vec3((clipPos.x() * 0.5f + 0.5f) * viewport->width() + viewport->x(),
                         (clipPos.y() * 0.5f + 0.5f) * viewport->height() + viewport->y(),
                         clipPos.z());
    };
    auto getTextScreenBoundingBox = [&](osgText::Text *text,
                                        osg::Camera *camera) -> osg::BoundingBox {
        if (!text || !camera || !camera->getViewport())
            return osg::BoundingBox();
        // 获取世界位置和字符高度
        osg::Vec3 worldPos = text->getPosition();
        float charHeight = text->getCharacterHeight();

        // 估算屏幕高度（从当前位置向上一个字符高度）
        osg::Vec3 worldTop = worldPos + osg::Vec3(0.0f, 0.0f, charHeight);
        float pixelHeight = fabs(worldToScreen(worldTop).y() - worldToScreen(worldPos).y());

        // 获取文字长度
        std::string content = text->getText().createUTF8EncodedString();
        size_t charCount = content.length();

        // 估算宽高比（英文字符为0.5~0.6，中文字符更接近1.0）
        float aspectRatio = 0.6f;
        float pixelWidth = pixelHeight * aspectRatio * charCount;

        // 左下角屏幕坐标
        osg::Vec3 screenOrigin = worldToScreen(worldPos);

        // 构造屏幕空间包围盒
        osg::BoundingBox screenBB;
        screenBB.set(screenOrigin.x(), screenOrigin.y(), 0.0f, screenOrigin.x() + pixelWidth,
                     screenOrigin.y() + pixelHeight, 0.0f);
        // 增加边界的偏移量，单位为像素
        float extraPadding = 0.0f; // 根据需要调整这个值
        // 创建一个新的包围盒，在原包围盒的基础上加上额外的边界
        screenBB._min -= osg::Vec3(extraPadding, extraPadding, 0.0f); // 左下角扩展
        screenBB._max += osg::Vec3(extraPadding, extraPadding, 0.0f); // 右上角扩展
        return screenBB;
    };

    struct TextInfo {
        osg::BoundingBox screenBB;
        osg::Vec3 originalPos;
        osg::Vec3 screenPos;
        int gridRow;
        int gridCol;
        bool isVisible;
    };
    std::vector<TextInfo> textInfos(texts.size());

    // 第一步：转换所有文字到屏幕坐标并计算网格位置
    for (size_t i = 0; i < texts.size(); ++i) {
        auto &text = texts[i];
        auto &info = textInfos[i];
        info.originalPos = text->getPosition();
        info.screenPos = worldToScreen(info.originalPos);
        info.screenBB = getTextScreenBoundingBox(text.get(), camera.get());

        // 计算网格位置
        info.gridRow = static_cast<int>(info.screenPos.y() / GRID_SIZE);
        info.gridCol = static_cast<int>(info.screenPos.x() / GRID_SIZE);

        // 确保网格索引在有效范围内
        info.gridRow = osg::clampBetween(info.gridRow, 0, GRID_ROWS - 1);
        info.gridCol = osg::clampBetween(info.gridCol, 0, GRID_COLS - 1);

        // 将文字索引添加到对应的网格中
        spatialGrid[info.gridRow][info.gridCol].push_back(i);

        info.isVisible = true;
    }
    // 反投影
    osg::Matrix VPInv =
        osg::Matrix::inverse(camera->getViewMatrix() * camera->getProjectionMatrix());
    auto screenToWorld = [&](const osg::Vec3 &screen) -> osg::Vec3 {
        float x = (screen.x() - viewport->x()) / viewport->width() * 2.0f - 1.0f;
        float y = (screen.y() - viewport->y()) / viewport->height() * 2.0f - 1.0f;
        float z = screen.z();

        osg::Vec4 ndc(x, y, z, 1.0f);
        osg::Vec4 world = ndc * VPInv;
        if (world.w() != 0.0f)
            world /= world.w();
        return osg::Vec3(world.x(), world.y(), world.z());
    };

    // 第二步：处理碰撞
    for (size_t i = 0; i < texts.size(); ++i) {
        auto &info = textInfos[i];
        if (!info.isVisible)
            continue;
        // 获取文字的屏幕空间包围盒
        float pixelWidth = info.screenBB._max.x() - info.screenBB._min.x(); // 计算文字宽度（像素）

        const float MAX_OFFSET = (pixelWidth) / 1000.0f; // 最大偏移距离
        const float STEP = MAX_OFFSET / 2.0f;            // 每次尝试偏移的步长
        // 检查相邻网格中的文字
        bool foundValidPosition = false;
        float currentOffset = 0.0f;
        osg::Vec3 bestScreenPos = info.screenPos; // 保存找到的最佳屏幕位置

        // 首先检查原始位置是否有碰撞
        bool hasInitialCollision = false;
        for (int dr = -1; dr <= 1 && !hasInitialCollision; ++dr) {
            for (int dc = -1; dc <= 1 && !hasInitialCollision; ++dc) {
                int checkRow = info.gridRow + dr;
                int checkCol = info.gridCol + dc;

                if (checkRow < 0 || checkRow >= GRID_ROWS || checkCol < 0 || checkCol >= GRID_COLS)
                    continue;

                for (int idx : spatialGrid[checkRow][checkCol]) {
                    if (idx == i)
                        continue;

                    if (info.screenBB.intersects(textInfos[idx].screenBB)) {
                        hasInitialCollision = true;
                        std::cout << "Checking text " << i << " with screenBB: " << idx
                                  << std::endl;
                        break;
                    }
                }
            }
        }

        // 如果原始位置没有碰撞，直接使用
        if (!hasInitialCollision) {
            foundValidPosition = true;
        } else {
            // 尝试不同的偏移位置
            while (currentOffset <= MAX_OFFSET && !foundValidPosition) {
                // 尝试8个方向的偏移
                for (int angle = 0; angle < 8; ++angle) {
                    float theta = angle * osg::PI_4;
                    osg::Vec3 screenOffset(cos(theta) * currentOffset, sin(theta) * currentOffset,
                                           0.0f);

                    osg::Vec3 testPos = info.screenPos + screenOffset;
                    osg::BoundingBox testBB = info.screenBB;
                    testBB._min += screenOffset;
                    testBB._max += screenOffset;

                    bool hasCollision = false;

                    // 检查周围网格
                    int testGridRow = static_cast<int>(testPos.y() / GRID_SIZE);
                    int testGridCol = static_cast<int>(testPos.x() / GRID_SIZE);

                    for (int dr = -1; dr <= 1 && !hasCollision; ++dr) {
                        for (int dc = -1; dc <= 1 && !hasCollision; ++dc) {
                            int checkRow = testGridRow + dr;
                            int checkCol = testGridCol + dc;

                            if (checkRow < 0 || checkRow >= GRID_ROWS || checkCol < 0 ||
                                checkCol >= GRID_COLS)
                                continue;

                            for (int idx : spatialGrid[checkRow][checkCol]) {
                                if (idx == i)
                                    continue;

                                if (testBB.intersects(textInfos[idx].screenBB)) {
                                    hasCollision = true;
                                    break;
                                }
                            }
                        }
                    }

                    if (!hasCollision) {
                        bestScreenPos = testPos;
                        foundValidPosition = true;
                        break;
                    }
                }

                currentOffset += STEP;
            }
        }

        if (foundValidPosition) {
            // 计算屏幕空间的偏移量
            // osg::Vec3 totalScreenOffset = bestScreenPos - info.screenPos;
            //// 根据屏幕偏移量计算世界空间的偏移
            // float scale = info.originalPos.length() * 0.001f;
            // osg::Vec3 worldOffset(totalScreenOffset.x() * scale, totalScreenOffset.y() * scale,
            //                       0.0f);

            //// 设置新位置
            // texts[i]->setPosition(info.originalPos + worldOffset);

            osg::Vec3 screenFrom = info.screenPos;
            osg::Vec3 screenTo = bestScreenPos;

            osg::Vec3 worldFrom = screenToWorld(screenFrom);
            osg::Vec3 worldTo = screenToWorld(screenTo);

            osg::Vec3 worldOffset = worldTo - worldFrom;
            texts[i]->setPosition(info.originalPos + worldOffset);

            // 更新网格
            spatialGrid[info.gridRow][info.gridCol].erase(
                std::remove(spatialGrid[info.gridRow][info.gridCol].begin(),
                            spatialGrid[info.gridRow][info.gridCol].end(), i),
                spatialGrid[info.gridRow][info.gridCol].end());
            // 重新计算网格位置
            info.screenPos = worldToScreen(info.originalPos + worldOffset); // 更新新的屏幕坐标
            info.screenBB =
                getTextScreenBoundingBox(texts[i].get(), camera.get()); // 重新计算包围盒

            info.gridRow = static_cast<int>(info.screenPos.y() / GRID_SIZE);
            info.gridCol = static_cast<int>(info.screenPos.x() / GRID_SIZE);

            // 确保网格索引在有效范围内
            info.gridRow = osg::clampBetween(info.gridRow, 0, GRID_ROWS - 1);
            info.gridCol = osg::clampBetween(info.gridCol, 0, GRID_COLS - 1);

            // 更新网格
            spatialGrid[info.gridRow][info.gridCol].push_back(i);
        } else {
            // 如果没找到合适的位置，隐藏文字
            texts[i]->setColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
            info.isVisible = false;
            // 从原来的网格中删除它
            spatialGrid[info.gridRow][info.gridCol].erase(
                std::remove(spatialGrid[info.gridRow][info.gridCol].begin(),
                            spatialGrid[info.gridRow][info.gridCol].end(), i),
                spatialGrid[info.gridRow][info.gridCol].end());
        }
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
    std::shared_ptr<std::map<std::string, Node>> nodesWithLevel = graphParam->nodes;
    // 添加新的标签
    for (const auto &labelId : newAddList) {
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
            osg::Vec3 pos = vec3ToSphere(it->second.pos);
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
    adjustTextPosition(textNodes, graphParam->nodeGeomSize, graphParam->_camera);
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

    // 调试：检查数据状态
    std::cout << "\n=== Debugging Data Status ===" << std::endl;
    debugNodeCoordinates(lodNodesData[3], 5);
    debugEarthGridStatus();
    std::cout << "============================\n" << std::endl;

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
int VIS4Earth::GraphRenderer::getCurrentLevel(double height) {
    std::cout << "height:" << height << std::endl;
    if (height < 0)
        return 0;
    if (height > 2.64834e+07)
        return 0; // 全球级
    else if (height > 1.73736e+07)
        return 1; // 大陆级
    else if (height > 1.02563e+07)
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
void hexToRGBf(const std::string &hex, float &r, float &g, float &b) {
    std::string hexCode = (hex[0] == '#') ? hex.substr(1) : hex;

    // 假设 hexCode 长度始终为 6，且合法
    int ri = std::stoi(hexCode.substr(0, 2), nullptr, 16);
    int gi = std::stoi(hexCode.substr(2, 2), nullptr, 16);
    int bi = std::stoi(hexCode.substr(4, 2), nullptr, 16);

    r = ri / 255.0f;
    g = gi / 255.0f;
    b = bi / 255.0f;
}
std::string rgbToHex(float r, float g, float b) {
    // 将 [0.0, 1.0] 范围的 r, g, b 转换为 [0, 255] 范围
    int ri = static_cast<int>(r * 255);
    int gi = static_cast<int>(g * 255);
    int bi = static_cast<int>(b * 255);

    // 使用 ostringstream 拼接每个分量的 16 进制字符串
    std::ostringstream oss;
    oss << "#" << (ri < 16 ? "0" : "") << std::hex << ri << (gi < 16 ? "0" : "") << std::hex << gi
        << (bi < 16 ? "0" : "") << std::hex << bi;

    return oss.str();
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
            node.pos = osg::Vec3(itr->second.pos.x, itr->second.pos.y, itr->second.pos.z);
            float r, g, b;
            hexToRGBf(itr->second.color, r, g, b);
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
            // edges->emplace_back(edge);
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
            graphParam->generateHierarchicalGraphs(nodes, edges);
            graphParam->setLevelGraph(0);
            graphParam->setCamera(param._camera);

            // 初始化LOD数据 (在GraphRenderer层面)
            this->initializeLODData(nodes, edges);
            updateActiveLOD(cameraHeightPresent);
            // graphParam->update();
            sceneLabels.clear();
            cameraUpdate("LoadedGraph", cameraHeightPresent);
            // loadMarker();
        }

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
        graphParam->generateHierarchicalGraphs(nodes, edges);
        graphParam->setLevelGraph(0);
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
        graphParam->generateHierarchicalGraphs(nodes, edges);
        graphParam->setLevelGraph(0);
        graphParam->setCamera(param._camera);
        graphParam->update();
        cameraUpdate("LoadedGraph", cameraHeightPresent);

        // loadMarker();
    }
}

void VIS4Earth::GraphRenderer::showBundling() {
    // 首先尝试加载已有的边绑定结果
    // QString bundledEdgesFile =
    //    "C:/Users/DL/Desktop/data/graph_data/usflight/bundled_edges_result.csv";
    // if (QFile::exists(bundledEdgesFile)) {
    //    // 如果存在缓存文件，直接读取
    //    try {
    //        QFile file(bundledEdgesFile);
    //        if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    //            QTextStream in(&file);

    //            auto nodes = std::make_shared<std::map<std::string, Node>>();
    //            auto edges = std::make_shared<std::vector<Edge>>();

    //            // 复制节点信息
    //            for (const auto &node : myGraph->getNodes()) {
    //                Node newNode;
    //                newNode.pos = osg::Vec3(node.second.pos.x, node.second.pos.y, 0.f);
    //                newNode.id = node.first;
    //                nodes->emplace(node.first, newNode);
    //            }

    //            // 读取边的信息
    //            while (!in.atEnd()) {
    //                QString line = in.readLine();
    //                QStringList fields = line.split(",");
    //                if (fields.size() >= 6) { // from,to,hasSubdiv,numPoints,x1,y1,...
    //                    Edge edge;
    //                    edge.from = fields[0].toStdString();
    //                    edge.to = fields[1].toStdString();

    //                    bool hasSubdiv = fields[2].toInt() == 1;
    //                    int numPoints = fields[3].toInt();

    //                    if (!hasSubdiv) {
    //                        // 只有起点和终点
    //                        float startX = fields[4].toFloat();
    //                        float startY = fields[5].toFloat();
    //                        float endX = fields[6].toFloat();
    //                        float endY = fields[7].toFloat();
    //                        edge.subDivs.emplace_back(osg::Vec3(startX, startY, 0.f));
    //                        edge.subDivs.emplace_back(osg::Vec3(endX, endY, 0.f));
    //                    } else {
    //                        // 有细分点
    //                        for (int i = 4; i < fields.size(); i += 2) {
    //                            float x = fields[i].toFloat();
    //                            float y = fields[i + 1].toFloat();
    //                            edge.subDivs.emplace_back(osg::Vec3(x, y, 0.f));
    //                        }
    //                    }
    //                    edges->push_back(edge);
    //                }
    //            }
    //            file.close();

    //            // 使用读取的结果更新图形
    //            auto lonOffs = 1.5f * (lonRng[1] - lonRng[0]);
    //            lonRng[0] += lonOffs;
    //            lonRng[1] += lonOffs;

    //            addGraph("LoadedGraph", nodes, edges);
    //            auto graphParam = getGraph("LoadedGraph");
    //            if (graphParam) {
    //                graphParam->graphTypeIndex = graphTypeIndex;
    //                graphParam->heightMap = heightMap;
    //                graphParam->setLongitudeRange(lonRng[0] * size, lonRng[1] * size);
    //                graphParam->setLatitudeRange(latRng[0] * size, latRng[1] * size);
    //                graphParam->setHeightFromCenterRange(
    //                    static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[0],
    //                    static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[1]);
    //                graphParam->setNodeGeometrySize(.02f *
    //                                                static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
    //                graphParam->setTextGeometrySize(.02f *
    //                                                static_cast<float>(osg::WGS_84_RADIUS_EQUATOR));
    //                graphParam->setRestriction(myRestriction);
    //                graphParam->restrictionOFF = true;
    //                graphParam->generateHierarchicalGraphs(nodes, edges);
    //                graphParam->setLevelGraph(0);
    //                graphParam->setCamera(param._camera);
    //                graphParam->update();
    //            }
    //            return;
    //        }
    //    } catch (const std::exception &e) {
    //        qDebug() << "Error loading bundled edges:" << e.what();
    //    }
    //}

    // 如果没有缓存文件或读取失败，执行边绑定计算
    auto edgeBundling = VIS4Earth::EdgeBundling();
    edgeBundling.SetGraph(myGraph);
    glm::vec3 gravitationCenter(-75.0, 30.0, 0.0);
    // 确保兼容性计算已完成
    if (compatibilityFuture.valid()) {
        compatibilityFuture.wait(); // 阻塞直到任务完成
    }
    // 在需要边绑定效果时才计算兼容性
    myGraph->buildCompatibilityListsIfNeeded();

    edgeBundling.SetParameter(mybundlingParam);
    edgeBundling.EdgeBundle();
    myGraph = edgeBundling.GetLayoutedGraph();

    // 保存边绑定结果
    // try {
    //    QFile file(bundledEdgesFile);
    //    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    //        QTextStream out(&file);

    //        // 保存所有边的信息
    //        for (const auto &edge : myGraph->getEdges()) {
    //            out << QString::fromStdString(edge.sourceLabel) << ","
    //                << QString::fromStdString(edge.targetLabel);

    //            // 保存细分点
    //            if (edge.subdivs.empty()) {
    //                // 标记为0表示无细分点，后面是2个点（起点终点）
    //                out << ",0,2";
    //                out << "," << edge.start.x << "," << edge.start.y << "," << edge.end.x << ","
    //                    << edge.end.y;
    //            } else {
    //                // 标记为1表示有细分点，后面是点的总数
    //                out << ",1," << edge.subdivs.size();
    //                out << "," << edge.start.x << "," << edge.start.y;
    //                for (const auto &subdiv : edge.subdivs) {
    //                    out << "," << subdiv.x << "," << subdiv.y;
    //                }
    //                out << "," << edge.end.x << "," << edge.end.y;
    //            }
    //            out << "\n";
    //        }
    //        file.close();
    //    }
    //} catch (const std::exception &e) {
    //    qDebug() << "Error saving bundled edges:" << e.what();
    //}

    // 更新显示
    auto lonOffs = 1.5f * (lonRng[1] - lonRng[0]);
    lonRng[0] += lonOffs;
    lonRng[1] += lonOffs;
    auto nodes = std::make_shared<std::map<std::string, Node>>();
    auto edges = std::make_shared<std::vector<Edge>>();

    // 复制节点和边的信息
    copyGraphData(nodes, edges);

    // 添加图到渲染器中
    addGraphForBundling("LoadedGraph", nodes, edges);
    // 更新图渲染
    auto graphParam = getGraph("LoadedGraph");
    if (graphParam) {
        graphParam->currentLODLevel = 3;
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
        hexToRGBf(itr.second.color, node.color.x(), node.color.y(), node.color.z());
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
    // graphParam->startTextureFlowAnimation();
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
void VIS4Earth::GraphRenderer::onResolutionSliderValueChanged(int value) {
    // 将滑块的值转换为百分比
    int percentage = (value * 10);

    // 更新resolutionLabel的文本
    ui->resolutionLabel->setText(QString("分辨率: %1%").arg(percentage));
    auto graphParam = getGraph("LoadedGraph");
    graphParam->graphTypeIndex = graphTypeIndex;
    graphParam->setLevelGraph(10 - value);
    graphParam->update();
    sceneLabels.clear();
    cameraUpdate("LoadedGraph", cameraHeightPresent);
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
        // 第0行: 动画参数 (y=0.0)
        int paramPos = (0 * texWidth + x) * 4;
        data[paramPos] = edge.highlightPos; // highlightPos
        data[paramPos + 1] = edge.speed;    // speed
        data[paramPos + 2] = 0.0f;          // 保留
        data[paramPos + 3] = 0.0f;          // 保留

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
            // 更新高光位置
            _lines->at(i).highlightPos += _lines->at(i).speed * deltaTime;
            if (_lines->at(i).highlightPos >= 1.0f) {
                _lines->at(i).highlightPos = 0.0f; // 重置到起点
            }

            int baseIdx = i * 4;
            _paramCache[baseIdx] = _lines->at(i).highlightPos; // 高光位置
            _paramCache[baseIdx + 1] = _lines->at(i).speed;    // 速度
            _paramCache[baseIdx + 2] = 0.0f;                   // 保留
            _paramCache[baseIdx + 3] = 0.0f;                   // 保留
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
    std::vector<float> _paramCache;
    bool _firstFrame;
};
class TextureBasedAnimationColorCallback : public osg::NodeCallback {
  public:
    TextureBasedAnimationColorCallback(osg::Image *lineDataImage,
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
                fmod(_lines->at(i).highlightPos + _lines->at(i).speed * deltaTime, 1.0f);

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

void main() {
    vPosition = vertexPosition;
    vLineID = lineID;
    
    // 从纹理获取当前线段的起点终点
    float texX = (lineID + 0.5) / uTotalLines;
    vLineStart = texture2D(uLineDataTex, vec2(texX, 0.25)).rgb;
    vLineEnd = texture2D(uLineDataTex, vec2(texX, 0.5)).rgb;
    
    gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexPosition, 1.0);
}
)";

    std::string fragSource = R"(
#version 120
uniform sampler2D uLineDataTex;
uniform float uTotalLines;
uniform float uHighlightWidth;
uniform vec4 uHighlightColor;
uniform float uGlowIntensity;    // 发光强度
uniform float uGlobalAlpha;      // 全局透明度
uniform float uLineThickness;    // 线条粗细

varying vec3 vPosition;
varying float vLineID;
varying vec3 vLineStart;
varying vec3 vLineEnd;

void main() {
    // 获取当前线段的动画参数
    float texX = (vLineID + 0.5) / uTotalLines;
    float highlightPos = texture2D(uLineDataTex, vec2(texX, 0.0)).r;
    
    // 计算当前点在线段上的投影位置 [0,1]
    vec3 lineVec = vLineEnd - vLineStart;
    float lineLength = length(lineVec);
    vec3 lineDir = lineVec / lineLength;
    float t = dot(vPosition - vLineStart, lineDir) / lineLength;
    t = clamp(t, 0.0, 1.0);
    
    // 计算到线段中心的距离（用于发光效果）
    vec3 projectedPos = vLineStart + t * lineVec;
    float centerDistance = length(vPosition - projectedPos);
    float glowRadius = uLineThickness * 2.0;
    float glowFactor = 1.0 - smoothstep(0.0, glowRadius, centerDistance);
    glowFactor = pow(glowFactor, 2.0); // 增强发光衰减
    
    // 计算高光效果（使用更窄的高光区域和更强的亮度）
    float dist = abs(t - highlightPos);
    float highlightWidth = uHighlightWidth * 0.5; // 减小高光宽度，使其更集中
    float highlightIntensity = 1.0 - smoothstep(0.0, highlightWidth, dist);
    highlightIntensity = pow(highlightIntensity, 0.5); // 使高光更亮
    
    // 基础颜色（深蓝色）(0.8f, 0.6f, 0.2f, 1.0f)
    vec4 baseColor = vec4(0.85f, 0.5f, 0.12f, uGlobalAlpha);
    
    // 发光效果
    vec3 glowColor = baseColor.rgb * uGlowIntensity * glowFactor;
    vec3 finalColor = baseColor.rgb + glowColor;
    
    // 高光颜色（明亮的白色）
    vec3 highlightColorRGB = vec3(1.0, 1.0, 1.0) * 1.2; // 增强高光亮度
    
    // 混合高光（使用更强的混合比例）
    finalColor = mix(finalColor, highlightColorRGB, highlightIntensity);
    
    // 最终透明度计算（增加高光处的透明度）
    float finalAlpha = baseColor.a * (1.0 + glowFactor * 0.5 + highlightIntensity * 0.5);
    finalAlpha = clamp(finalAlpha, 0.0, 1.0);
    
    gl_FragColor = vec4(finalColor, finalAlpha);
}
)";

    osg::ref_ptr<osg::Program> program = new osg::Program;
    program->addBindAttribLocation("vertexPosition", 2);
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
    t = clamp(t, 0.0, 1);

    // 关键改进：计算颜色权重（使用 cos 实现平滑循环）
    float colorWeight = 0.5 + 0.5 * cos(2.0 * 3.1415926 * (t - phase));
    
    // 混合颜色（蓝色 ↔ 黄色 ↔ 蓝色...）
    vec3 color = mix(blue, yellow, colorWeight);
    
    // 输出颜色（固定透明度 1.0）
    gl_FragColor = vec4(color, 0.2);
}
)";

    osg::ref_ptr<osg::Program> program = new osg::Program;
    // 必须显式绑定属性位置
    program->addBindAttribLocation("vertexPosition", 2);
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
    vec4 baseColor = vec4(0.85f, 0.5f, 0.12f, 0.2f);
    // 最终颜色
    gl_FragColor =mix(baseColor, uHighlightColor, highlightIntensity*1);
}
)";

    osg::ref_ptr<osg::Program> program = new osg::Program;
    // 必须显式绑定属性位置
    program->addBindAttribLocation("vertexPosition", 2);
    program->addBindAttribLocation("lineID", 1);
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));
    return program.release();
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
            arrowStates->setAttributeAndModes(createTextureBasedShaderProgram(edges->size()),
                                              osg::StateAttribute::ON);

            // 启用透明度混合
            osg::BlendFunc *blendFunc = new osg::BlendFunc();
            blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE); // 加法混合
            arrowStates->setAttributeAndModes(blendFunc, osg::StateAttribute::ON);

            // 禁用深度写入但保留深度测试
            osg::Depth *depth = new osg::Depth();
            depth->setWriteMask(false);
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
            arrowStates->addUniform(new osg::Uniform("uGlobalAlpha", 0.2f)); // 增加全局透明度
            arrowStates->addUniform(new osg::Uniform("uLineThickness", 1.5f)); // 减小线条粗细

            // 初始化每条边的动画参数
            for (auto &edge : *edges) {
                edge.highlightPos = 0.0f;
                edge.speed = 0.3f; // 降低移动速度，使高光更容易观察
            }

            // 设置动画回调
            lineGeometry->setUpdateCallback(
                new TextureBasedAnimationCallback(lineDataImage, edges));
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
                lineGeometry->setUpdateCallback(
                    new TextureBasedAnimationColorCallback(lineDataImage, edges));
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
                lineGeometry->setUpdateCallback(
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
    if (!edgeNodegrp) {
        edgeNodegrp = new osg::Group;
    }
    edgeNodegrp->removeChildren(0, edgeNodegrp->getNumChildren());
    grp->removeChildren(0, grp->getNumChildren());
    auto tessl = new osg::TessellationHints;
    tessl->setDetailRatio(1.f);
    std::map<std::string, osg::ShapeDrawable *> osgNodes;
    std::vector<osg::ref_ptr<osgText::Text>> textNodes;
    
    
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
                color = osg::Vec4(1.0f, 1.0f, 1.0f, 0.5f); // 设置边框内的点为半透明白色
            }
        }
        auto p = itr->second.pos;
        if (p.z() < 100.f) {
            p.z() = getBuildingHeightAtLatLon(p.x(), p.y()) + 100.0f;
        }
        p = vec3ToSphere(p);
        int scale = .050f;
        if (itr->second.level == 100) {
            scale = 1.f;
        }
        auto sphere = new osg::ShapeDrawable(
            new osg::Sphere(p, itr->second.size * scale * nodeGeomSize), tessl);
        osg::ref_ptr<osg::Vec3Array> centerData = new osg::Vec3Array;
        centerData->push_back(itr->second.pos);
        sphere->setUserData(centerData);
        sphere->setColor(color);
        edgeNodegrp->addChild(sphere);
    }

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
            osg::Vec3 startPos = vec3ToSphere(prevPos); // 起点
            osg::Vec3 endPos = nodes->at(edge.to).pos;
            endPos = vec3ToSphere(endPos);

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
            initEdgeShaders();
        }

        // 更新VBO数据
        // LOD 3: 使用原有逻辑，完整显示所有边
        updateEdgeVBO();
    }
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
                
                gl_FragColor = vec4(finalColor, finalAlpha);
            }
        )";

        mEdgeProgram->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
        mEdgeProgram->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));
        mEdgeProgram->addBindAttribLocation("lineID", 1); // lineID -> slot 1
        mEdgeProgram->addBindAttribLocation("vertexPosition", 2); // lineID -> slot 1
    }

    // 每次调用都为当前几何体设置StateSet和uniform参数
    if (mEdgeGeometry) {
        auto stateset = mEdgeGeometry->getOrCreateStateSet();
        stateset->setAttributeAndModes(mEdgeProgram, osg::StateAttribute::ON);

        // 启用透明度混合
        osg::BlendFunc *blendFunc = new osg::BlendFunc();
        blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE); // Additive blending
        stateset->setAttributeAndModes(blendFunc, osg::StateAttribute::ON);

        // 禁用深度写入但保留深度测试
        osg::Depth *depth = new osg::Depth();
        depth->setWriteMask(false);
        stateset->setAttributeAndModes(depth, osg::StateAttribute::ON);

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
    osg::ref_ptr<osg::FloatArray> mLineIDArray = new osg::FloatArray;
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
    auto isLinePassingThroughEarth = [&](const osg::Vec3 &surfacePoint,
                                         const osg::Vec3 &satellite) -> bool {
        // 地球半径（使用极地半径作为基准）
        const float earthRadius = osg::WGS_84_RADIUS_POLAR;

        // 计算线段参数化方程: P(t) = surfacePoint + t * (satellite - surfacePoint)
        // 其中 t ∈ [0, 1]
        osg::Vec3 direction = satellite - surfacePoint;

        // 计算线段到原点（地心）的最近距离
        // 对于线段 P(t) = A + t * (B - A)，到原点距离的平方为：
        // |P(t)|² = |A + t*D|² = |A|² + 2t*(A·D) + t²*|D|²
        // 其中 A = surfacePoint, D = direction

        float a = direction.length2();               // |D|²
        float b = 2.0f * (surfacePoint * direction); // 2*(A·D)
        float c = surfacePoint.length2();            // |A|²

        // 如果 a 接近 0，说明两点几乎重合
        if (std::abs(a) < 1e-6f) {
            return surfacePoint.length() < earthRadius;
        }

        // 求导数为0的点：d/dt|P(t)|² = 2*(A·D) + 2t*|D|² = 0
        // 得到 t = -(A·D) / |D|²
        float t = -b / (2.0f * a);

        // 将 t 限制在 [0, 1] 范围内（线段范围）
        t = std::max(0.0f, std::min(1.0f, t));

        // 计算线段上最近点到地心的距离
        osg::Vec3 closestPoint = surfacePoint + direction * t;
        float minDistance = closestPoint.length();

        // 如果最近距离小于地球半径，则线段穿过地球
        return minDistance < earthRadius;
    };


    // 创建顶点数组
    mVertexArray = new osg::Vec3Array;
    mColorFromArray = new osg::Vec4Array;
    mColorToArray = new osg::Vec4Array;
    mWeightArray = new osg::FloatArray;
    int lineID = 0;
    
    std::cout << "updateEdgeVBO: Using LOD level " << currentLODLevel << std::endl;

    if (currentLODLevel == 3) {
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
        cityLoader.drawBuildings(grp, latLonBounds, scale, currentLODLevel);
        heightMap = cityLoader.getHeightMap();

        updateEdgeVBO_Original(vec3ToSphere, mLineIDArray);
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
                if (isLinePassingThroughEarth(vec3ToSphere(endPoint), vec3ToSphere(startPoint))) {
                    lineID++;
                    edge.visible = false;
                    continue; // 如果穿过地球则跳过该线条
                }
            }
            else {
                if (isLinePassingThroughEarth(vec3ToSphere(startPoint), vec3ToSphere(endPoint))) {
                    lineID++;
                    continue; // 如果穿过地球则跳过该线条
                }
            }
            
            // 获取边的颜色（可以使用自定义的颜色或者节点的颜色）
            osg::Vec4 edgeColor(1.0f, 1.0f, 1.0f, 1.0f); // 默认白色，可以修改为其他颜色

            // 将边的起点和终点加入顶点数组
            mVertexArray->push_back(vec3ToSphere(startPoint));
            mColorFromArray->push_back(edgeColor);
            mColorToArray->push_back(edgeColor);

            mVertexArray->push_back(vec3ToSphere(endPoint));
            mColorFromArray->push_back(edgeColor);
            mColorToArray->push_back(edgeColor);

            // 记录该线段的标识符（如果需要）
            mLineIDArray->push_back(static_cast<float>(lineID));
            mLineIDArray->push_back(static_cast<float>(lineID));

            // 线段的权重（如果需要，可以根据边的属性设置权重）
            mWeightArray->push_back(edge.weight);
            mWeightArray->push_back(edge.weight);

            lineID++; // 增加线ID
        }
        else {
            // 计算边长度（大圆距离）
            float lat1 = osg::DegreesToRadians(startPoint.x());
            float lon1 = osg::DegreesToRadians(startPoint.y());
            float lat2 = osg::DegreesToRadians(endPoint.x());
            float lon2 = osg::DegreesToRadians(endPoint.y());

            float dlat = lat2 - lat1;
            float dlon = lon2 - lon1;
            float a = std::sin(dlat / 2) * std::sin(dlat / 2) +
                      std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2) * std::sin(dlon / 2);
            float c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
            float totalLength = 6371.0f * c; // 6371km是地球平均半径

            // 计算细分段数
            int totalSegments = static_cast<int>(BASE_SEGMENTS * (totalLength / BASE_LENGTH));
            totalSegments = std::max(MIN_SEGMENTS, std::min(MAX_SEGMENTS, totalSegments));

            // osg::Vec4 startColor = osg::Vec4(fromNodeIt->second.color, 0.0f);
            // osg::Vec4 endColor = osg::Vec4(toNodeIt->second.color, 0.5f);
            // osg::Vec4 edgeColor = (startColor + endColor) * 0.5f;
            //  获取边的颜色（使用节点颜色或默认颜色）
            osg::Vec4 edgeColor(0.8f, 0.6f, 0.2f, 1.0f); // 默认金色
            if (currentLODLevel < 3) {
                // 聚合边使用统一的金色
                edgeColor = osg::Vec4(0.8f, 0.6f, 0.2f, 1.0f);
            }

            osg::Vec3 prevPos;

            // 生成边的插值点
            for (int j = 0; j <= totalSegments; ++j) {
                float t = static_cast<float>(j) / totalSegments;

                // 线性插值位置
                osg::Vec3 interpolatedPos;
                interpolatedPos.x() =
                    fromNodeIt->second.pos.x() * (1.0f - t) + toNodeIt->second.pos.x() * t;
                interpolatedPos.y() =
                    fromNodeIt->second.pos.y() * (1.0f - t) + toNodeIt->second.pos.y() * t;

                // 计算弧线高度
                float baseHeight =
                    getBuildingHeightAtLatLon(interpolatedPos.x(), interpolatedPos.y());
                float arcHeight = std::sin(osg::PI * t) * 100000.f; // 使用边的最大高度
                interpolatedPos.z() = std::max(baseHeight, arcHeight);

                // 转换为球面坐标
                osg::Vec3 spherePos = vec3ToSphere(interpolatedPos);

                if (j > 0) {
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
    auto isLinePassingThroughEarth = [&](const osg::Vec3 &surfacePoint,
                                         const osg::Vec3 &satellite) -> bool {
        // 地球半径（使用极地半径作为基准）
        const float earthRadius = osg::WGS_84_RADIUS_POLAR;

        // 计算线段参数化方程: P(t) = surfacePoint + t * (satellite - surfacePoint)
        // 其中 t ∈ [0, 1]
        osg::Vec3 direction = satellite - surfacePoint;

        // 计算线段到原点（地心）的最近距离
        // 对于线段 P(t) = A + t * (B - A)，到原点距离的平方为：
        // |P(t)|² = |A + t*D|² = |A|² + 2t*(A·D) + t²*|D|²
        // 其中 A = surfacePoint, D = direction

        float a = direction.length2();               // |D|²
        float b = 2.0f * (surfacePoint * direction); // 2*(A·D)
        float c = surfacePoint.length2();            // |A|²

        // 如果 a 接近 0，说明两点几乎重合
        if (std::abs(a) < 1e-6f) {
            return surfacePoint.length() < earthRadius;
        }

        // 求导数为0的点：d/dt|P(t)|² = 2*(A·D) + 2t*|D|² = 0
        // 得到 t = -(A·D) / |D|²
        float t = -b / (2.0f * a);

        // 将 t 限制在 [0, 1] 范围内（线段范围）
        t = std::max(0.0f, std::min(1.0f, t));

        // 计算线段上最近点到地心的距离
        osg::Vec3 closestPoint = surfacePoint + direction * t;
        float minDistance = closestPoint.length();

        // 如果最近距离小于地球半径，则线段穿过地球
        return minDistance < earthRadius;
    };
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
                if (isLinePassingThroughEarth(vec3ToSphere(endPoint), vec3ToSphere(startPoint))) {
                    lineID++;
                    edge.visible = false;
                    continue; // 如果穿过地球则跳过该线条
                }
            } else {
                if (isLinePassingThroughEarth(vec3ToSphere(startPoint), vec3ToSphere(endPoint))) {
                    lineID++;
                    continue; // 如果穿过地球则跳过该线条
                }
            }

            // 获取边的颜色（可以使用自定义的颜色或者节点的颜色）
            osg::Vec4 edgeColor(1.0f, 1.0f, 1.0f, 1.0f); // 默认白色，可以修改为其他颜色

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
        if (node.level <= lodLevel || node.level==100) {
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

// 设置活动LOD数据源 (从GraphRenderer获取数据)
void VIS4Earth::GraphRenderer::PerGraphParam::setActiveLODDataSource(int targetMaxLevel) {
    // 这个方法现在需要通过GraphRenderer来调用，暂时保留接口
    std::cout << "setActiveLODDataSource called with level " << targetMaxLevel << std::endl;
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
    auto isLinePassingThroughEarth = [&](const osg::Vec3 &surfacePoint,
                                         const osg::Vec3 &satellite) -> bool {
        // 地球半径（使用极地半径作为基准）
        const float earthRadius = osg::WGS_84_RADIUS_POLAR;

        // 计算线段参数化方程: P(t) = surfacePoint + t * (satellite - surfacePoint)
        // 其中 t ∈ [0, 1]
        osg::Vec3 direction = satellite - surfacePoint;

        // 计算线段到原点（地心）的最近距离
        // 对于线段 P(t) = A + t * (B - A)，到原点距离的平方为：
        // |P(t)|² = |A + t*D|² = |A|² + 2t*(A·D) + t²*|D|²
        // 其中 A = surfacePoint, D = direction

        float a = direction.length2();               // |D|²
        float b = 2.0f * (surfacePoint * direction); // 2*(A·D)
        float c = surfacePoint.length2();            // |A|²

        // 如果 a 接近 0，说明两点几乎重合
        if (std::abs(a) < 1e-6f) {
            return surfacePoint.length() < earthRadius;
        }

        // 求导数为0的点：d/dt|P(t)|² = 2*(A·D) + 2t*|D|² = 0
        // 得到 t = -(A·D) / |D|²
        float t = -b / (2.0f * a);

        // 将 t 限制在 [0, 1] 范围内（线段范围）
        t = std::max(0.0f, std::min(1.0f, t));

        // 计算线段上最近点到地心的距离
        osg::Vec3 closestPoint = surfacePoint + direction * t;
        float minDistance = closestPoint.length();

        // 如果最近距离小于地球半径，则线段穿过地球
        return minDistance < earthRadius;
    };
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
            case 0: // LOD0 - 最粗糙级别，需要强烈发光突出聚合边
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
                                    rgbToHex(itr->second.color.x(), itr->second.color.y(),
                                             itr->second.color.z()))));
            }
            std::vector<VIS4Earth::Edge> garphEdges;

            for (auto itr = graphParam->edges->begin(); itr != graphParam->edges->end(); ++itr) {

                osg::Vec3 fromPos = graphParam->nodes->at(itr->from).pos;
                osg::Vec3 toPos = graphParam->nodes->at(itr->to).pos;
                if (graphParam->nodes->at(itr->from).level == 100 &&
                    isLinePassingThroughEarth(vec3ToSphere(toPos), vec3ToSphere(toPos))) {
                    continue;
                }
                if (graphParam->nodes->at(itr->to).level == 100 &&
                    isLinePassingThroughEarth(vec3ToSphere(fromPos), vec3ToSphere(toPos))) {
                    continue;
                }
                glm::vec3 from = glm::vec3(fromPos.x(), fromPos.y(), fromPos.z());
                glm::vec3 to = glm::vec3(toPos.x(), toPos.y(), toPos.z());
                garphEdges.push_back(
                    VIS4Earth::Edge(itr->from, itr->to, from, to, itr->weight));
                
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

// 简化区域名称
std::string VIS4Earth::GraphRenderer::simplifyRegionName(const std::string &originalName) {
    // 将长区域名简化为更短的版本
    if (originalName.find("North_America") != std::string::npos ||
        originalName.find("USA") != std::string::npos ||
        originalName.find("Canada") != std::string::npos) {
        return "N_America";
    }
    if (originalName.find("South_America") != std::string::npos ||
        originalName.find("Brazil") != std::string::npos ||
        originalName.find("Argentina") != std::string::npos) {
        return "S_America";
    }
    if (originalName.find("Europe") != std::string::npos ||
        originalName.find("Germany") != std::string::npos ||
        originalName.find("France") != std::string::npos ||
        originalName.find("UK") != std::string::npos) {
        return "Europe";
    }
    if (originalName.find("Asia") != std::string::npos ||
        originalName.find("China") != std::string::npos ||
        originalName.find("Japan") != std::string::npos) {
        return "Asia_East";
    }
    if (originalName.find("Africa") != std::string::npos) {
        return "Africa";
    }
    if (originalName.find("Oceania") != std::string::npos ||
        originalName.find("Australia") != std::string::npos) {
        return "Oceania";
    }

    // 默认情况下，简化为前8个字符
    size_t presetL = 8;
    std::string simplified = originalName.substr(0, std::min(presetL, originalName.length()));
    // 移除下划线
    std::replace(simplified.begin(), simplified.end(), '_', ' ');
    return simplified;
}

// 计算区域质心
osg::Vec3 VIS4Earth::GraphRenderer::calculateRegionCentroid(
    const std::vector<std::string> &nodeIds,
    std::shared_ptr<std::map<std::string, Node>> allNodes) {

    if (nodeIds.empty()) {
        return osg::Vec3(0, 0, 0);
    }

    double totalLat = 0.0, totalLon = 0.0, totalWeight = 0.0;

    for (const std::string &nodeId : nodeIds) {
        auto nodeIt = allNodes->find(nodeId);
        if (nodeIt != allNodes->end()) {
            const Node &node = nodeIt->second;
            // 使用degree作为权重，degree越高权重越大
            double weight = std::max(1.0, static_cast<double>(node.degree));

            totalLat += node.pos.x() * weight;
            totalLon += node.pos.y() * weight;
            totalWeight += weight;
        }
    }

    if (totalWeight > 0) {
        return osg::Vec3(totalLat / totalWeight, totalLon / totalWeight, 0);
    }

    return osg::Vec3(0, 0, 0);
}

// 评估节点重要性
float VIS4Earth::GraphRenderer::evaluateNodeImportance(const Node &node) {
    // 基于level和degree计算重要性分数
    // level越低分数越高，degree越高分数越高
    float levelScore =
        (4.0f - node.level) * 10.0f; // level 0=40分, level 1=30分, level 2=20分, level 3=10分
    float degreeScore = std::min(node.degree, 50) * 1.0f; // degree最多贡献50分

    return levelScore + degreeScore;
}

// 从实际数据选择代表节点
std::vector<std::string> VIS4Earth::GraphRenderer::selectRepresentativeNodes(
    const std::vector<std::string> &nodeIds, std::shared_ptr<std::map<std::string, Node>> allNodes,
    int maxNodes, int minLevel, int maxLevel) {

    std::vector<std::pair<std::string, float>> nodeScores;

    // 计算每个节点的重要性分数
    for (const std::string &nodeId : nodeIds) {
        auto nodeIt = allNodes->find(nodeId);
        if (nodeIt != allNodes->end()) {
            const Node &node = nodeIt->second;

            // 只考虑指定level范围内的节点
            if (node.level >= minLevel && node.level <= maxLevel) {
                float importance = evaluateNodeImportance(node);
                nodeScores.push_back({nodeId, importance});
            }
        }
    }

    // 按重要性排序
    std::sort(nodeScores.begin(), nodeScores.end(),
              [](const std::pair<std::string, float> &a, const std::pair<std::string, float> &b) {
                  return a.second > b.second; // 重要性高的在前
              });

    // 选择前maxNodes个
    std::vector<std::string> selected;
    for (int i = 0; i < std::min(maxNodes, static_cast<int>(nodeScores.size())); i++) {
        selected.push_back(nodeScores[i].first);
    }

    return selected;
}

// 生成LOD0基础代表点
void VIS4Earth::GraphRenderer::generateBaseLODNodes(
    std::shared_ptr<std::map<std::string, Node>> lodNodes,
    const std::vector<GeographicRegion> &regions,
    const std::map<int, std::vector<std::string>> &regionNodes,
    std::shared_ptr<std::map<std::string, Node>> allNodes) {

    std::cout << "Generating LOD0 base representative nodes..." << std::endl;
    static int lod0NodeCounter = 0; // 用于生成唯一的数字ID

    for (const auto &regionPair : regionNodes) {
        int regionId = regionPair.first;
        const std::vector<std::string> &nodeIds = regionPair.second;

        if (nodeIds.empty())
            continue;

        // 找到区域信息
        const GeographicRegion *regionInfo = nullptr;
        for (const auto &region : regions) {
            if (region.regionId == regionId) {
                regionInfo = &region;
                break;
            }
        }

        if (!regionInfo)
            continue;

        // 创建区域质心代表节点，使用数字ID
        std::string repNodeId = std::to_string(lod0NodeCounter++);
        Node repNode;

        // 使用计算出的质心位置
        repNode.pos = calculateRegionCentroid(nodeIds, allNodes);
        repNode.id = repNodeId;
        repNode.color = osg::Vec3(1.0f, 0.6f, 0.0f); // 橙色
        repNode.visible = true;
        repNode.isRepresent = true;
        repNode.level = 0;   // LOD0代表节点
        repNode.size = 3.0f; // 较大尺寸
        repNode.degree = static_cast<int>(nodeIds.size());
        repNode.cluster = regionId;
        repNode.label = simplifyRegionName(regionInfo->regionName); // 描述性文本放在label中

        (*lodNodes)[repNodeId] = repNode;

        std::cout << "Created LOD0 representative: " << repNode.label << " at (" << repNode.pos.x()
                  << ", " << repNode.pos.y() << ") for " << nodeIds.size() << " nodes" << std::endl;
    }
}

// 为LOD1添加实际重要节点
void VIS4Earth::GraphRenderer::addLOD1Nodes(
    std::shared_ptr<std::map<std::string, Node>> lodNodes,
    const std::map<int, std::vector<std::string>> &regionNodes,
    std::shared_ptr<std::map<std::string, Node>> allNodes) {

    std::cout << "Adding LOD1 actual important nodes..." << std::endl;
    static int lod1NodeCounter = 10000; // LOD1节点ID从10000开始，避免与LOD0冲突

    for (const auto &regionPair : regionNodes) {
        int regionId = regionPair.first;
        const std::vector<std::string> &nodeIds = regionPair.second;

        // 选择每个区域内最重要的2-3个实际节点
        // 标准：degree > 高阈值 && level <= 1
        std::vector<std::string> representatives =
            selectRepresentativeNodes(nodeIds, allNodes, 3, 0, 1 // 最多3个，level 0-1
            );

        // 添加这些实际节点到LOD1
        for (const std::string &nodeId : representatives) {
            auto nodeIt = allNodes->find(nodeId);
            if (nodeIt != allNodes->end()) {
                Node actualNode = nodeIt->second;                  // 复制实际节点
                actualNode.id = std::to_string(lod1NodeCounter++); // 使用数字ID
                actualNode.color = osg::Vec3(0.0f, 0.8f, 1.0f);    // 蓝色
                actualNode.size = 2.0f;                            // 中等尺寸
                // 保持原有节点的label
                if (actualNode.label.empty()) {
                    actualNode.label = nodeId; // 如果原节点没有label，使用原始ID作为label
                }

                (*lodNodes)[actualNode.id] = actualNode;

                std::cout << "Added LOD1 node: " << actualNode.label
                          << " (degree=" << actualNode.degree << ", level=" << actualNode.level
                          << ")" << std::endl;
            }
        }
    }
}

// 为LOD2添加详细实际节点
void VIS4Earth::GraphRenderer::addLOD2Nodes(
    std::shared_ptr<std::map<std::string, Node>> lodNodes,
    const std::map<int, std::vector<std::string>> &regionNodes,
    std::shared_ptr<std::map<std::string, Node>> allNodes) {

    std::cout << "Adding LOD2 detailed nodes..." << std::endl;
    static int lod2NodeCounter = 20000; // LOD2节点ID从20000开始，避免与LOD0/LOD1冲突

    for (const auto &regionPair : regionNodes) {
        int regionId = regionPair.first;
        const std::vector<std::string> &nodeIds = regionPair.second;

        // 选择每个区域内的详细节点
        // 标准：level <= 2 的所有节点
        std::vector<std::string> detailedNodes =
            selectRepresentativeNodes(nodeIds, allNodes, 10, 0, 2 // 最多10个，level 0-2
            );

        // 添加这些详细节点到LOD2
        for (const std::string &nodeId : detailedNodes) {
            auto nodeIt = allNodes->find(nodeId);
            if (nodeIt != allNodes->end()) {
                Node actualNode = nodeIt->second;                  // 复制实际节点
                actualNode.id = std::to_string(lod2NodeCounter++); // 使用数字ID
                actualNode.color = osg::Vec3(0.2f, 1.0f, 0.2f);    // 绿色
                actualNode.size = 1.5f;                            // 较小尺寸
                // 保持原有节点的label
                if (actualNode.label.empty()) {
                    actualNode.label = nodeId; // 如果原节点没有label，使用原始ID作为label
                }

                (*lodNodes)[actualNode.id] = actualNode;

                std::cout << "Added LOD2 node: " << actualNode.label
                          << " (degree=" << actualNode.degree << ", level=" << actualNode.level
                          << ")" << std::endl;
            }
        }
    }
}

// 生成渐进式边连接
void VIS4Earth::GraphRenderer::generateProgressiveEdges(
    int lodLevel, std::shared_ptr<std::vector<Edge>> lodEdges,
    std::shared_ptr<std::map<std::string, Node>> lodNodes,
    const std::map<int, std::vector<std::string>> &regionNodes,
    std::map<int, std::string> &regionRepresentatives, std::shared_ptr<std::vector<Edge>> allEdges,
    std::shared_ptr<std::map<std::string, Node>> allNodes) {

    if (lodLevel == 0) {
        // LOD0：生成区域间聚合边
        generateLOD0Edges(lodEdges, regionRepresentatives, allEdges, allNodes);
    } else if (lodLevel == 1) {
        // LOD1：继承LOD0边 + 新增实际节点连接
        generateLOD1Edges(lodEdges, lodNodes, regionNodes, regionRepresentatives, allEdges,
                          allNodes);
    } else if (lodLevel == 2) {
        // LOD2：继承LOD1边 + 新增详细连接
        generateLOD2Edges(lodEdges, lodNodes, allEdges, allNodes);
    }
}

// 生成LOD0区域间聚合边
void VIS4Earth::GraphRenderer::generateLOD0Edges(
    std::shared_ptr<std::vector<Edge>> lodEdges, std::map<int, std::string> &regionRepresentatives,
    std::shared_ptr<std::vector<Edge>> allEdges,
    std::shared_ptr<std::map<std::string, Node>> allNodes) {

    static int lod0EdgeCounter = 0; // 用于生成唯一的边ID

    // 计算区域间连接并生成聚合边
    std::map<std::pair<int, int>, int> regionConnections;
    std::map<std::pair<int, int>, float> regionWeights;

    for (const auto &edge : *allEdges) {
        if (!edge.visible)
            continue;

        auto fromNodeIt = allNodes->find(edge.from);
        auto toNodeIt = allNodes->find(edge.to);
        if (fromNodeIt == allNodes->end() || toNodeIt == allNodes->end())
            continue;

        // 找到起点和终点节点所属的区域
        int fromRegion = -1, toRegion = -1;

        float fromLat = fromNodeIt->second.pos.x();
        float fromLon = fromNodeIt->second.pos.y();
        float toLat = toNodeIt->second.pos.x();
        float toLon = toNodeIt->second.pos.y();

        for (const auto &region : LOD0_REGIONS) {
            if (fromRegion == -1 && fromLat >= region.minLat && fromLat <= region.maxLat &&
                fromLon >= region.minLon && fromLon <= region.maxLon) {
                fromRegion = region.regionId;
            }
            if (toRegion == -1 && toLat >= region.minLat && toLat <= region.maxLat &&
                toLon >= region.minLon && toLon <= region.maxLon) {
                toRegion = region.regionId;
            }
            if (fromRegion != -1 && toRegion != -1)
                break;
        }

        if (fromRegion == -1 || toRegion == -1 || fromRegion == toRegion)
            continue;

        std::pair<int, int> regionPair =
            std::make_pair(std::min(fromRegion, toRegion), std::max(fromRegion, toRegion));

        regionConnections[regionPair]++;
        regionWeights[regionPair] += edge.weight;
    }

    std::cout << "Found " << regionConnections.size() << " inter-region connections" << std::endl;

    // 生成LOD0聚合边
    for (const auto &connPair : regionConnections) {
        int region1 = connPair.first.first;
        int region2 = connPair.first.second;
        float totalWeight = regionWeights[connPair.first];

        if (regionRepresentatives.find(region1) == regionRepresentatives.end() ||
            regionRepresentatives.find(region2) == regionRepresentatives.end()) {
            continue;
        }

        Edge aggregatedEdge;
        aggregatedEdge.id = std::to_string(lod0EdgeCounter++); // 使用数字ID
        aggregatedEdge.from = regionRepresentatives[region1];
        aggregatedEdge.to = regionRepresentatives[region2];
        aggregatedEdge.weight = totalWeight;
        aggregatedEdge.visible = true;
        aggregatedEdge.maxHeight = 60000.0f; // LOD0边较高

        lodEdges->push_back(aggregatedEdge);

        std::cout << "Created LOD0 aggregated edge " << aggregatedEdge.id << " between regions "
                  << region1 << " and " << region2 << " with weight=" << totalWeight << std::endl;
    }
}

// 生成LOD1边（实际节点间连接）
void VIS4Earth::GraphRenderer::generateLOD1Edges(
    std::shared_ptr<std::vector<Edge>> lodEdges,
    std::shared_ptr<std::map<std::string, Node>> lodNodes,
    const std::map<int, std::vector<std::string>> &regionNodes,
    std::map<int, std::string> &regionRepresentatives, std::shared_ptr<std::vector<Edge>> allEdges,
    std::shared_ptr<std::map<std::string, Node>> allNodes) {

    static int lod1EdgeCounter = 10000; // LOD1边ID从10000开始，避免与LOD0冲突

    // 为LOD1实际节点生成直接连接
    std::map<std::string, std::string> originalToLOD1; // 映射原始ID到LOD1 ID
    for (const auto &nodePair : *lodNodes) {
        const std::string &lod1Id = nodePair.first;
        const Node &node = nodePair.second;
        if (!node.label.empty()) {
            originalToLOD1[node.label] = lod1Id; // 使用label（原始ID）建立映射
        }
    }

    // 查找LOD1节点间的实际边连接
    for (const auto &edge : *allEdges) {
        if (!edge.visible)
            continue;

        auto fromIt = originalToLOD1.find(edge.from);
        auto toIt = originalToLOD1.find(edge.to);

        if (fromIt != originalToLOD1.end() && toIt != originalToLOD1.end()) {
            // 这是LOD1节点间的边，添加到LOD1边集合
            Edge lod1Edge = edge;
            lod1Edge.id = std::to_string(lod1EdgeCounter++); // 使用数字ID
            lod1Edge.from = fromIt->second;                  // 使用新的LOD1节点ID
            lod1Edge.to = toIt->second;                      // 使用新的LOD1节点ID
            lod1Edge.maxHeight = 45000.0f;                   // 中等高度

            lodEdges->push_back(lod1Edge);
        }
    }
}

// 生成LOD2边（继承LOD1+详细连接）
void VIS4Earth::GraphRenderer::generateLOD2Edges(
    std::shared_ptr<std::vector<Edge>> lodEdges,
    std::shared_ptr<std::map<std::string, Node>> lodNodes,
    std::shared_ptr<std::vector<Edge>> allEdges,
    std::shared_ptr<std::map<std::string, Node>> allNodes) {

    static int lod2EdgeCounter = 20000; // LOD2边ID从20000开始，避免与LOD0/LOD1冲突

    // 第一步：继承LOD1的所有边
    if (lodEdgesData[1]) {
        for (const auto &lod1Edge : *lodEdgesData[1]) {
            // 直接继承LOD1的边，保持原始ID和节点连接
            lodEdges->push_back(lod1Edge);
        }
        std::cout << "Inherited " << lodEdgesData[1]->size() << " edges from LOD1" << std::endl;
    }

    // 第二步：建立原始ID到LOD2 ID的映射
    std::map<std::string, std::string> originalToLOD2; // 映射原始ID到LOD2 ID
    for (const auto &nodePair : *lodNodes) {
        const std::string &lod2Id = nodePair.first;
        const Node &node = nodePair.second;
        if (!node.label.empty()) {
            originalToLOD2[node.label] = lod2Id; // 使用label（原始ID）建立映射
        }
    }

    // 第三步：添加涉及LOD2新增节点的边
    for (const auto &edge : *allEdges) {
        if (!edge.visible)
            continue;

        auto fromIt = originalToLOD2.find(edge.from);
        auto toIt = originalToLOD2.find(edge.to);

        if (fromIt != originalToLOD2.end() && toIt != originalToLOD2.end()) {
            Edge lod2Edge = edge;
            lod2Edge.id = std::to_string(lod2EdgeCounter++); // 使用数字ID
            lod2Edge.from = fromIt->second;                  // 使用新的LOD2节点ID
            lod2Edge.to = toIt->second;                      // 使用新的LOD2节点ID
            lod2Edge.maxHeight = 30000.0f;                   // 较低高度

            lodEdges->push_back(lod2Edge);
        }
    }

    std::cout << "Added " << (lodEdges->size() - (lodEdgesData[1] ? lodEdgesData[1]->size() : 0))
              << " new edges for LOD2" << std::endl;
}

// 生成基于地理分区的LOD数据（重新设计为基于节点level的简化版本）
void VIS4Earth::GraphRenderer::generateGeographicLODDataOld(
    int lodLevel, std::shared_ptr<std::map<std::string, Node>> allNodes,
    std::shared_ptr<std::vector<Edge>> allEdges) {

    auto lodNodes = std::make_shared<std::map<std::string, Node>>();
    auto lodEdges = std::make_shared<std::vector<Edge>>();

    std::cout << "Generating geographic LOD data for level " << lodLevel << std::endl;

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
        std::cout << "Invalid LOD level: " << lodLevel << std::endl;
        return;
    }

    std::cout << "Using " << currentRegions->size() << " regions for LOD " << lodLevel << std::endl;

    // 为每个区域分配节点
    std::map<int, std::vector<std::string>> regionNodes;
    for (const auto &nodePair : *allNodes) {
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

    std::cout << "Allocated nodes to " << regionNodes.size() << " regions" << std::endl;

    // 为每个有节点的区域创建代表节点
    std::map<int, std::string> regionRepresentatives;
    for (const auto &regionPair : regionNodes) {
        int regionId = regionPair.first;
        const std::vector<std::string> &nodeIds = regionPair.second;

        if (nodeIds.empty())
            continue;

        // 找到区域信息
        const GeographicRegion *regionInfo = nullptr;
        for (const auto &region : *currentRegions) {
            if (region.regionId == regionId) {
                regionInfo = &region;
                break;
            }
        }

        if (!regionInfo)
            continue;

        // 创建区域代表节点
        std::string repNodeId = "rep_" + std::to_string(regionId) + "_" + regionInfo->regionName;
        Node repNode;

        if (lodLevel == 0) {
            // LOD 0: 使用区域中心点
            repNode.pos = regionInfo->centerPoint;
        } else {
            // LOD 1/2: 选择区域内最重要的节点位置
            std::string bestNodeId =
                *std::max_element(nodeIds.begin(), nodeIds.end(),
                                  [allNodes](const std::string &a, const std::string &b) {
                                      const Node &nodeA = allNodes->at(a);
                                      const Node &nodeB = allNodes->at(b);
                                      if (nodeA.level != nodeB.level) {
                                          return nodeA.level > nodeB.level; // level越低优先级越高
                                      }
                                      return nodeA.degree < nodeB.degree; // 度数越高优先级越高
                                  });
            repNode.pos = allNodes->at(bestNodeId).pos;
        }

        repNode.id = repNodeId;
        repNode.color = osg::Vec3(1.0f, 0.8f, 0.0f); // 金色
        repNode.visible = true;
        repNode.isRepresent = true;
        repNode.level = 0;   // 代表节点level为0
        repNode.size = 2.0f; // 代表节点较大
        repNode.degree = static_cast<int>(nodeIds.size());
        repNode.cluster = regionId;
        repNode.label = regionInfo->regionName;

        (*lodNodes)[repNodeId] = repNode;
        regionRepresentatives[regionId] = repNodeId;

        std::cout << "Created representative node for region " << regionInfo->regionName << " with "
                  << nodeIds.size() << " nodes" << std::endl;
    }

    // 计算区域间连接并生成聚合边
    std::map<std::pair<int, int>, int> regionConnections;
    std::map<std::pair<int, int>, float> regionWeights;

    for (const auto &edge : *allEdges) {
        if (!edge.visible)
            continue;

        auto fromNodeIt = allNodes->find(edge.from);
        auto toNodeIt = allNodes->find(edge.to);
        if (fromNodeIt == allNodes->end() || toNodeIt == allNodes->end())
            continue;

        // 找到起点和终点节点所属的区域
        int fromRegion = -1, toRegion = -1;

        float fromLat = fromNodeIt->second.pos.x();
        float fromLon = fromNodeIt->second.pos.y();
        float toLat = toNodeIt->second.pos.x();
        float toLon = toNodeIt->second.pos.y();

        for (const auto &region : *currentRegions) {
            if (fromRegion == -1 && fromLat >= region.minLat && fromLat <= region.maxLat &&
                fromLon >= region.minLon && fromLon <= region.maxLon) {
                fromRegion = region.regionId;
            }
            if (toRegion == -1 && toLat >= region.minLat && toLat <= region.maxLat &&
                toLon >= region.minLon && toLon <= region.maxLon) {
                toRegion = region.regionId;
            }
            if (fromRegion != -1 && toRegion != -1)
                break;
        }

        if (fromRegion == -1 || toRegion == -1 || fromRegion == toRegion)
            continue;

        // 累计区域间连接
        std::pair<int, int> regionPair =
            std::make_pair(std::min(fromRegion, toRegion), std::max(fromRegion, toRegion));

        regionConnections[regionPair]++;
        regionWeights[regionPair] += edge.weight;
    }

    std::cout << "Found " << regionConnections.size() << " inter-region connections" << std::endl;

    // 生成聚合边
    for (const auto &connPair : regionConnections) {
        int region1 = connPair.first.first;
        int region2 = connPair.first.second;
        int connectionCount = connPair.second;
        float totalWeight = regionWeights[connPair.first];

        if (regionRepresentatives.find(region1) == regionRepresentatives.end() ||
            regionRepresentatives.find(region2) == regionRepresentatives.end()) {
            continue;
        }

        Edge aggregatedEdge;
        aggregatedEdge.id = "agg_" + std::to_string(region1) + "_" + std::to_string(region2);
        aggregatedEdge.from = regionRepresentatives[region1];
        aggregatedEdge.to = regionRepresentatives[region2];
        aggregatedEdge.weight = totalWeight;
        aggregatedEdge.visible = true;
        aggregatedEdge.maxHeight = 50000.0f + connectionCount * 10000.0f; // 根据连接数调整高度

        lodEdges->push_back(aggregatedEdge);

        std::cout << "Created aggregated edge between regions " << region1 << " and " << region2
                  << " with " << connectionCount << " connections, weight=" << totalWeight
                  << std::endl;
    }

    std::cout << "Generated " << lodNodes->size() << " representative nodes and "
              << lodEdges->size() << " aggregated edges for LOD " << lodLevel << std::endl;

    // 设置LOD数据
    lodNodesData[lodLevel] = lodNodes;
    lodEdgesData[lodLevel] = lodEdges;
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

osg::Vec3 VIS4Earth::GraphRenderer::latLonToWorldPos(double lat, double lon) {
    // 复用updateEdgeVBO中的vec3ToSphere逻辑（反向）
    float latRad = osg::DegreesToRadians(lat);
    float lonRad = osg::DegreesToRadians(lon);
    float h = osg::WGS_84_RADIUS_POLAR; // 地表高度

    osg::Vec3 worldPos;
    worldPos.z() = h * std::sin(latRad);
    h = h * std::cos(latRad);
    worldPos.y() = h * std::sin(lonRad);
    worldPos.x() = h * std::cos(lonRad);

    return worldPos;
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
        osg::Vec3 worldPos = latLonToWorldPos(it->second.pos.x(), it->second.pos.y());
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

// 调试函数实现
void VIS4Earth::GraphRenderer::debugEarthGridStatus() {
    std::cout << "=== Earth Grid Debug ===" << std::endl;
    std::cout << "Grid size: " << earthGrid.latitude_cells << " x " << earthGrid.longitude_cells
              << std::endl;

    int totalNodes = 0;
    int nonemptyGrids = 0;
    double minLat = 90.0, maxLat = -90.0, minLon = 180.0, maxLon = -180.0;

    for (int i = 0; i < earthGrid.latitude_cells; ++i) {
        for (int j = 0; j < earthGrid.longitude_cells; ++j) {
            int nodeCount = earthGrid.grid[i][j].node_ids.size();
            if (nodeCount > 0) {
                totalNodes += nodeCount;
                nonemptyGrids++;

                // 计算当前网格的地理边界
                double gridMinLat = (i * 180.0 / earthGrid.latitude_cells) - 90.0;
                double gridMaxLat = ((i + 1) * 180.0 / earthGrid.latitude_cells) - 90.0;
                double gridMinLon = (j * 360.0 / earthGrid.longitude_cells) - 180.0;
                double gridMaxLon = ((j + 1) * 360.0 / earthGrid.longitude_cells) - 180.0;

                minLat = std::min(minLat, gridMinLat);
                maxLat = std::max(maxLat, gridMaxLat);
                minLon = std::min(minLon, gridMinLon);
                maxLon = std::max(maxLon, gridMaxLon);

                if (nonemptyGrids <= 5) { // 只打印前5个非空网格的详细信息
                    std::cout << "Grid[" << i << "][" << j << "]: " << nodeCount
                              << " nodes, bounds: lat[" << gridMinLat << ", " << gridMaxLat
                              << "], lon[" << gridMinLon << ", " << gridMaxLon << "]" << std::endl;
                }
            }
        }
    }

    std::cout << "Total nodes in grid: " << totalNodes << std::endl;
    std::cout << "Non-empty grids: " << nonemptyGrids << "/"
              << (earthGrid.latitude_cells * earthGrid.longitude_cells) << std::endl;
    if (nonemptyGrids > 0) {
        std::cout << "Data coverage: lat[" << minLat << ", " << maxLat << "], lon[" << minLon
                  << ", " << maxLon << "]" << std::endl;
    }
    std::cout << "======================" << std::endl;
}

void VIS4Earth::GraphRenderer::debugNodeCoordinates(
    std::shared_ptr<std::map<std::string, Node>> nodes, int maxSamples) {
    if (!nodes || nodes->empty()) {
        std::cout << "=== Node Coordinates Debug: NO DATA ===" << std::endl;
        return;
    }

    std::cout << "=== Node Coordinates Debug ===" << std::endl;
    std::cout << "Total nodes: " << nodes->size() << std::endl;

    double minLat = 90.0, maxLat = -90.0, minLon = 180.0, maxLon = -180.0;
    int sampleCount = 0;

    for (const auto &nodePair : *nodes) {
        const Node &node = nodePair.second;
        double lat = node.pos.x();
        double lon = node.pos.y();

        minLat = std::min(minLat, lat);
        maxLat = std::max(maxLat, lat);
        minLon = std::min(minLon, lon);
        maxLon = std::max(maxLon, lon);

        if (sampleCount < maxSamples) {
            std::cout << "Node[" << node.id << "]: lat=" << lat << ", lon=" << lon
                      << ", level=" << node.level << std::endl;
            sampleCount++;
        }
    }

    std::cout << "Coordinate range: lat[" << minLat << ", " << maxLat << "], lon[" << minLon << ", "
              << maxLon << "]" << std::endl;
    std::cout << "=============================" << std::endl;
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

// 生成基于地理分区的LOD数据（重新设计为基于节点level的简化版本）

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

// 更新边的VBO数据

osg::Program *createTextureFlowShaderProgram() {
    const char *vertexShaderSource = R"(
        #version 120
        attribute vec3 vertexPosition; // Vertex position in model space
        attribute float lineID;       // Unique ID for each line segment

        varying vec3 v_LineStart;     // Start position of the current line
        varying vec3 v_LineEnd;       // End position of the current line
        varying vec3 v_Position;      // Vertex position to be passed to fragment shader

        uniform sampler2D uLineDataTex; // Texture containing line start/end data
        uniform float uTotalLines;    // Total number of lines for texture coordinate calculation

        void main()
        {
            v_Position = vertexPosition;
            // Calculate texture X coordinate to fetch line start/end from uLineDataTex
            float texX = (lineID + 0.5) / uTotalLines; 
            v_LineStart = texture2D(uLineDataTex, vec2(texX, 0.25)).rgb;
            v_LineEnd = texture2D(uLineDataTex, vec2(texX, 0.5)).rgb;

            // Standard model-view-projection transformation
            gl_Position = gl_ModelViewProjectionMatrix * vec4(vertexPosition, 1.0);
        }
    )";

    const char *fragmentShaderSource = R"(
        #version 120
        uniform sampler2D baseTexture; // The flowing arrow texture (on texture unit 1)
        uniform float u_time;          // Time uniform for animation

        varying vec3 v_Position;       // Current fragment's position
        varying vec3 v_LineStart;      // Start position of the current line
        varying vec3 v_LineEnd;        // End position of the current line

        void main()
        {
            if (length(v_LineEnd - v_LineStart) < 0.0001) { // Avoid division by zero for zero-length lines
                 discard;
            }
            vec3 lineDir = normalize(v_LineEnd - v_LineStart);
            float lineLength = length(v_LineEnd - v_LineStart);
            
            // Project current fragment position onto the line to get 't' [0,1]
            float t = dot(v_Position - v_LineStart, lineDir) / lineLength;
            t = clamp(t, 0.0, 1.0); // Ensure t is within [0,1]

            // Animate texture coordinate along the line
            // The texture S coordinate scrolls with time.
            // V coordinate is 0.5 for using the middle of the texture.
            vec2 texCoord = vec2(t * 2.0 + u_time, 0.5); // t*2.0 to make texture repeat more often along the line if desired, adjust as needed

            gl_FragColor = texture2D(baseTexture, texCoord);
            // For potentially better alpha blending with existing glow:
            // vec4 texColor = texture2D(baseTexture, texCoord);
            // gl_FragColor = vec4(texColor.rgb, texColor.a * 0.7); // Modulate alpha
        }
    )";

    osg::ref_ptr<osg::Program> program = new osg::Program;
    program->addBindAttribLocation("vertexPosition", 2);
    program->addBindAttribLocation("lineID", 1); // Make sure lineID is bound
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vertexShaderSource));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentShaderSource));
    return program.release();
}

// Insert TextureFlowAnimationCallback class definition here

class TextureFlowAnimationCallback : public osg::NodeCallback {
  public:
    TextureFlowAnimationCallback(osg::Uniform *timeUniform)
        : _timeUniform(timeUniform), _startTime(-1.0), _pausedTime(0.0), _isPaused(false) {}

    virtual void operator()(osg::Node *node, osg::NodeVisitor *nv) override {
        if (!_timeUniform || !nv || !nv->getFrameStamp()) {
            traverse(node, nv);
            return;
        }

        double currentTime = nv->getFrameStamp()->getSimulationTime();

        if (_isPaused) {
            traverse(node, nv);
            return;
        }

        if (_startTime < 0.0) {
            _startTime = currentTime;
        }

        float timeVal =
            static_cast<float>((currentTime - _startTime) * 0.2f); // Animation speed factor
        _timeUniform->set(timeVal);

        traverse(node, nv);
    }

    void pause() {
        if (!_isPaused) {
            _isPaused = true;
            double currentTime = osg::Timer::instance()->time_s();
            if (_startTime >= 0.0) {
                _pausedTime = currentTime - _startTime;
            } else {
                _pausedTime = 0.0;
            }
        }
    }

    void resume() {
        if (_isPaused) {
            _isPaused = false;
            double currentTime = osg::Timer::instance()->time_s();
            _startTime = currentTime - _pausedTime;
        }
    }

    void reset() {
        _startTime = -1.0;
        _pausedTime = 0.0;
        _isPaused = false; // Ensure it's not stuck in paused state if reset is called externally
        if (_timeUniform.valid())
            _timeUniform->set(0.0f);
    }

  private:
    osg::ref_ptr<osg::Uniform> _timeUniform;
    double _startTime;
    double _pausedTime;
    bool _isPaused;
};

void VIS4Earth::GraphRenderer::PerGraphParam::startTextureFlowAnimation() {
    if (isTextureFlowAnimating) {
        OSG_NOTIFY(osg::INFO) << "Stopping Texture Flow Animation." << std::endl;
        if (lineGeometry) {
            // Pause and remove our specific callback
            if (textureFlowCallback.valid()) {
                // Check if the current callback is ours before removing,
                // or simply remove all if this function manages the callback exclusively.
                if (lineGeometry->getUpdateCallback() == textureFlowCallback.get()) {
                    lineGeometry->setUpdateCallback(nullptr);
                }
                // We might want to keep the callback instance if we intend to resume it with its
                // previous state intact Or clear it: textureFlowCallback = nullptr;
            } else {
                // If no specific callback reference, just remove any existing one.
                lineGeometry->setUpdateCallback(nullptr);
            }

            osg::ref_ptr<osg::StateSet> currentSS = lineGeometry->getStateSet();
            if (currentSS.valid()) {
                osg::ref_ptr<osg::StateSet> newSS = new osg::StateSet(*currentSS);

                // Remove attributes specific to texture flow animation
                newSS->removeTextureAttribute(
                    1, osg::StateAttribute::TEXTURE); // baseTexture was on unit 1
                newSS->removeUniform("baseTexture");
                newSS->removeUniform("u_time");

                // Remove the shader program. This assumes this animation was the one that set it.
                // If multiple effects might be active, program management needs to be more
                // sophisticated.
                newSS->removeAttribute(osg::StateAttribute::PROGRAM);

                // Restore default blend state (typically OFF)
                newSS->setMode(GL_BLEND, osg::StateAttribute::OFF);
                osg::BlendFunc *defaultBlendFunc =
                    new osg::BlendFunc(); // Default (GL_ONE, GL_ZERO)
                newSS->setAttributeAndModes(defaultBlendFunc, osg::StateAttribute::OVERRIDE |
                                                                  osg::StateAttribute::ON);

                // Restore default depth state (typically write enabled, test LESS)
                osg::Depth *defaultDepth = new osg::Depth();
                defaultDepth->setWriteMask(true);
                defaultDepth->setFunction(osg::Depth::LESS);
                newSS->setAttributeAndModes(defaultDepth, osg::StateAttribute::OVERRIDE |
                                                              osg::StateAttribute::ON);

                newSS->setRenderingHint(osg::StateSet::DEFAULT_BIN);

                lineGeometry->setStateSet(newSS);
                lineGeometry->dirtyDisplayList();
            }
        }
        isTextureFlowAnimating = false;
    } else {
        OSG_NOTIFY(osg::INFO) << "开始纹理流动动画." << std::endl;

        if (lineGeode && lineGeometry) {
            // 创建线数据纹理（几何信息）
            lineDataImageForGeom = createLineDataTexture();
            osg::ref_ptr<osg::Texture2D> lineDataTex = new osg::Texture2D;
            lineDataTex->setImage(lineDataImageForGeom);
            lineDataTex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
            lineDataTex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);
            lineDataTex->setResizeNonPowerOfTwoHint(false);

            // 加载箭头流动纹理
            osg::ref_ptr<osg::Image> arrowTextureImage =
                osgDB::readImageFile("D:/A-my-work/vis-qt-osg/vis-on-earth-qt-osg-master-ui/"
                                     "bug-fix/improved_arrow_texture.png");
            if (!arrowTextureImage.valid()) {
                OSG_NOTIFY(osg::WARN)
                    << "无法加载纹理文件: improved_arrow_texture.png" << std::endl;
                isTextureFlowAnimating = false;
                return;
            }

            osg::ref_ptr<osg::Texture2D> arrowTexture = new osg::Texture2D;
            arrowTexture->setImage(arrowTextureImage);
            arrowTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
            arrowTexture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
            arrowTexture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
            arrowTexture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);

            // 获取状态集
            osg::ref_ptr<osg::StateSet> stateSet = lineGeometry->getOrCreateStateSet();

            // 设置着色器程序
            stateSet->setAttributeAndModes(createTextureFlowShaderProgram(),
                                           osg::StateAttribute::ON);

            // 绑定纹理单元
            stateSet->setTextureAttributeAndModes(0, lineDataTex,
                                                  osg::StateAttribute::ON); // 线几何数据纹理
            stateSet->setTextureAttributeAndModes(1, arrowTexture,
                                                  osg::StateAttribute::ON); // 箭头流动纹理

            // 设置uniform变量
            stateSet->addUniform(new osg::Uniform("uLineDataTex", 0));
            stateSet->addUniform(new osg::Uniform("baseTexture", 1));
            stateSet->addUniform(
                new osg::Uniform("uTotalLines", static_cast<float>(edges->size())));

            // 创建时间uniform
            osg::ref_ptr<osg::Uniform> timeUniform = new osg::Uniform("u_time", 0.0f);
            stateSet->addUniform(timeUniform);

            // 设置透明混合
            stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
            osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc();
            blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            stateSet->setAttributeAndModes(blendFunc, osg::StateAttribute::ON);
            stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

            // 设置深度测试
            osg::ref_ptr<osg::Depth> depth = new osg::Depth();
            depth->setWriteMask(false); // 禁用深度写入但保留深度测试
            stateSet->setAttributeAndModes(depth, osg::StateAttribute::ON);

            // 创建动画回调
            textureFlowCallback = new TextureFlowAnimationCallback(timeUniform);
            lineGeometry->setUpdateCallback(textureFlowCallback);

            OSG_NOTIFY(osg::INFO) << "纹理流动动画设置完成." << std::endl;
        }

        isTextureFlowAnimating = true;
    }
}