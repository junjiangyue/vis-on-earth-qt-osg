// #include <array>
// #include <memory>
//
// #include <osgGA/TrackballManipulator>
// #include <osgViewer/Viewer>
//
// #include <vis4earth/osg_util.h>
//
// #include <vis4earth/io/tf_io.h>
// #include <vis4earth/io/tf_osg_io.h>
// #include <vis4earth/io/vol_io.h>
// #include <vis4earth/io/vol_osg_io.h>
//
// #include <vis4earth/info_viser/line_chart.h>
//// D:/Project/OSG/vis-on-earth-qt-osg/data
// static const std::string volPath = DATA_PATH_PREFIX "OSS/OSS000.raw";
//// static const std::string volDPath0 = "C:/Code/bin/data/vis-osg-scatter-plot/linechart0.txt";
// static const std::string volDPath0 = DATA_PATH_PREFIX "linechart0.txt";
// static const std::string volDPath1 = DATA_PATH_PREFIX "linechart1.txt";
// static const std::string volDPath2 = DATA_PATH_PREFIX "linechart2.txt";
// static const std::string volDPath3 = DATA_PATH_PREFIX "linechart3.txt";
// static const std::string volName0 = "0";
// static const std::string volName1 = "1";
// static const std::string volName2 = "2";
// static const std::string volName3 = "3";
//
// static const std::array<uint32_t, 3> dim = {300, 350, 50};
// static const std::array<uint32_t, 3> graphDim = {100, 100, 50};
// static const std::array<int32_t, 3> coordinateDimMax = {3, 3, 2};
// static const std::array<int32_t, 3> coordinateDimMin = {-3, -3, -2};
// static const std::array<uint8_t, 3> log2Dim = {9, 9, 6};
// static const std::array<float, 2> lonRng = {100.05f, 129.95f};
// static const std::array<float, 2> latRng = {-4.95f, 29.95};
//// static const std::array<float, 2> hRng = { 1.f, 5316.f };
// static const std::array<float, 2> hRng = {1.f, 21264.f};
// static const float hScale = 100.f;
//
// int main(int argc, char **argv) {
//     auto *viewer = new osgViewer::Viewer;
//     viewer->setUpViewInWindow(200, 50, 800, 600);
//
//     auto *manipulator = new osgGA::TrackballManipulator;
//     viewer->setCameraManipulator(manipulator);
//
//     osg::ref_ptr<osg::Group> grp = new osg::Group;
//     grp->addChild(VIS4Earth::CreateEarth());
//
//     std::shared_ptr<VIS4Earth::InfoViser::LineChart> mcb =
//         std::make_shared<VIS4Earth::InfoViser::LineChart>();
//     std::string errMsg;
//     {
//         auto volDiscreteDat0 = VIS4Earth::Loader::TXTVolume::LoadFromFile(volDPath0);
//         if (!errMsg.empty())
//             goto ERR;
//         auto volDatShrd0 = std::make_shared<std::vector<osg::Vec3f>>(volDiscreteDat0);
//         mcb->AddDiscreteData(volName0, volDatShrd0);
//         auto vol0 = mcb->GetData(volName0);
//         vol0->SetLongtituteRange(lonRng[0], lonRng[1]);
//         vol0->SetLatituteRange(latRng[0], latRng[1]);
//         vol0->SetHeightFromCenterRange(.7f, .75f);
//         vol0->SetHeightFromCenterRange(
//             static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[0],
//             static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[1]);
//         // std::vector<osg::Vec3f> point = vol->GetDiscreteVec(coordinateDimMax,
//         coordinateDimMin);
//         // vol->DrawPlot(point);
//         // vol->DrawPlot(volDiscreteDat);
//
//         grp->addChild(vol0->MakeLineChart());
//         grp->addChild(vol0->MakeCoordinate());
//
//         auto volDiscreteDat1 = VIS4Earth::Loader::TXTVolume::LoadFromFile(volDPath1);
//         if (!errMsg.empty())
//             goto ERR;
//         auto volDatShrd1 = std::make_shared<std::vector<osg::Vec3f>>(volDiscreteDat1);
//         mcb->AddDiscreteData(volName1, volDatShrd1);
//         auto vol1 = mcb->GetData(volName1);
//         vol1->SetLongtituteRange(lonRng[0], lonRng[1]);
//         vol1->SetLatituteRange(latRng[0], latRng[1]);
//         vol1->SetHeightFromCenterRange(.7f, .75f);
//         vol1->SetHeightFromCenterRange(
//             static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[0],
//             static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[1]);
//         // std::vector<osg::Vec3f> point = vol->GetDiscreteVec(coordinateDimMax,
//         coordinateDimMin);
//         // vol->DrawPlot(point);
//         // vol->DrawPlot(volDiscreteDat);
//
//         grp->addChild(vol1->MakeLineChart(osg::Vec4f(0.6, 0.3, 0.5, 1.0)));
//
//         auto volDiscreteDat2 = VIS4Earth::Loader::TXTVolume::LoadFromFile(volDPath2);
//         if (!errMsg.empty())
//             goto ERR;
//         auto volDatShrd2 = std::make_shared<std::vector<osg::Vec3f>>(volDiscreteDat2);
//         mcb->AddDiscreteData(volName2, volDatShrd2);
//         auto vol2 = mcb->GetData(volName2);
//         vol2->SetLongtituteRange(lonRng[0], lonRng[1]);
//         vol2->SetLatituteRange(latRng[0], latRng[1]);
//         vol2->SetHeightFromCenterRange(.7f, .75f);
//         vol2->SetHeightFromCenterRange(
//             static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[0],
//             static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[1]);
//         // std::vector<osg::Vec3f> point = vol->GetDiscreteVec(coordinateDimMax,
//         coordinateDimMin);
//         // vol->DrawPlot(point);
//         // vol->DrawPlot(volDiscreteDat);
//
//         grp->addChild(vol2->MakeLineChart(osg::Vec4f(0.3, 0.6, 0.5, 1.0)));
//
//         auto volDiscreteDat3 = VIS4Earth::Loader::TXTVolume::LoadFromFile(volDPath3);
//         if (!errMsg.empty())
//             goto ERR;
//         auto volDatShrd3 = std::make_shared<std::vector<osg::Vec3f>>(volDiscreteDat3);
//         mcb->AddDiscreteData(volName3, volDatShrd3);
//         auto vol3 = mcb->GetData(volName3);
//         vol3->SetLongtituteRange(lonRng[0], lonRng[1]);
//         vol3->SetLatituteRange(latRng[0], latRng[1]);
//         vol3->SetHeightFromCenterRange(.7f, .75f);
//         vol3->SetHeightFromCenterRange(
//             static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[0],
//             static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[1]);
//         // std::vector<osg::Vec3f> point = vol->GetDiscreteVec(coordinateDimMax,
//         coordinateDimMin);
//         // vol->DrawPlot(point);
//         // vol->DrawPlot(volDiscreteDat);
//
//         grp->addChild(vol3->MakeLineChart(osg::Vec4f(0.3, 0.4, 0.7, 1.0)));
//     }
//
//     grp->addChild(mcb->GetGroup());
//
//     viewer->setSceneData(grp);
//
//     auto prevClk = clock();
//     while (!viewer->done()) {
//         auto currClk = clock();
//         auto duration = currClk - prevClk;
//
//         if (duration >= CLOCKS_PER_SEC / 45) {
//             viewer->frame();
//             prevClk = clock();
//         }
//     }
//     return 0;
//
// ERR:
//     std::cerr << errMsg << std::endl;
//     return 1;
//
// }
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/NodeCallback>
#include <osg/Program>
#include <osg/Shader>
#include <osg/Uniform>
#include <osg/Vec3>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
// 线条数据
struct LineSegment {
    osg::Vec3 start;
    osg::Vec3 end;
    float highlightPos; // 0.0到1.0之间的高光位置
    float speed;
};

// 高光动画回调类
class MultiHighlightAnimationCallback : public osg::NodeCallback {
  public:
    MultiHighlightAnimationCallback(const std::vector<LineSegment> &lines) : _lines(lines) {}

    virtual void operator()(osg::Node *node, osg::NodeVisitor *nv) {
        // 获取上一帧的时间
        static double lastTime = nv->getFrameStamp()->getSimulationTime();
        double currentTime = nv->getFrameStamp()->getSimulationTime();
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        // 更新所有线条的高光位置
        for (size_t i = 0; i < _lines.size(); ++i) {
            _lines[i].highlightPos += _lines[i].speed * deltaTime;

            // 到达终点后回到起点
            if (_lines[i].highlightPos >= 1.0f) {
                _lines[i].highlightPos = 0.0f;
            }
        }

        // 更新着色器uniform数组
        osg::StateSet *ss = node->getOrCreateStateSet();
        osg::Uniform *highlightPosUniform = ss->getUniform("uHighlightPos");
        if (highlightPosUniform) {
            std::vector<float> highlightPositions;
            for (const auto &line : _lines) {
                highlightPositions.push_back(line.highlightPos);
            }
            // highlightPosUniform->setArray(highlightPositions);
            for (size_t i = 0; i < highlightPositions.size(); ++i) {
                highlightPosUniform->setElement(i, highlightPositions[i]);
            }
        }

        // 继续遍历
        traverse(node, nv);
    }

  private:
    std::vector<LineSegment> _lines;
};

// 创建着色器程序
osg::Program *createMultiHighlightShaderProgram(int lineCount) {
    std::string vertSource = R"(
#version 120
attribute float lineID;
varying vec3 vPosition;
varying float vLineID;
uniform vec3 uLineStarts[)" + std::to_string(lineCount) +
                             R"(];
uniform vec3 uLineEnds[)" + std::to_string(lineCount) +
                             R"(];

void main()
{
    vPosition = gl_Vertex.xyz;
    vLineID = lineID;
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
)";

    std::string fragSource = R"(
#version 120
uniform float uHighlightPos[)" +
                             std::to_string(lineCount) + R"(];
uniform float uHighlightWidth;
uniform vec4 uHighlightColor;
varying vec3 vPosition;
varying float vLineID;
uniform vec3 uLineStarts[)" + std::to_string(lineCount) +
                             R"(];
uniform vec3 uLineEnds[)" + std::to_string(lineCount) +
                             R"(];

void main()
{
    int id = int(vLineID + 0.5);
    vec3 lineStart = uLineStarts[id];
    vec3 lineEnd = uLineEnds[id];
    
    // 基础颜色
    vec4 baseColor = gl_Color;
    
    // 计算当前点在直线上的投影位置
    vec3 lineVec = lineEnd - lineStart;
    vec3 pointVec = vPosition - lineStart;
    float t = dot(pointVec, lineVec) / dot(lineVec, lineVec);
    t = clamp(t, 0.0, 1.0);
    
    // 计算高光强度
    float distToHighlight = abs(t - uHighlightPos[id]);
    float highlightIntensity = exp(-distToHighlight * distToHighlight / (uHighlightWidth * uHighlightWidth));
    
    // 混合颜色
    vec4 finalColor = mix(baseColor, uHighlightColor, highlightIntensity);
    
    gl_FragColor = finalColor;
}
)";

    osg::ref_ptr<osg::Program> program = new osg::Program;
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));

    return program.release();
}

// 创建包含多条线的Geometry
osg::Geometry *createMultiLineGeometry(const std::vector<LineSegment> &lines) {
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

    // 创建顶点数组和线ID数组
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::FloatArray> lineIDs = new osg::FloatArray;

    for (size_t i = 0; i < lines.size(); ++i) {
        vertices->push_back(lines[i].start);
        vertices->push_back((lines[i].start + lines[i].end) / 3);
        vertices->push_back((lines[i].start + lines[i].end) / 2);

        vertices->push_back(lines[i].end);
        lineIDs->push_back(static_cast<float>(i));
        lineIDs->push_back(static_cast<float>(i));
        lineIDs->push_back(static_cast<float>(i));
        lineIDs->push_back(static_cast<float>(i));
    }

    geom->setVertexArray(vertices);

    // 设置线ID属性
    geom->setVertexAttribArray(6, lineIDs, osg::Array::BIND_PER_VERTEX);
    geom->setVertexAttribBinding(6, osg::Geometry::BIND_PER_VERTEX);

    // 设置颜色
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(0.2f, 0.2f, 1.0f, 1.0f)); // 线条基础颜色
    geom->setColorArray(colors, osg::Array::BIND_OVERALL);
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));
    //// 添加图元 - 每2个顶点一条线
    // for (size_t i = 0; i < lines.size(); ++i) {
    //     geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, i * 2, 2));
    // }

    // 设置线宽
    osg::ref_ptr<osg::LineWidth> linewidth = new osg::LineWidth(2.0f);
    geom->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);

    return geom.release();
}

// 创建带有高光动画的多线条节点
osg::Node *createAnimatedMultiHighlightLines(const std::vector<LineSegment> &lines) {
    // 创建多线条几何体
    osg::ref_ptr<osg::Geometry> lineGeom = createMultiLineGeometry(lines);
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(lineGeom);

    // 创建并设置着色器程序
    osg::ref_ptr<osg::Program> program =
        createMultiHighlightShaderProgram(static_cast<int>(lines.size()));

    // 设置着色器uniform
    osg::StateSet *ss = geode->getOrCreateStateSet();
    ss->setAttributeAndModes(program, osg::StateAttribute::ON);

    // 设置高光位置数组
    std::vector<float> initialHighlightPos(lines.size(), 0.0f);
    osg::Uniform *highlightPosUniform =
        new osg::Uniform(osg::Uniform::FLOAT, "uHighlightPos", static_cast<int>(lines.size()));
    for (size_t i = 0; i < initialHighlightPos.size(); ++i) {
        highlightPosUniform->setElement(i, initialHighlightPos[i]);
    }
    ss->addUniform(highlightPosUniform);

    // 设置线条起点和终点数组
    std::vector<osg::Vec3> lineStarts, lineEnds;
    for (const auto &line : lines) {
        lineStarts.push_back(line.start);
        lineEnds.push_back(line.end);
    }

    osg::Uniform *lineStartsUniform =
        new osg::Uniform(osg::Uniform::FLOAT_VEC3, "uLineStarts", lines.size());
    for (size_t i = 0; i < lineStarts.size(); ++i) {
        lineStartsUniform->setElement(i, lineStarts[i]);
    }
    // lineStartsUniform->setArray(lineStarts);
    ss->addUniform(lineStartsUniform);

    osg::Uniform *lineEndsUniform =
        new osg::Uniform(osg::Uniform::FLOAT_VEC3, "uLineEnds", lines.size());
    for (size_t i = 0; i < lineEnds.size(); ++i) {
        lineEndsUniform->setElement(i, lineEnds[i]);
    }
    // lineEndsUniform->setArray(lineEnds);
    ss->addUniform(lineEndsUniform);

    ss->addUniform(new osg::Uniform("uHighlightWidth", 0.15f));
    ss->addUniform(new osg::Uniform("uHighlightColor", osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)));

    // 添加动画回调
    geode->setUpdateCallback(new MultiHighlightAnimationCallback(lines));

    return geode.release();
}

int main(int argc, char **argv) {
    // 创建20条线
    std::vector<LineSegment> lines;
    for (int i = 0; i < 20; ++i) {
        float y = static_cast<float>(i) * 0.5f - 5.0f;
        LineSegment line;
        line.start = osg::Vec3(-5.0f, y, 0.0f);
        line.end = osg::Vec3(5.0f, y, 0.0f);
        line.highlightPos = 0.0f;
        line.speed = 0.5f + static_cast<float>(i) * 0.05f; // 每条线速度不同
        lines.push_back(line);
    }

    // 创建带有动画效果的多线条
    osg::ref_ptr<osg::Node> animatedLines = createAnimatedMultiHighlightLines(lines);

    // 创建场景根节点
    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addChild(animatedLines);

    // 创建查看器并设置场景数据
    osgViewer::Viewer viewer;
    viewer.setSceneData(root);

    // 开始渲染循环
    return viewer.run();
}