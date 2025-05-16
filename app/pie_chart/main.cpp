// #include <memory>
//
// #include <array>
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
// #include <vis4earth/info_viser/pie_chart.h>
//
//// static const std::string volPath = DATA_PATH_PREFIX"OSS/OSS000.raw";
//// static const std::string volDPath0 = "C:/Code/bin/data/vis-osg-scatter-plot/piechart0.txt";
//// static const std::string volDPath1 = "C:/Code/bin/data/vis-osg-scatter-plot/linechart1.txt";
//// static const std::string volDPath2 = "C:/Code/bin/data/vis-osg-scatter-plot/linechart2.txt";
//// static const std::string volDPath3 = "C:/Code/bin/data/vis-osg-scatter-plot/linechart3.txt";
// static const std::string volName0 = "0";
//// static const std::string volName1 = "1";
//// static const std::string volName2 = "2";
//// static const std::string volName3 = "3";
//
//// unused
// static const std::array<uint32_t, 3> dim = {300, 350, 50};
// static const std::array<uint32_t, 3> graphDim = {100, 100, 50};
// static const std::array<int32_t, 3> coordinateDimMax = {3, 3, 2};
// static const std::array<int32_t, 3> coordinateDimMin = {-3, -3, -2};
// static const std::array<uint8_t, 3> log2Dim = {9, 9, 6};
//
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
//     // 相机
//     auto *manipulator = new osgGA::TrackballManipulator;
//     viewer->setCameraManipulator(manipulator);
//
//     osg::ref_ptr<osg::Group> grp = new osg::Group;
//     grp->addChild(VIS4Earth::CreateEarth());
//
//     std::shared_ptr<VIS4Earth::InfoViser::PieChart> mcb =
//         std::make_shared<VIS4Earth::InfoViser::PieChart>();
//     std::string errMsg;
//     {
//         std::vector<std::pair<const wchar_t *, float>> pieDat0;
//         pieDat0.push_back(std::make_pair(L"第一季度", 0.2f));
//         pieDat0.push_back(std::make_pair(L"第二季度", 0.3f));
//         pieDat0.push_back(std::make_pair(L"第三季度", 0.4f));
//         pieDat0.push_back(std::make_pair(L"第四季度", 0.1f));
//         // pieDat0.push_back(std::make_pair(L"w", 0.15f));
//
//         if (!errMsg.empty())
//             goto ERR;
//         auto volDatShrd0 =
//             std::make_shared<std::vector<std::pair<const wchar_t *, float>>>(pieDat0);
//         mcb->AddPieData(volName0, volDatShrd0);
//         auto vol0 = mcb->GetData(volName0);
//         vol0->SetLongtituteRange(lonRng[0], lonRng[1]);
//         vol0->SetLatituteRange(latRng[0], latRng[1]);
//         vol0->SetHeightFromCenterRange(.7f, .75f);
//         vol0->SetHeightFromCenterRange(
//             static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[0],
//             static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[1]);
//
//         grp->addChild(vol0->MakePieChart());
//         // grp->addChild(vol0->MakeCoordinate());
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
// }
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/NodeCallback>
#include <osg/Program>
#include <osg/Shader>
#include <osg/Texture2D>
#include <osg/Vec3>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

// 线条数据结构
struct LineSegment {
    osg::Vec3 start;
    osg::Vec3 end;
    float highlightPos; // 0.0到1.0之间的高光位置
    float speed;
};

class TextureBasedAnimationCallback : public osg::NodeCallback {
  public:
    TextureBasedAnimationCallback(osg::Image *lineDataImage, const std::vector<LineSegment> &lines)
        : _lineDataImage(lineDataImage), _lines(lines), _firstFrame(true) {
        // 预分配足够大小的缓存
        _paramCache.resize(lines.size() * 4); // 每个线条4个float(RGBA)
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
        for (size_t i = 0; i < _lines.size(); ++i) {
            // 独立更新每条线的高光位置
            _lines[i].highlightPos =
                fmod(_lines[i].highlightPos + _lines[i].speed * deltaTime, 1.0f);

            int baseIdx = i * 4;
            _paramCache[baseIdx] = _lines[i].highlightPos;
            _paramCache[baseIdx + 1] = _lines[i].speed;
            _paramCache[baseIdx + 2] = 0.0f; // 保留
            _paramCache[baseIdx + 3] = 0.0f; // 保留
        }

        // 更新纹理（仅参数行）
        if (_lineDataImage.valid()) {
            float *data = reinterpret_cast<float *>(_lineDataImage->data());
            if (data) {
                const int rowStride = _lineDataImage->s() * 4;
                for (size_t i = 0; i < _lines.size(); ++i) {
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
    std::vector<LineSegment> _lines;
    std::vector<float> _paramCache; // 本地参数缓存
    bool _firstFrame = true;
};
// 创建着色器程序（纹理版本）
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

varying vec3 vPosition;
varying float vLineID;
varying vec3 vLineStart;
varying vec3 vLineEnd;

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
    //if(dist > uHighlightWidth) discard;
    
    // 高光强度计算（仅在前向移动方向增强）
    float highlightIntensity = 0.0;
    if(t >= highlightPos - uHighlightWidth && t <= highlightPos) {
        float falloff = 1.0 - smoothstep(highlightPos - uHighlightWidth, highlightPos, t);
        highlightIntensity = falloff * exp(-pow((highlightPos - t)/0.05, 2.0));
    }
    
    // 基础颜色
    vec4 baseColor = vec4(0.2, 0.2, 1.0, 1.0);
    
    // 最终颜色
    gl_FragColor = mix(baseColor, uHighlightColor, highlightIntensity);
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

// 创建线条几何体（每个线段3个顶点）
osg::Geometry *createMultiLineGeometry(const std::vector<LineSegment> &lines) {
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::FloatArray> lineIDs = new osg::FloatArray;

    for (size_t i = 0; i < lines.size(); ++i) {
        // 起点、中点、终点
        vertices->push_back(lines[i].start);
        vertices->push_back((lines[i].start + lines[i].end) * 0.5f);
        vertices->push_back(lines[i].end);

        lineIDs->push_back(static_cast<float>(i));
        lineIDs->push_back(static_cast<float>(i));
        lineIDs->push_back(static_cast<float>(i));
    }

    geom->setVertexArray(vertices);
    // 修改顶点属性设置
    geom->setVertexAttribArray(0, vertices, osg::Array::BIND_PER_VERTEX);
    geom->setVertexAttribArray(1, lineIDs, osg::Array::BIND_PER_VERTEX);

    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(0.2f, 0.2f, 1.0f, 1.0f));
    geom->setColorArray(colors, osg::Array::BIND_OVERALL);
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));
    // for (size_t i = 0; i < lines.size(); ++i) {
    //     geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, i * 3, 3));
    // }

    osg::ref_ptr<osg::LineWidth> linewidth = new osg::LineWidth(2.0f);
    geom->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);

    return geom.release();
}

osg::Image *createLineDataTexture(const std::vector<LineSegment> &lines) {
    int texWidth = lines.size();
    int texHeight = 4; // 使用4行存储不同参数

    osg::Image *image = new osg::Image;
    image->allocateImage(texWidth, texHeight, 1, GL_RGBA, GL_FLOAT);
    image->setInternalTextureFormat(GL_RGBA32F_ARB);

    // 初始填充0
    memset(image->data(), 0, texWidth * texHeight * 4 * sizeof(float));

    // 初始化静态数据(起点/终点)
    float *data = reinterpret_cast<float *>(image->data());
    for (int x = 0; x < texWidth; ++x) {
        // 第1行: 起点 (y=1)
        int startPos = (1 * texWidth + x) * 4;
        data[startPos] = lines[x].start.x();
        data[startPos + 1] = lines[x].start.y();
        data[startPos + 2] = lines[x].start.z();
        data[startPos + 3] = 1.0f;

        // 第2行: 终点 (y=2)
        int endPos = (2 * texWidth + x) * 4;
        data[endPos] = lines[x].end.x();
        data[endPos + 1] = lines[x].end.y();
        data[endPos + 2] = lines[x].end.z();
        data[endPos + 3] = 1.0f;
    }

    return image;
}

osg::Node *createTextureBasedAnimatedLines(const std::vector<LineSegment> &lines) {
    // 创建几何体
    osg::ref_ptr<osg::Geometry> geom = createMultiLineGeometry(lines);
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(geom);

    // 创建数据纹理
    osg::ref_ptr<osg::Image> lineDataImage = createLineDataTexture(lines);
    osg::ref_ptr<osg::Texture2D> lineDataTex = new osg::Texture2D;
    lineDataTex->setImage(lineDataImage);
    lineDataTex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
    lineDataTex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);
    lineDataTex->setResizeNonPowerOfTwoHint(false);

    // 设置着色器
    osg::StateSet *ss = geode->getOrCreateStateSet();
    ss->setAttributeAndModes(createTextureBasedShaderProgram(lines.size()),
                             osg::StateAttribute::ON);

    // 绑定纹理
    ss->setTextureAttributeAndModes(0, lineDataTex, osg::StateAttribute::ON);
    ss->addUniform(new osg::Uniform("uLineDataTex", 0));
    ss->addUniform(new osg::Uniform("uTotalLines", static_cast<float>(lines.size())));
    ss->addUniform(new osg::Uniform("uHighlightWidth", 0.15f));
    ss->addUniform(new osg::Uniform("uHighlightColor", osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)));

    // 设置回调
    geode->setUpdateCallback(new TextureBasedAnimationCallback(lineDataImage, lines));

    return geode.release();
}

int main(int argc, char **argv) {
    // 创建测试线条
    std::vector<LineSegment> lines;
    for (int i = 0; i < 10000; ++i) { // 测试1000条线
        float y = static_cast<float>(i) * 0.1f - 50.0f;
        LineSegment line;
        line.start = osg::Vec3(-50.0f, y, 0.0f);
        line.end = osg::Vec3(50.0f, y, 0.0f);
        line.highlightPos = 0.0f;
        // line.speed = 0.5f + static_cast<float>(i) * 0.002f;
        line.speed = 0.5f;
        lines.push_back(line);
    }

    // 创建场景
    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addChild(createTextureBasedAnimatedLines(lines));

    // 设置查看器
    osgViewer::Viewer viewer;
    auto pStatsEventHandler = new osgViewer::StatsHandler; // 构造一视景器统计事件处理器
    viewer.addEventHandler(pStatsEventHandler); // 向视景器增加统计事件处理器
    viewer.setSceneData(root);
    viewer.setUpViewInWindow(100, 100, 800, 600);

    return viewer.run();
}