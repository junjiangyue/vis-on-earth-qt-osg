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
#include <cmath>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PositionAttitudeTransform>
#include <osg/Program>
#include <osg/StateSet>
#include <osg/TexEnv>
#include <osg/Texture2D>
#include <osg/Uniform>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>

// 创建一条简单的正弦曲线几何体
osg::ref_ptr<osg::Geometry> createCurveGeometry() {
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec2Array> texcoords = new osg::Vec2Array;

    int count = 200;
    float length = 10.0f;
    for (int i = 0; i < count; ++i) {
        float t = float(i) / (count - 1);
        float x = t * length;
        float y = std::sin(t * osg::PI * 2.0f);
        vertices->push_back(osg::Vec3(x, y, 0.0f));
        texcoords->push_back(osg::Vec2(t * 5.0f, 0.5f)); // U坐标拉伸，V恒定
    }

    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
    geometry->setVertexArray(vertices);
    geometry->setTexCoordArray(0, texcoords);
    geometry->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, count));
    return geometry;
}

// 创建 Shader 程序
osg::ref_ptr<osg::Program> createShaderProgram() {
    const char *vertexShaderSource = R"(
         #version 120
         varying vec2 v_TexCoord;
         void main()
         {
             gl_Position = ftransform();
             v_TexCoord = gl_MultiTexCoord0.xy;
         }
     )";

    const char *fragmentShaderSource = R"(
         #version 120
         uniform sampler2D baseTexture;
         uniform float u_time;
         varying vec2 v_TexCoord;
         void main()
         {
             vec2 coord = v_TexCoord;
             coord.x += u_time;
             gl_FragColor = texture2D(baseTexture, coord);
         }
     )";

    osg::ref_ptr<osg::Program> program = new osg::Program;
    program->addShader(new osg::Shader(osg::Shader::VERTEX, vertexShaderSource));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentShaderSource));
    return program;
}
class TimeUpdateCallback : public osg::NodeCallback {
  public:
    TimeUpdateCallback(osg::Uniform *timeUniform) : _timeUniform(timeUniform), _startTime(-1.0) {}

    virtual void operator()(osg::Node *node, osg::NodeVisitor *nv) override {
        double current = nv->getFrameStamp()->getReferenceTime();
        if (_startTime < 0.0)
            _startTime = current; // 第一次记录

        float time = fmod((current - _startTime) * 0.5f, 1.0f);
        _timeUniform->set(time);
        traverse(node, nv);
    }

  private:
    osg::ref_ptr<osg::Uniform> _timeUniform;
    double _startTime;
};
int main() {
    osg::ref_ptr<osg::Geometry> curve = createCurveGeometry();
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(curve);

    osg::ref_ptr<osg::StateSet> stateSet = geode->getOrCreateStateSet();

    // 加载纹理
    osg::ref_ptr<osg::Image> image = osgDB::readImageFile(
        "D:/A-my-work/vis-qt-osg/vis-on-earth-qt-osg-master-ui/bug-fix/improved_arrow_texture.png");
    osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D(image);
    texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP);
    texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
    texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    stateSet->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);

    // 设置混合（支持透明）
    stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    // 设置 Shader 程序
    osg::ref_ptr<osg::Program> program = createShaderProgram();
    stateSet->setAttributeAndModes(program, osg::StateAttribute::ON);

    // 设置时间 Uniform
    osg::ref_ptr<osg::Uniform> timeUniform = new osg::Uniform("u_time", 0.0f);
    stateSet->addUniform(timeUniform);

    // Viewer 和更新回调
    auto *viewer = new osgViewer::Viewer;
    viewer->setUpViewInWindow(200, 50, 1000, 1000);

    geode->addUpdateCallback(new TimeUpdateCallback(timeUniform.get()));
    viewer->setSceneData(geode);

    return viewer->run();
}
//
// #include <cmath>
// #include <osg/BlendFunc>
// #include <osg/Geode>
// #include <osg/Geometry>
// #include <osg/Program>
// #include <osg/StateSet>
// #include <osg/Texture2D>
// #include <osg/Uniform>
// #include <osgDB/ReadFile>
// #include <osgViewer/Viewer>
//
//// 构建带宽度的带状面片（triangle strip）
// osg::ref_ptr<osg::Geometry> createArrowRibbon() {
//     osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
//     osg::ref_ptr<osg::Vec2Array> texcoords = new osg::Vec2Array;
//
//     int count = 100;
//     float length = 10.0f;
//     float halfWidth = 0.1f;
//
//     for (int i = 0; i < count; ++i) {
//         float t = static_cast<float>(i) / (count - 1);
//         float x = t * length;
//         float y = std::sin(t * osg::PI * 2.0f);
//
//         osg::Vec3 center(x, y, 0.0f);
//         osg::Vec3 normal(0.0f, 0.0f, 1.0f); // 假设 z 朝上
//         osg::Vec3 tangent(1.0f, std::cos(t * osg::PI * 2.0f) * osg::PI * 2.0f, 0.0f);
//         osg::Vec3 binormal = normal ^ tangent; // 叉乘求垂线
//         binormal.normalize();
//
//         osg::Vec3 left = center - binormal * halfWidth;
//         osg::Vec3 right = center + binormal * halfWidth;
//
//         vertices->push_back(left);
//         texcoords->push_back(osg::Vec2(t * 5.0f, 0.0f));
//
//         vertices->push_back(right);
//         texcoords->push_back(osg::Vec2(t * 5.0f, 1.0f));
//     }
//
//     osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
//     geom->setVertexArray(vertices);
//     geom->setTexCoordArray(0, texcoords);
//     geom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLE_STRIP, 0, vertices->size()));
//     return geom;
// }
//
// osg::ref_ptr<osg::Program> createShaderProgram() {
//     const char *vert = R"(
//         #version 120
//         varying vec2 v_TexCoord;
//         void main() {
//             gl_Position = ftransform();
//             v_TexCoord = gl_MultiTexCoord0.xy;
//         }
//     )";
//
//     const char *frag = R"(
//         #version 120
//         uniform sampler2D baseTexture;
//         uniform float u_time;
//         varying vec2 v_TexCoord;
//         void main() {
//             vec2 coord = v_TexCoord;
//             coord.x += u_time;
//             gl_FragColor = texture2D(baseTexture, coord);
//         }
//     )";
//
//     osg::ref_ptr<osg::Program> prog = new osg::Program;
//     prog->addShader(new osg::Shader(osg::Shader::VERTEX, vert));
//     prog->addShader(new osg::Shader(osg::Shader::FRAGMENT, frag));
//     return prog;
// }
//
// class AnimateCallback : public osg::NodeCallback {
//   public:
//     AnimateCallback(osg::Uniform *timeUniform) : _uniform(timeUniform), _start(-1.0) {}
//
//     void operator()(osg::Node *node, osg::NodeVisitor *nv) override {
//         if (!nv->getFrameStamp())
//             return;
//         double now = nv->getFrameStamp()->getSimulationTime();
//         if (_start < 0.0)
//             _start = now;
//         float time = fmod((now - _start) * 0.5, 1.0);
//         _uniform->set(time);
//         traverse(node, nv);
//     }
//
//   private:
//     osg::ref_ptr<osg::Uniform> _uniform;
//     double _start;
// };
//
// int main() {
//     auto geom = createArrowRibbon();
//     auto geode = new osg::Geode;
//     geode->addDrawable(geom);
//
//     osg::StateSet *ss = geode->getOrCreateStateSet();
//
//     // 加载纹理
//     auto img = osgDB::readImageFile(
//         "D:/A-my-work/vis-qt-osg/vis-on-earth-qt-osg-master-ui/bug-fix/improved_arrow_texture.png");
//     auto tex = new osg::Texture2D(img);
//     tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
//     tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP);
//     tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
//     tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
//     ss->setTextureAttributeAndModes(0, tex, osg::StateAttribute::ON);
//
//     // 设置透明混合
//     ss->setMode(GL_BLEND, osg::StateAttribute::ON);
//     ss->setAttributeAndModes(new osg::BlendFunc, osg::StateAttribute::ON);
//     ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
//
//     // Shader 和时间控制
//     auto prog = createShaderProgram();
//     ss->setAttributeAndModes(prog, osg::StateAttribute::ON);
//     auto u_time = new osg::Uniform("u_time", 0.0f);
//     ss->addUniform(u_time);
//
//     // 回调更新动画
//     geode->addUpdateCallback(new AnimateCallback(u_time));
//
//     osgViewer::Viewer viewer;
//     viewer.setUpViewInWindow(200, 50, 1000, 1000);
//     viewer.setSceneData(geode);
//     return viewer.run();
// }