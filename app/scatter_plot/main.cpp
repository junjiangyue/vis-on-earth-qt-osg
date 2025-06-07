//#include <memory>
//
//#include <array>
//
//#include <osgGA/TrackballManipulator>
//#include <osgViewer/Viewer>
//
//#include <vis4earth/osg_util.h>
//
//#include <vis4earth/io/tf_io.h>
//#include <vis4earth/io/tf_osg_io.h>
//#include <vis4earth/io/vol_io.h>
//#include <vis4earth/io/vol_osg_io.h>
//
//#include <vis4earth/info_viser/scatter_plot.h>
//
//static const std::string volPath = DATA_PATH_PREFIX "OSS/OSS000.raw";
//static const std::string volDPath = DATA_PATH_PREFIX "/linechart0.txt";
//static const std::string volName = "0";
//static const std::array<uint32_t, 3> dim = {300, 350, 50};
//static const std::array<uint32_t, 3> graphDim = {100, 100, 50};
//static const std::array<int32_t, 3> coordinateDimMax = {3, 3, 2};
//static const std::array<int32_t, 3> coordinateDimMin = {-3, -3, -2};
//static const std::array<uint8_t, 3> log2Dim = {9, 9, 6};
//static const std::array<float, 2> lonRng = {100.05f, 129.95f};
//static const std::array<float, 2> latRng = {-4.95f, 29.95};
//// static const std::array<float, 2> hRng = { 1.f, 5316.f };
//static const std::array<float, 2> hRng = {1.f, 21264.f};
//static const float hScale = 100.f;
//
//int main(int argc, char **argv) {
//    VIS4Earth::InfoViser::ScatterPlot p;
//    auto *viewer = new osgViewer::Viewer;
//    viewer->setUpViewInWindow(200, 50, 800, 600);
//
//    auto *manipulator = new osgGA::TrackballManipulator;
//    viewer->setCameraManipulator(manipulator);
//
//    osg::ref_ptr<osg::Group> grp = new osg::Group;
//    grp->addChild(VIS4Earth::CreateEarth());
//
//    std::shared_ptr<VIS4Earth::InfoViser::ScatterPlot> mcb =
//        std::make_shared<VIS4Earth::InfoViser::ScatterPlot>();
//    std::string errMsg;
//    {
//        auto volDiscreteDat = VIS4Earth::Loader::TXTVolume::LoadFromFile(volDPath);
//        if (!errMsg.empty())
//            goto ERR;
//        auto volDatShrd = std::make_shared<std::vector<osg::Vec3f>>(volDiscreteDat);
//        mcb->AddDiscreteVolume(volName, volDatShrd);
//        auto vol = mcb->GetVolume(volName);
//        vol->SetLongtituteRange(lonRng[0], lonRng[1]);
//        vol->SetLatituteRange(latRng[0], latRng[1]);
//        vol->SetHeightFromCenterRange(.7f, .75f);
//        vol->SetHeightFromCenterRange(
//            static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[0],
//            static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[1]);
//        std::vector<osg::Vec3f> point = vol->GetDiscreteVec(coordinateDimMax, coordinateDimMin);
//        vol->DrawPlot(point);
//        grp->addChild(vol->MakeCoordinate());
//    }
//
//    grp->addChild(mcb->GetGroup());
//
//    viewer->setSceneData(grp);
//
//    auto prevClk = clock();
//    while (!viewer->done()) {
//        auto currClk = clock();
//        auto duration = currClk - prevClk;
//
//        if (duration >= CLOCKS_PER_SEC / 45) {
//            viewer->frame();
//            prevClk = clock();
//        }
//    }
//    return 0;
//
//ERR:
//    std::cerr << errMsg << std::endl;
//    return 1;
//}
#include <cmath>
#include <fstream>
#include <map>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <sstream>
#include <vector>
#include <corecrt_math_defines.h>

// Constants
const double EARTH_RADIUS = 6371.0;
const double SATELLITE_HEIGHT = 35786.0;
const double DEG2RAD = M_PI / 180.0;

struct Node {
    int id;
    std::string name;
    double lat, lon;
    std::string level;
    std::string color;
    osg::Vec3d position;
};

osg::Vec3d latLonToXYZ(double lat, double lon, double height_km) {
    double radius = EARTH_RADIUS + height_km;
    double latRad = lat * DEG2RAD;
    double lonRad = lon * DEG2RAD;

    double x = radius * cos(latRad) * cos(lonRad);
    double y = radius * cos(latRad) * sin(lonRad);
    double z = radius * sin(latRad);
    return osg::Vec3d(x, y, z);
}

std::map<int, Node> loadNodes(const std::string &file) {
    std::ifstream in(file);
    std::string line;
    std::map<int, Node> nodes;

    getline(in, line); // Skip header
    while (getline(in, line)) {
        std::stringstream ss(line);
        Node node;
        std::string latStr, lonStr;
        std::string idStr;

        getline(ss, idStr, ',');
        node.id = std::stoi(idStr);
        getline(ss, node.name, ',');
        getline(ss, latStr, ',');
        node.lat = std::stod(latStr);
        getline(ss, lonStr, ',');
        node.lon = std::stod(lonStr);
        getline(ss, node.level, ',');
        getline(ss, node.color, ',');

        double height = (node.level == "satellite") ? SATELLITE_HEIGHT : 0.0;
        node.position = latLonToXYZ(node.lat, node.lon, height);
        nodes[node.id] = node;
    }
    return nodes;
}

std::vector<std::pair<osg::Vec3d, osg::Vec3d>> loadEdges(const std::string &file,
                                                         const std::map<int, Node> &nodes) {
    std::ifstream in(file);
    std::string line;
    std::vector<std::pair<osg::Vec3d, osg::Vec3d>> edges;
    getline(in, line); // skip header

    while (getline(in, line)) {
        std::stringstream ss(line);
        std::string fromStr, toStr, val;
        getline(ss, fromStr, ',');
        getline(ss, toStr, ',');
        getline(ss, val, ',');
        int from = std::stoi(fromStr);
        int to = std::stoi(toStr);
        if (nodes.count(from) && nodes.count(to)) {
            edges.push_back({nodes.at(from).position, nodes.at(to).position});
        }
    }
    return edges;
}

osg::ref_ptr<osg::Node> createEarth() {
    osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere(osg::Vec3(0, 0, 0), EARTH_RADIUS);
    osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(sphere);
    shape->setColor(osg::Vec4(0.2f, 0.4f, 0.8f, 1.0f));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(shape);
    return geode;
}

osg::ref_ptr<osg::Group> createNodeVisuals(const std::map<int, Node> &nodes, osg::Node *satModel) {
    osg::ref_ptr<osg::Group> group = new osg::Group;
    for (const auto &pair : nodes) {
        const Node &node = pair.second;
        osg::Vec3d pos = node.position;

        osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
        mt->setMatrix(osg::Matrix::translate(pos));

        if (node.level == "satellite") {

            // 设置缩放矩阵（把单位立方体放大成 500 米大小的卫星）
            osg::Matrix scale = osg::Matrix::scale(100.0f, 100.0f, 100.0f);

            // 设置平移矩阵（放到轨道位置）
            osg::Matrix translate = osg::Matrix::translate(pos);

            // 组合矩阵：先缩放，再平移
            mt->setMatrix(scale * translate);

            // 克隆模型节点（防止共享同一个状态树冲突）
            osg::ref_ptr<osg::Node> satClone =
                dynamic_cast<osg::Node *>(satModel->clone(osg::CopyOp::DEEP_COPY_ALL));

            // 关闭剔除：显示所有面，防止模型背面不显示
            satClone->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);

            // 添加到变换节点
            mt->addChild(satClone.get());
        } else {
            osg::ref_ptr<osg::Geode> dot = new osg::Geode;
            osg::ref_ptr<osg::ShapeDrawable> sd =
                new osg::ShapeDrawable(new osg::Sphere(pos, 200.0));
            sd->setColor(osg::Vec4(0.1f, 0.8f, 0.2f, 1.0f));
            dot->addDrawable(sd);
            mt->addChild(dot);
        }
        group->addChild(mt);
    }
    return group;
}

osg::ref_ptr<osg::Geode>
createEdgeLines(const std::vector<std::pair<osg::Vec3d, osg::Vec3d>> &edges) {
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    for (const auto &edge : edges) {
        vertices->push_back(edge.first);
        vertices->push_back(edge.second);
    }

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setVertexArray(vertices.get());
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, vertices->size()));
    geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(geom);
    return geode;
}

int main(int argc, char **argv) {
    auto nodes = loadNodes("C:/Users/shan/Desktop/graph_data/50-satellite/nodes.csv");
    auto edges = loadEdges("C:/Users/shan/Desktop/graph_data/50-satellite/edges.csv", nodes);
    //auto satModel = osgDB::readNodeFile("cessna.osg"); // OSG 内置模型
    auto satModel = osgDB::readNodeFile("C:/Users/shan/Desktop/graph_data/50-satellite/satellite_obj.obj");

    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addChild(createEarth());
    root->addChild(createNodeVisuals(nodes, satModel));
    root->addChild(createEdgeLines(edges));

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow(200, 50, 1000, 1000);
    viewer.setSceneData(root.get());
    return viewer.run();
}
