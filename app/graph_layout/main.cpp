#include <iostream>

#include <QtWidgets/QApplication>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>

#include <osgGA/TrackballManipulator>

#include <osgGA/TerrainManipulator>
#include <osgViewer/Viewer>

#include <vis4earth/graph_viser/NodeClickHandler.h>
#include <vis4earth/graph_viser/graph_display.h>
#include <vis4earth/graph_viser/graph_draw.h>
#include <vis4earth/graph_viser/markManager.h>
#include <vis4earth/graph_viser/nodeHoverHandler.h>

#include <osgViewer/ViewerEventHandlers>


int main(int argc, char **argv) {
    QApplication app(argc, argv);

    auto *viewer = new osgViewer::Viewer;
    viewer->setUpViewInWindow(200, 50, 1000, 1000);
    osg::ref_ptr<osgGA::TerrainManipulator> terrainManipulator = new osgGA::TerrainManipulator();

    // 2. 将它设置给 Viewer
    viewer->setCameraManipulator(terrainManipulator);
    //auto *manipulator = new osgGA::TrackballManipulator;
    //viewer->setCameraManipulator(manipulator);

    osg::ref_ptr<osg::Group> grp = new osg::Group;
    grp->addChild(VIS4Earth::CreateEarth());

    VIS4Earth::GraphRenderer *graphLayout = new VIS4Earth::GraphRenderer;
    // 设置camera
    graphLayout->param.setCamera(viewer->getCamera());

    grp->addChild(graphLayout->getGroup());
    graphLayout->show();

    osg::ref_ptr<VIS4Earth::CameraMovementCallback> cb =
        new VIS4Earth::CameraMovementCallback(graphLayout);
    viewer->getCamera()->addUpdateCallback(cb);
    //  添加点击事件
    VIS4Earth::NodeClickHandler *nodeClickHandler =
        new VIS4Earth::NodeClickHandler(graphLayout, viewer);
    // 将 NodeClickHandler 添加到 Viewer 的事件处理器中
    viewer->addEventHandler(nodeClickHandler);

    auto pStatsEventHandler = new osgViewer::StatsHandler; // 构造一视景器统计事件处理器
    viewer->addEventHandler(pStatsEventHandler); // 向视景器增加统计事件处理器

    viewer->setSceneData(grp);
    auto prevClk = clock();
    while (!viewer->done()) {
        auto currClk = clock();
        auto duration = currClk - prevClk;

        app.processEvents();
        viewer->frame();
        prevClk = clock();
        // if (duration >= CLOCKS_PER_SEC / 200) {
        //
        // }
    }

    return 0;
}