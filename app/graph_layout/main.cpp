#include <iostream>

#include <QtWidgets/QApplication>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>

#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

#include <vis4earth/graph_viser/graph_display.h>

int main(int argc, char **argv) {
    QApplication app(argc, argv);

    auto *viewer = new osgViewer::Viewer;
    viewer->setUpViewInWindow(200, 50, 1000, 1000);
    auto *manipulator = new osgGA::TrackballManipulator;
    viewer->setCameraManipulator(manipulator);

    osg::ref_ptr<osg::Group> grp = new osg::Group;
    grp->addChild(VIS4Earth::CreateEarth());

    VIS4Earth::GraphRenderer *graphLayout = new VIS4Earth::GraphRenderer;
    grp->addChild(graphLayout->getGroup());
    graphLayout->show();

    viewer->setSceneData(grp);
    auto prevClk = clock();
    while (!viewer->done()) {
        auto currClk = clock();
        auto duration = currClk - prevClk;

        app.processEvents();

        if (duration >= CLOCKS_PER_SEC / 45) {
            viewer->frame();
            prevClk = clock();
        }
    }

    return 0;
}