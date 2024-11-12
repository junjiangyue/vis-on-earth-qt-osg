#include <memory>

#include <array>
#include <tuple>

#include <QtWidgets/qapplication.h>

#include <osgGA/GUIEventHandler>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

#include <vis4earth/osg_util.h>

#include <vis4earth/io/tf_io.h>
#include <vis4earth/io/tf_osg_io.h>
#include <vis4earth/io/vol_io.h>
#include <vis4earth/io/vol_osg_io.h>
#include <vis4earth/scalar_viser/multi_isosurfaces_renderer.h>

#include "misf_main_window.h"

static const std::string volPath = DATA_PATH_PREFIX "OSS/OSS000.raw";
static const std::string volName = "0";
static const std::vector<std::tuple<float, std::array<float, 4>>> isosurfaces = {
    std::make_tuple(30.f / 255.f, std::array<float, 4>{0.f, 0.f, 1.f, .5f}),
    std::make_tuple(120.f / 255.f, std::array<float, 4>{0.f, 1.f, 0.f, .5f}),
    std::make_tuple(160.f / 255.f, std::array<float, 4>{1.f, 0.f, 0.f, .5f})};
static const std::array<uint32_t, 3> dim = {300, 350, 50};
static const std::array<uint8_t, 3> log2Dim = {9, 9, 6};
static const std::array<float, 2> lonRng = {100.05f, 129.95f};
static const std::array<float, 2> latRng = {-4.95f, 29.95f};
static const std::array<float, 2> hRng = {1.f, 5316.f};
static const float hScale = 100.f;

struct SelectCamera {
    osg::ref_ptr<osg::Camera> camera;
    osg::ref_ptr<osg::Image> cameraRenderTo;

    SelectCamera() {}
    void Setup(osg::GraphicsContext *ctx) {
        camera = new osg::Camera();
        camera->setGraphicsContext(ctx);
        camera->setRenderTargetImplementation(
            osg::Camera::RenderTargetImplementation::FRAME_BUFFER_OBJECT);
        camera->setViewport(0, 0, ctx->getTraits()->width, ctx->getTraits()->height);
        camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        camera->setClearColor(osg::Vec4(1.f, 0.f, 0.f, 1.f));

        cameraRenderTo = new osg::Image;
        cameraRenderTo->allocateImage(ctx->getTraits()->width, ctx->getTraits()->height, 1, GL_RGBA,
                                      GL_UNSIGNED_BYTE);

        camera->attach(osg::Camera::COLOR_BUFFER0, cameraRenderTo);
    }
    osg::Vec4 ReadPixel(int col, int row) {
        col = std::min(col, cameraRenderTo->s() - 1);
        row = std::min(row, cameraRenderTo->t() - 1);

        cameraRenderTo->readPixels(0, 0, cameraRenderTo->s(), cameraRenderTo->t(), GL_RGBA,
                                   GL_UNSIGNED_BYTE);
        auto color = cameraRenderTo->getColor(col, row);
        return color;
    }
};

class MouseHanlder : public osgGA::GUIEventHandler {
  private:
    std::shared_ptr<VIS4Earth::ScalarViser::MultiIsosurfacesRenderer> renderer;
    std::shared_ptr<SelectCamera> selectCam;

  public:
    MouseHanlder(std::shared_ptr<VIS4Earth::ScalarViser::MultiIsosurfacesRenderer> renderer,
                 std::shared_ptr<SelectCamera> selectCam)
        : renderer(renderer), selectCam(selectCam) {}

    virtual bool handle(const osgGA::GUIEventAdapter &eAdpt,
                        osgGA::GUIActionAdapter &aAdpt) override {
        auto *viewer = dynamic_cast<osgViewer::Viewer *>(&aAdpt);
        if (!viewer)
            return false;

        switch (eAdpt.getEventType()) {
        case osgGA::GUIEventAdapter::RELEASE:
            if (eAdpt.getButton() == 1) {
                if (renderer->GetVolumeNum() == 0)
                    break;

                auto scrnX = static_cast<int>(eAdpt.getX());
                auto scrnY = static_cast<int>(eAdpt.getY());

                auto col = selectCam->ReadPixel(scrnX, scrnY);
                auto bgn = renderer->GetVolumes().begin();
                if (col.r() == col.g() && col.r() == col.b() && col.a() == 1.f) {
                    bgn->second.SelectIsosurface(col.r());
                } else
                    bgn->second.UnselectIsosurface();
            }
            break;
        }

        osgGA::GUIEventHandler::handle(eAdpt, aAdpt);
    }
};

int main(int argc, char **argv) {
    QApplication app(argc, argv);

    auto *viewer = new osgViewer::Viewer;
    viewer->setUpViewInWindow(200, 50, 800, 600);
    auto *manipulator = new osgGA::TrackballManipulator;
    viewer->setCameraManipulator(manipulator);

    osg::ref_ptr<osg::Group> grp = new osg::Group;
    grp->addChild(VIS4Earth::CreateEarth());

    auto misf = std::make_shared<VIS4Earth::ScalarViser::MultiIsosurfacesRenderer>();
    misf->SetDeltaT(hScale * (hRng[1] - hRng[0]) / dim[2] * .3f);
    misf->SetMaxStepCount(800);

    auto selectCam = std::make_shared<SelectCamera>();
    selectCam->Setup(viewer->getCamera()->getGraphicsContext());
    viewer->addEventHandler(new MouseHanlder(misf, selectCam));
    viewer->addSlave(selectCam->camera, false);

    MISFMainWindow mainWnd(misf);

    std::string errMsg;
    {
        auto volU8Dat = VIS4Earth::Loader::RAWVolume::LoadU8FromFile(volPath, dim, &errMsg);
        if (!errMsg.empty())
            goto ERR;

        auto volDat = VIS4Earth::Convertor::RAWVolume::U8ToNormalizedFloat(volU8Dat);
        auto volDatSmoothed = VIS4Earth::Convertor::RAWVolume::RoughFloatToSmooth(volDat, dim);
        auto volTex =
            VIS4Earth::OSGConvertor::RAWVolume::NormalizedFloatToTexture(volDat, dim, log2Dim);
        auto volTexSmoothed = VIS4Earth::OSGConvertor::RAWVolume::NormalizedFloatToTexture(
            volDatSmoothed, dim, log2Dim);

        misf->AddVolume(volName, volTex, volTexSmoothed, isosurfaces, dim);
        auto vol = misf->GetVolume(volName);
        vol->SetLongtituteRange(lonRng[0], lonRng[1]);
        vol->SetLatituteRange(latRng[0], latRng[1]);
        vol->SetHeightFromCenterRange(
            static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[0],
            static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) + hScale * hRng[1]);
    }

    mainWnd.UpdateFromRenderer();
    mainWnd.show();

    grp->addChild(misf->GetGroup());

    viewer->setSceneData(grp);
    selectCam->camera->addChild(misf->GetSelectGroup());

    auto prevClk = clock();
    while (!viewer->done()) {
        auto currClk = clock();
        auto duration = currClk - prevClk;

        auto masterCam = viewer->getCamera();
        selectCam->camera->setViewMatrix(masterCam->getViewMatrix());
        selectCam->camera->setProjectionMatrix(masterCam->getProjectionMatrix());

        app.processEvents();

        if (duration >= CLOCKS_PER_SEC / 45) {
            viewer->frame();
            prevClk = clock();
        }
    }

    return 0;

ERR:
    std::cerr << errMsg << std::endl;
    return 1;
}
