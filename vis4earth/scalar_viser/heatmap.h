#ifndef VIS4EARTH_SCALAR_VISER_HEATMAP_H
#define VIS4EARTH_SCALAR_VISER_HEATMAP_H

#include <osg/CoordinateSystemnode>
#include <osg/Group>
#include <osg/ShapeDrawable>

#include <vis4earth/geographics_cmpt.h>
#include <vis4earth/osg_util.h>
#include <vis4earth/qt_osg_reflectable.h>
#include <vis4earth/volume_cmpt.h>

#pragma comment(lib, "opengl32.lib")

namespace Ui {
class HeatmapRenderer;
}

namespace VIS4Earth {

class HeatmapRenderer : public QtOSGReflectableWidget {
    Q_OBJECT

  public:
    enum class ETextureFilterMode { Nearest, Linear };

    HeatmapRenderer(QWidget *parent = nullptr);

    osg::ref_ptr<osg::Group> GetGroup() const { return grp; }

    void emitUpdateHeatmap2DSignal() { emit updateHeatmap2DSignal(); }

  protected:
    osg::ref_ptr<osg::Group> grp;
    osg::ref_ptr<osg::Geometry> geom;
    osg::ref_ptr<osg::Geode> geode;
    osg::ref_ptr<osg::Program> program;
    osg::ref_ptr<osg::Vec3Array> verts;
    osg::ref_ptr<osg::Texture2D> volSliceTex;
    osg::ref_ptr<osg::Image> volSliceImg;

    osg::ref_ptr<osg::Uniform> volHeight;

    Ui::HeatmapRenderer *ui;
    GeographicsComponent geoCmpt;
    VolumeComponent volCmpt;
    QImage heatmap2D;

    void initOSGResource();
    void updateHeatmap2D();
    void updateGeometry();

  signals:
    void updateHeatmap2DSignal();
};

class Heatmap2DDrawCallback : public QObject, public osg::Drawable::DrawCallback {
    Q_OBJECT
  public:
    Heatmap2DDrawCallback(HeatmapRenderer *heatmapRenderer, Ui::HeatmapRenderer *ui,
                          osg::ref_ptr<osg::Texture2D> volSliceTex, VolumeComponent &volCmpt,
                          QImage &heatmap2D);
    void drawImplementation(osg::RenderInfo &renderInfo,
                            const osg::Drawable *drawable) const override;

  protected:
    HeatmapRenderer *heatmapRenderer;
    Ui::HeatmapRenderer *ui;
    osg::ref_ptr<osg::Texture2D> volSliceTex;
    VolumeComponent &volCmpt;
    QImage &heatmap2D;

    osg::ref_ptr<osg::Program> program;
};

} // namespace VIS4Earth

#endif
