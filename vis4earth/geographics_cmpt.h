#ifndef VIS4EARTH_GEOGRAPHICS_CMPT_H
#define VIS4EARTH_GEOGRAPHICS_CMPT_H

#include <osg/CoordinateSystemNode>

#include <vis4earth/math.h>
#include <vis4earth/qt_osg_reflectable.h>

namespace Ui {
class GeographicsComponent;
}

namespace VIS4Earth {

class GeographicsComponent : public QtOSGReflectableWidget {
    Q_OBJECT

  public:
    GeographicsComponent(QWidget *parent = nullptr);

    const Ui::GeographicsComponent *GetUI() const { return ui; }

    const osg::ref_ptr<osg::Uniform> &GetRotateMatrix() const { return rotMat; }

  Q_SIGNALS:
    void GeographicsChanged();

  private:
    Ui::GeographicsComponent *ui;

    osg::ref_ptr<osg::Uniform> rotMat;
};

} // namespace VIS4Earth

#endif // !VIS4EARTH_GEOGRAPHICS_CMPT_H
