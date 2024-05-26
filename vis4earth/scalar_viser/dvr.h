#ifndef VIS4EARTH_SCALAR_VISER_DVR_H
#define VIS4EARTH_SCALAR_VISER_DVR_H

#include <ui_dvr.h>

#include <QtCore/QTimer>

#include <osg/CoordinateSystemNode>
#include <osg/CullFace>
#include <osg/Group>
#include <osg/ShapeDrawable>

#include <vis4earth/geographics_cmpt.h>
#include <vis4earth/osg_util.h>
#include <vis4earth/qt_osg_reflectable.h>
#include <vis4earth/volume_cmpt.h>

namespace Ui {
class DirectVolumeRenderer;
}

namespace VIS4Earth {

class DirectVolumeRenderer : public QtOSGReflectableWidget {
    Q_OBJECT

  public:
    DirectVolumeRenderer(QWidget *parent = nullptr) : QtOSGReflectableWidget(ui, parent) {
        layout()->addWidget(&geoCmpt);
        layout()->addWidget(&volCmpt);

        for (auto name : {"lightPosX", "lightPosY", "lightPosZ"}) {
            // 为光源位置进行坐标转换
            auto &prop = properties.at(name);
            prop->SetConvertor(
                [&, name = std::string(name)](Reflectable::Type val) -> Reflectable::Type {
                    assert(val.type == Reflectable::ESupportedType::Float);

                    float lon = ui.doubleSpinBox_lightPosX_float_VIS4EarthReflectable->value();
                    float lat = ui.doubleSpinBox_lightPosY_float_VIS4EarthReflectable->value();
                    float h = ui.doubleSpinBox_lightPosZ_float_VIS4EarthReflectable->value();
                    auto xyz = Math::BLHToEarth(Math::Deg2Rad(lon), Math::Deg2Rad(lat),
                                                static_cast<float>(osg::WGS_84_RADIUS_POLAR) + h);

                    if (std::strcmp(name.c_str(), "lightPosX") == 0)
                        return Reflectable::Type(xyz[0]);
                    else if (std::strcmp(name.c_str(), "lightPosY") == 0)
                        return Reflectable::Type(xyz[1]);
                    return Reflectable::Type(xyz[2]);
                });
        }
        for (auto name : {"sliceDirX", "sliceDirY", "sliceDirZ"}) {
            // 为切片方向进行归一化
            auto &prop = properties.at(name);
            prop->SetConvertor(
                [&, name = std::string(name)](Reflectable::Type val) -> Reflectable::Type {
                    assert(val.type == Reflectable::ESupportedType::Float);

                    osg::Vec3 dir(ui.doubleSpinBox_sliceDirX_float_VIS4EarthReflectable->value(),
                                  ui.doubleSpinBox_sliceDirY_float_VIS4EarthReflectable->value(),
                                  ui.doubleSpinBox_sliceDirZ_float_VIS4EarthReflectable->value());
                    dir.normalize();

                    ui.doubleSpinBox_sliceDirX_float_VIS4EarthReflectable->setValue(dir.x());
                    ui.doubleSpinBox_sliceDirY_float_VIS4EarthReflectable->setValue(dir.y());
                    ui.doubleSpinBox_sliceDirZ_float_VIS4EarthReflectable->setValue(dir.z());

                    if (std::strcmp(name.c_str(), "sliceDirX") == 0)
                        return Reflectable::Type(dir.x());
                    else if (std::strcmp(name.c_str(), "sliceDirY") == 0)
                        return Reflectable::Type(dir.y());
                    return Reflectable::Type(dir.z());
                });
        }

        initOSGResource();

        auto onHeightChanged = [&](double) {
            sphere->setShape(new osg::Sphere(
                osg::Vec3(0.f, 0.f, 0.f),
                static_cast<float>(osg::WGS_84_RADIUS_POLAR) +
                    geoCmpt.GetUI().doubleSpinBox_heightMax_float_VIS4EarthReflectable->value()));
        };
        connect(geoCmpt.GetUI().doubleSpinBox_heightMax_float_VIS4EarthReflectable,
                QOverload<double>::of(&QDoubleSpinBox::valueChanged), onHeightChanged);
        onHeightChanged(0.);

        connect(&volCmpt, &VolumeComponent::VolumeChanged, [&]() {
            for (int i = 0; i < 2; ++i) {
                auto timeNum = volCmpt.GetVolumeTimeNumber(i);
                if (timeNum == 0)
                    continue;

                std::array<int, 3> voxPerVol = {volCmpt.GetVolume(i, 0)->getImage()->s(),
                                                volCmpt.GetVolume(i, 0)->getImage()->t(),
                                                volCmpt.GetVolume(i, 0)->getImage()->r()};
                dSamplePoss[i]->set(
                    osg::Vec3(1.f / voxPerVol[0], 1.f / voxPerVol[1], 1.f / voxPerVol[2]));
            }
        });
        connect(&volCmpt, &VolumeComponent::TransferFunctionChanged, [&]() {
            auto stateSet = sphere->getOrCreateStateSet();
            stateSet->setTextureAttributeAndModes(2, volCmpt.GetPreIntegratedTransferFunction(0),
                                                  osg::StateAttribute::ON);
            stateSet->setTextureAttributeAndModes(3, volCmpt.GetPreIntegratedTransferFunction(1),
                                                  osg::StateAttribute::ON);
            stateSet->setTextureAttributeAndModes(4, volCmpt.GetTransferFunction(0),
                                                  osg::StateAttribute::ON);
            stateSet->setTextureAttributeAndModes(5, volCmpt.GetTransferFunction(1),
                                                  osg::StateAttribute::ON);
        });

        auto changeVol = [&]() {
            static uint32_t currTimeID = 0;

            auto stateSet = sphere->getOrCreateStateSet();
            int validVolNum = 0;
            for (int i = 0; i < 2; ++i) {
                auto timeNum = volCmpt.GetVolumeTimeNumber(i);
                if (timeNum == 0)
                    continue;

                stateSet->setTextureAttributeAndModes(i, volCmpt.GetVolume(i, currTimeID % timeNum),
                                                      osg::StateAttribute::ON);
                ++validVolNum;
            }

            useMultiVols->set(validVolNum == 2);
            ++currTimeID;
        };
        connect(&timer, &QTimer::timeout, changeVol);
        timer.setInterval(330);
        timer.start();

        auto changeStep = [&]() {
            ui.doubleSpinBox_step_float_VIS4EarthReflectable->setMinimum(0.);

            auto heightDlt =
                geoCmpt.GetUI().doubleSpinBox_heightMax_float_VIS4EarthReflectable->value() -
                geoCmpt.GetUI().doubleSpinBox_heightMin_float_VIS4EarthReflectable->value();
            auto heightVol = volCmpt.GetVolume(0, 0) ? volCmpt.GetVolume(0, 0)->getImage()->r() : 0;
            if (heightDlt == 0. || heightVol == 0) {
                ui.doubleSpinBox_step_float_VIS4EarthReflectable->setValue(0.);
                return;
            }

            ui.doubleSpinBox_step_float_VIS4EarthReflectable->setMaximum(.5 * heightDlt);
            ui.doubleSpinBox_step_float_VIS4EarthReflectable->setValue(.4 * heightDlt / heightVol);
        };
        connect(&volCmpt, &VolumeComponent::VolumeChanged, changeStep);
        connect(&geoCmpt, &GeographicsComponent::GeographicsChanged, changeStep);
        changeStep();

        debugProperties();
    }

    osg::ref_ptr<osg::Group> GetGroup() const { return grp; }

  private:
    osg::ref_ptr<osg::Group> grp;
    osg::ref_ptr<osg::ShapeDrawable> sphere;
    osg::ref_ptr<osg::Program> program;

    osg::ref_ptr<osg::Uniform> eyePos;
    std::array<osg::ref_ptr<osg::Uniform>, 2> dSamplePoss;
    osg::ref_ptr<osg::Uniform> useMultiVols;

    Ui::DirectVolumeRenderer ui;
    QTimer timer;
    GeographicsComponent geoCmpt;
    VolumeComponent volCmpt;

    void initOSGResource() {
        grp = new osg::Group();
        sphere = new osg::ShapeDrawable();
        program = new osg::Program;

        auto stateSet = sphere->getOrCreateStateSet();
        eyePos = new osg::Uniform("eyePos", osg::Vec3());
        dSamplePoss[0] = new osg::Uniform("dSamplePos0", osg::Vec3(1.f, 1.f, 1.f));
        dSamplePoss[1] = new osg::Uniform("dSamplePos1", osg::Vec3(1.f, 1.f, 1.f));
        useMultiVols = new osg::Uniform("useMultiVols", false);
        stateSet->addUniform(eyePos);
        stateSet->addUniform(dSamplePoss[0]);
        stateSet->addUniform(dSamplePoss[1]);
        stateSet->addUniform(useMultiVols);
        stateSet->addUniform(geoCmpt.GetRotateMatrix());
        {
            osg::ref_ptr<osg::Shader> vertShader = osg::Shader::readShaderFile(
                osg::Shader::VERTEX,
                GetDataPathPrefix() + VIS4EARTH_SHADER_PREFIX "scalar_viser/dvr_vert.glsl");
            osg::ref_ptr<osg::Shader> fragShader = osg::Shader::readShaderFile(
                osg::Shader::FRAGMENT,
                GetDataPathPrefix() + VIS4EARTH_SHADER_PREFIX "scalar_viser/dvr_frag.glsl");
            program->addShader(vertShader);
            program->addShader(fragShader);

            for (auto obj : std::array<QtOSGReflectableWidget *, 3>{this, &geoCmpt, &volCmpt})
                obj->ForEachProperty([&](const std::string &name, const Property &prop) {
                    stateSet->addUniform(prop.GetUniform());
                });

            auto volTexUni = new osg::Uniform(osg::Uniform::SAMPLER_3D, "volTex0");
            volTexUni->set(0);
            stateSet->addUniform(volTexUni);
            volTexUni = new osg::Uniform(osg::Uniform::SAMPLER_3D, "volTex1");
            volTexUni->set(1);
            stateSet->addUniform(volTexUni);

            auto tfTexPreIntUni = new osg::Uniform(osg::Uniform::SAMPLER_2D, "tfTexPreInt0");
            tfTexPreIntUni->set(2);
            stateSet->addUniform(tfTexPreIntUni);
            tfTexPreIntUni = new osg::Uniform(osg::Uniform::SAMPLER_2D, "tfTexPreInt1");
            tfTexPreIntUni->set(3);
            stateSet->addUniform(tfTexPreIntUni);

            auto tfTexUni = new osg::Uniform(osg::Uniform::SAMPLER_1D, "tfTex0");
            tfTexUni->set(4);
            stateSet->addUniform(tfTexUni);
            tfTexUni = new osg::Uniform(osg::Uniform::SAMPLER_1D, "tfTex1");
            tfTexUni->set(5);
            stateSet->addUniform(tfTexUni);

            osg::ref_ptr<osg::CullFace> cf = new osg::CullFace(osg::CullFace::BACK);
            stateSet->setAttributeAndModes(cf);

            stateSet->setAttributeAndModes(program, osg::StateAttribute::ON);
            stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
            stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

            sphere->setCullCallback(new EyePositionUpdateCallback(eyePos));
        }
        grp->addChild(sphere);
    }

    void debugProperties() {
        for (auto obj : std::array<QtOSGReflectableWidget *, 3>{this, &geoCmpt, &volCmpt})
            obj->ForEachProperty([&](const std::string &name, const Property &prop) {
                auto val = prop.GetValue();
                switch (val.type) {
                case Reflectable::ESupportedType::Int:
                    qDebug() << QString::fromStdString(name) << ": " << val.val.asInt;
                    break;
                case Reflectable::ESupportedType::Bool:
                    qDebug() << QString::fromStdString(name) << ": " << val.val.asBool;
                    break;
                case Reflectable::ESupportedType::Float:
                    qDebug() << QString::fromStdString(name) << ": " << val.val.asFloat;
                    break;
                case Reflectable::ESupportedType::Double:
                    qDebug() << QString::fromStdString(name) << ": " << val.val.asDouble;
                    break;
                default:
                    assert(false);
                }
            });
    }
};

} // namespace VIS4Earth

#endif // !VIS4EARTH_SCALAR_VISER_DVR_H
