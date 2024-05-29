#ifndef VIS4EARTH_SCALAR_VISER_ISOSURFACE_H
#define VIS4EARTH_SCALAR_VISER_ISOSURFACE_H

#include <ui_isosurface.h>

#include <set>
#include <vector>

#include <osg/CoordinateSystemNode>
#include <osg/CullFace>
#include <osg/Group>
#include <osg/ShapeDrawable>

#include <vis4earth/geographics_cmpt.h>
#include <vis4earth/osg_util.h>
#include <vis4earth/qt_osg_reflectable.h>
#include <vis4earth/volume_cmpt.h>

#include <vis4earth/scalar_viser/marching_cube_table.h>

namespace VIS4Earth {

class IsosurfaceRenderer : public QtOSGReflectableWidget {
    Q_OBJECT

  public:
    enum class EMeshSmoothType { None, Laplacian, Curvature };

    IsosurfaceRenderer(QWidget *parent = nullptr)
        : volCmpt(true, true), QtOSGReflectableWidget(ui, parent) {
        ui.scrollAreaWidgetContents_main->layout()->addWidget(&geoCmpt);
        ui.scrollAreaWidgetContents_main->layout()->addWidget(&volCmpt);

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

        initOSGResource();

        auto genIsosurface = [&]() {
            isoval = ui.horizontalSlider_isoval->value();
            useVolSmoothed = ui.checkBox_useVolSmoothed->isChecked();
            meshSmoothType =
                static_cast<EMeshSmoothType>(ui.comboBox_meshSmoothType->currentIndex());

            vertIndices.clear();
            verts->clear();
            norms->clear();
            uvs->clear();
            multiEdges[0].clear();
            multiEdges[1].clear();
            for (int i = 0; i < 2; ++i) {
                if (volCmpt.GetVolumeTimeNumber(i) == 0)
                    continue;
                marchingCube(i);
            }
            for (int i = 0; i < 2; ++i)
                if (volCmpt.GetVolumeTimeNumber(i) != 0)
                    updateGeometry(i);
        };
        connect(ui.horizontalSlider_isoval, &QSlider::sliderMoved,
                [&](int val) { ui.label_isoval->setText(QString::number(val)); });
        connect(ui.horizontalSlider_isoval, &QSlider::valueChanged, genIsosurface);
        connect(ui.checkBox_useVolSmoothed, &QCheckBox::stateChanged, genIsosurface);
        connect(&volCmpt, &VolumeComponent::VolumeChanged, genIsosurface);
        connect(&geoCmpt, &GeographicsComponent::GeographicsChanged, genIsosurface);
        connect(ui.comboBox_meshSmoothType, QOverload<int>::of(&QComboBox::currentIndexChanged),
                [&](int idx) {
                    meshSmoothType = static_cast<EMeshSmoothType>(idx);
                    for (int i = 0; i < 2; ++i)
                        if (volCmpt.GetVolumeTimeNumber(i) != 0)
                            updateGeometry(i);
                });

        connect(&volCmpt, &VolumeComponent::TransferFunctionChanged, [&]() {
            auto stateSet = geode->getOrCreateStateSet();
            stateSet->setTextureAttributeAndModes(0, volCmpt.GetTransferFunction(0),
                                                  osg::StateAttribute::ON);
            stateSet->setTextureAttributeAndModes(1, volCmpt.GetTransferFunction(1),
                                                  osg::StateAttribute::ON);
        });

        debugProperties({this, &volCmpt, &geoCmpt});
    }

    osg::ref_ptr<osg::Group> GetGroup() const { return grp; }

  private:
    uint8_t isoval;
    bool useVolSmoothed;
    EMeshSmoothType meshSmoothType;

    Ui::IsosurfaceRenderer ui;
    GeographicsComponent geoCmpt;
    VolumeComponent volCmpt;

    osg::ref_ptr<osg::Group> grp;
    osg::ref_ptr<osg::Geometry> geom;
    osg::ref_ptr<osg::Geode> geode;
    osg::ref_ptr<osg::Program> program;
    osg::ref_ptr<osg::Vec3Array> verts;
    osg::ref_ptr<osg::Vec3Array> smoothedVerts;
    osg::ref_ptr<osg::Vec3Array> norms;
    osg::ref_ptr<osg::Vec3Array> smoothedNorms;
    osg::ref_ptr<osg::Vec2Array> uvs;

    osg::ref_ptr<osg::Uniform> eyePos;

    std::vector<GLuint> vertIndices;
    std::array<std::set<std::array<GLuint, 2>>, 2> multiEdges;

    void initOSGResource() {
        grp = new osg::Group();
        geom = new osg::Geometry();
        geode = new osg::Geode();
        verts = new osg::Vec3Array();
        smoothedVerts = new osg::Vec3Array();
        norms = new osg::Vec3Array();
        smoothedNorms = new osg::Vec3Array();
        uvs = new osg::Vec2Array();
        program = new osg::Program();

        auto stateSet = geode->getOrCreateStateSet();
        eyePos = new osg::Uniform("eyePos", osg::Vec3());
        stateSet->addUniform(eyePos);
        for (auto obj : std::array<QtOSGReflectableWidget *, 3>{this, &geoCmpt, &volCmpt})
            obj->ForEachProperty([&](const std::string &name, const Property &prop) {
                stateSet->addUniform(prop.GetUniform());
            });
        {
            osg::ref_ptr<osg::Shader> vertShader = osg::Shader::readShaderFile(
                osg::Shader::VERTEX,
                GetDataPathPrefix() + VIS4EARTH_SHADER_PREFIX "scalar_viser/isosurface_vert.glsl");
            osg::ref_ptr<osg::Shader> fragShader = osg::Shader::readShaderFile(
                osg::Shader::FRAGMENT,
                GetDataPathPrefix() + VIS4EARTH_SHADER_PREFIX "scalar_viser/isosurface_frag.glsl");
            program->addShader(vertShader);
            program->addShader(fragShader);
        }
        {
            auto tfTexUni = new osg::Uniform(osg::Uniform::SAMPLER_1D, "tfTex0");
            tfTexUni->set(0);
            stateSet->addUniform(tfTexUni);
            tfTexUni = new osg::Uniform(osg::Uniform::SAMPLER_1D, "tfTex1");
            tfTexUni->set(1);
            stateSet->addUniform(tfTexUni);
        }
        stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
        stateSet->setAttributeAndModes(program, osg::StateAttribute::ON);

        geode->setCullCallback(new EyePositionUpdateCallback(eyePos));

        geode->addDrawable(geom);
        grp->addChild(geode);
    }

    void marchingCube(uint32_t volID) {
        using T = uint8_t;

        std::array<uint32_t, 3> voxPerVol = {volCmpt.GetUI().spinBox_voxPerVolX->value(),
                                             volCmpt.GetUI().spinBox_voxPerVolY->value(),
                                             volCmpt.GetUI().spinBox_voxPerVolZ->value()};
        auto voxPerVolYxX = static_cast<size_t>(voxPerVol[1]) * voxPerVol[0];

        auto lonMin = geoCmpt.GetPropertyOSGValue<float>("longtitudeMin").val;
        auto lonMax = geoCmpt.GetPropertyOSGValue<float>("longtitudeMax").val;
        auto latMin = geoCmpt.GetPropertyOSGValue<float>("latitudeMin").val;
        auto latMax = geoCmpt.GetPropertyOSGValue<float>("latitudeMax").val;
        auto hMin = geoCmpt.GetPropertyOSGValue<float>("heightMin").val;
        auto hMax = geoCmpt.GetPropertyOSGValue<float>("heightMax").val;
        auto lonExt = lonMax - lonMin;
        auto latExt = latMax - latMin;
        auto hExt = hMax - hMin;

        auto sample = [&](const osg::Vec3i &pos) -> T {
            if (useVolSmoothed)
                return volCmpt.GetVolumeCPUSmoothed(volID, 0).Sample<T>(pos.x(), pos.y(), pos.z());
            return volCmpt.GetVolumeCPU(volID, 0).Sample<T>(pos.x(), pos.y(), pos.z());
        };
        auto vec3ToSphere = [&](const osg::Vec3 &v3) -> osg::Vec3 {
            float lon = lonMin + v3.x() * lonExt;
            float lat = latMin + v3.y() * latExt;
            float h = hMin + v3.z() * hExt;

            return Math::BLHToEarthOSGVec3(lon, lat, h);
        };

        struct HashEdge {
            size_t operator()(const std::array<int, 3> &edgeID) const {
                size_t hash = edgeID[0];
                hash = (hash << 32) | edgeID[1];
                hash = (hash << 2) | edgeID[2];
                return std::hash<size_t>()(hash);
            };
        };
        std::array<std::unordered_map<std::array<int, 3>, GLuint, HashEdge>, 2> edge2vertIDs;

        osg::Vec3i startPos;
        for (startPos.z() = 0; startPos.z() < voxPerVol[2] - 1; ++startPos.z()) {
            if (startPos.z() != 0) {
                edge2vertIDs[0] = std::move(edge2vertIDs[1]);
                edge2vertIDs[1].clear(); // hash map only stores vertices of 2 consecutive heights
            }

            for (startPos.y() = 0; startPos.y() < voxPerVol[1] - 1; ++startPos.y())
                for (startPos.x() = 0; startPos.x() < voxPerVol[0] - 1; ++startPos.x()) {
                    // Voxels in CCW order form a grid
                    // +-----------------+
                    // |       3 <--- 2  |
                    // |       |     /|\ |
                    // |      \|/     |  |
                    // |       0 ---> 1  |
                    // |      /          |
                    // |  7 <--- 6       |
                    // |  | /   /|\      |
                    // | \|/_    |       |
                    // |  4 ---> 5       |
                    // +-----------------+
                    uint8_t cornerState = 0;
                    std::array<T, 8> scalars;
                    for (int i = 0; i < 8; ++i) {
                        scalars[i] = sample(startPos);
                        if (scalars[i] >= isoval)
                            cornerState |= 1 << i;

                        startPos.x() += i == 0 || i == 4 ? 1 : i == 2 || i == 6 ? -1 : 0;
                        startPos.y() += i == 1 || i == 5 ? 1 : i == 3 || i == 7 ? -1 : 0;
                        startPos.z() += i == 3 ? 1 : i == 7 ? -1 : 0;
                    }
                    std::array<float, 12> omegas = {1.f * scalars[0] / (scalars[1] + scalars[0]),
                                                    1.f * scalars[1] / (scalars[2] + scalars[1]),
                                                    1.f * scalars[3] / (scalars[3] + scalars[2]),
                                                    1.f * scalars[0] / (scalars[0] + scalars[3]),
                                                    1.f * scalars[4] / (scalars[5] + scalars[4]),
                                                    1.f * scalars[5] / (scalars[6] + scalars[5]),
                                                    1.f * scalars[7] / (scalars[7] + scalars[6]),
                                                    1.f * scalars[4] / (scalars[4] + scalars[7]),
                                                    1.f * scalars[0] / (scalars[0] + scalars[4]),
                                                    1.f * scalars[1] / (scalars[1] + scalars[5]),
                                                    1.f * scalars[2] / (scalars[2] + scalars[6]),
                                                    1.f * scalars[3] / (scalars[3] + scalars[7])};

                    // Edge indexed by Start Voxel Position
                    // +----------+
                    // | /*\  *|  |
                    // |  |  /    |
                    // | e1 e2    |
                    // |  * e0 *> |
                    // +----------+
                    // *:   startPos
                    // *>:  startPos + (1,0,0)
                    // /*\: startPos + (0,1,0)
                    // *|:  startPos + (0,0,1)
                    // ID(e0) = (startPos.xy, 00)
                    // ID(e1) = (startPos.xy, 01)
                    // ID(e2) = (startPos.xy, 10)
                    for (uint32_t i = 0; i < VertNumTable[cornerState]; i += 3) {
                        for (int32_t ii = 0; ii < 3; ++ii) {
                            auto ei = TriangleTable[cornerState][i + ii];
                            std::array<int, 3> edgeID = {
                                startPos.x() + (ei == 1 || ei == 5 || ei == 9 || ei == 10 ? 1 : 0),
                                startPos.y() + (ei == 2 || ei == 6 || ei == 10 || ei == 11 ? 1 : 0),
                                ei >= 8                                    ? 2
                                : ei == 1 || ei == 3 || ei == 5 || ei == 7 ? 1
                                                                           : 0};
                            auto edge2vertIDIdx = ei >= 4 && ei < 8 ? 1 : 0;
                            auto itr = edge2vertIDs[edge2vertIDIdx].find(edgeID);
                            if (itr != edge2vertIDs[edge2vertIDIdx].end()) {
                                vertIndices.emplace_back(itr->second);
                                continue;
                            }

                            osg::Vec3 pos(startPos.x() +
                                              (ei == 0 || ei == 2 || ei == 4 || ei == 6 ? omegas[ei]
                                               : ei == 1 || ei == 5 || ei == 9 || ei == 10 ? 1.f
                                                                                           : 0.f),
                                          startPos.y() +
                                              (ei == 1 || ei == 3 || ei == 5 || ei == 7 ? omegas[ei]
                                               : ei == 2 || ei == 6 || ei == 10 || ei == 11 ? 1.f
                                                                                            : 0.f),
                                          startPos.z() + (ei >= 8   ? omegas[ei]
                                                          : ei >= 4 ? 1.f
                                                                    : 0.f));
                            pos.x() /= voxPerVol[0];
                            pos.y() /= voxPerVol[1];
                            pos.z() /= voxPerVol[2];
                            pos = vec3ToSphere(pos);

                            float scalar;
                            switch (ei) {
                            case 0:
                                scalar = omegas[0] * scalars[0] + (1.f - omegas[0]) * scalars[1];
                                break;
                            case 1:
                                scalar = omegas[1] * scalars[1] + (1.f - omegas[1]) * scalars[2];
                                break;
                            case 2:
                                scalar = omegas[2] * scalars[3] + (1.f - omegas[2]) * scalars[2];
                                break;
                            case 3:
                                scalar = omegas[3] * scalars[0] + (1.f - omegas[3]) * scalars[3];
                                break;
                            case 4:
                                scalar = omegas[4] * scalars[4] + (1.f - omegas[4]) * scalars[5];
                                break;
                            case 5:
                                scalar = omegas[5] * scalars[5] + (1.f - omegas[5]) * scalars[6];
                                break;
                            case 6:
                                scalar = omegas[6] * scalars[7] + (1.f - omegas[6]) * scalars[6];
                                break;
                            case 7:
                                scalar = omegas[7] * scalars[4] + (1.f - omegas[7]) * scalars[7];
                                break;
                            default:
                                scalar = omegas[ei] * scalars[ei - 8] +
                                         (1.f - omegas[ei]) * scalars[ei - 4];
                            }

                            vertIndices.emplace_back(verts->size());
                            verts->push_back(pos);
                            norms->push_back(osg::Vec3(0.f, 0.f, 0.f));
                            uvs->push_back(osg::Vec2(volID, scalar / 255.f));
                            edge2vertIDs[edge2vertIDIdx].emplace(edgeID, vertIndices.back());
                        }

                        std::array<GLuint, 3> triVertIdxs = {vertIndices[vertIndices.size() - 3],
                                                             vertIndices[vertIndices.size() - 2],
                                                             vertIndices[vertIndices.size() - 1]};
                        osg::Vec3 norm;
                        {
                            auto e0 = (*verts)[triVertIdxs[1]] - (*verts)[triVertIdxs[0]];
                            auto e1 = (*verts)[triVertIdxs[2]] - (*verts)[triVertIdxs[0]];
                            norm = e1 ^ e0;
                            norm.normalize();
                        }

                        (*norms)[triVertIdxs[0]] += norm;
                        (*norms)[triVertIdxs[1]] += norm;
                        (*norms)[triVertIdxs[2]] += norm;

                        multiEdges[volID].emplace(
                            std::array<GLuint, 2>{triVertIdxs[0], triVertIdxs[1]});
                        multiEdges[volID].emplace(
                            std::array<GLuint, 2>{triVertIdxs[1], triVertIdxs[0]});
                        multiEdges[volID].emplace(
                            std::array<GLuint, 2>{triVertIdxs[1], triVertIdxs[2]});
                        multiEdges[volID].emplace(
                            std::array<GLuint, 2>{triVertIdxs[2], triVertIdxs[1]});
                        multiEdges[volID].emplace(
                            std::array<GLuint, 2>{triVertIdxs[2], triVertIdxs[0]});
                        multiEdges[volID].emplace(
                            std::array<GLuint, 2>{triVertIdxs[0], triVertIdxs[2]});
                    }
                }
        }

        for (auto &norm : *norms)
            norm.normalize();
    }

    void updateGeometry(uint32_t volID) {
        if (vertIndices.empty())
            return;

        auto laplacianSmooth = [&]() {
            for (GLuint vIdx = 0; vIdx < verts->size(); ++vIdx) {
                auto itr = multiEdges[volID].lower_bound(std::array<GLuint, 2>{vIdx, 0});
                assert(itr != multiEdges[volID].end());

                auto &smoothedVert = (*smoothedVerts)[vIdx];
                auto &smoothedNorm = (*smoothedNorms)[vIdx];
                int lnkNum = 1;
                while (itr != multiEdges[volID].end() && (*itr)[0] == vIdx) {
                    smoothedVert += (*verts)[(*itr)[1]];
                    smoothedNorm += (*norms)[(*itr)[1]];
                    ++itr;
                    ++lnkNum;
                }
                smoothedVert /= lnkNum;
                smoothedNorm /= lnkNum;
            }
        };
        auto curvatureSmooth = [&]() {
            for (GLuint vIdx = 0; vIdx < verts->size(); ++vIdx) {
                auto itr = multiEdges[volID].lower_bound(std::array<GLuint, 2>{vIdx, 0});
                assert(itr != multiEdges[volID].end());

                auto &smoothedVert = (*smoothedVerts)[vIdx];
                auto &norm = (*smoothedNorms)[vIdx];
                auto projLen = 0.f;
                while (itr != multiEdges[volID].end() && (*itr)[0] == vIdx) {
                    auto dlt = (*verts)[(*itr)[1]] - smoothedVert;
                    projLen = dlt * norm;
                    ++itr;
                }

                smoothedVert = smoothedVert + norm * projLen;
            }
        };

        if (meshSmoothType != EMeshSmoothType::None)
            switch (meshSmoothType) {
            case EMeshSmoothType::Laplacian:
            case EMeshSmoothType::Curvature:
                smoothedVerts->assign(verts->begin(), verts->end());
                smoothedNorms->assign(norms->begin(), norms->end());
                break;
            }
        switch (meshSmoothType) {
        case EMeshSmoothType::Laplacian:
            laplacianSmooth();
            break;
        case EMeshSmoothType::Curvature:
            curvatureSmooth();
            break;
        }

        switch (meshSmoothType) {
        case EMeshSmoothType::Laplacian:
        case EMeshSmoothType::Curvature:
            geom->setVertexArray(smoothedVerts);
            geom->setNormalArray(smoothedNorms);
            break;
        default:
            geom->setVertexArray(verts);
            geom->setNormalArray(norms);
        }
        geom->setTexCoordArray(0, uvs);

        geom->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);

        geom->getPrimitiveSetList().clear();
        geom->addPrimitiveSet(
            new osg::DrawElementsUInt(GL_TRIANGLES, vertIndices.size(), vertIndices.data()));
    }
};

} // namespace VIS4Earth

#endif // !VIS4EARTH_SCALAR_VISER_ISOSURFACE_H
