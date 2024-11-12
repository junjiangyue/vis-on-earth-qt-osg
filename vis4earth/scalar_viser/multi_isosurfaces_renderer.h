#ifndef SCIVIS_SCALAR_VISER_MULTI_ISOSURFACES_RENDERER
#define SCIVIS_SCALAR_VISER_MULTI_ISOSURFACES_RENDERER

#include <string>

#include <array>
#include <map>

#include <osg/CoordinateSystemNode>
#include <osg/CullFace>
#include <osg/ShapeDrawable>
#include <osg/Texture1D>
#include <osg/Texture3D>

#include <vis4earth/util.h>
using namespace VIS4Earth;

namespace VIS4Earth {
namespace ScalarViser {
class MultiIsosurfacesRenderer {
  public:
    static constexpr auto MaxIsoValNum = 16;

    struct ShadingParam {
        bool useShading;
        float ka;
        float kd;
        float ks;
        float shininess;
        osg::Vec3 lightPos;
    };

  private:
    struct PerRendererParam {
        osg::ref_ptr<osg::Group> grp;
        osg::ref_ptr<osg::Group> selectGrp;
        osg::ref_ptr<osg::Program> program;
        osg::ref_ptr<osg::Program> selectProgram;

        osg::ref_ptr<osg::Uniform> eyePos;
        osg::ref_ptr<osg::Uniform> dt;
        osg::ref_ptr<osg::Uniform> maxStepCnt;

        osg::ref_ptr<osg::Uniform> useShading;
        osg::ref_ptr<osg::Uniform> ka;
        osg::ref_ptr<osg::Uniform> kd;
        osg::ref_ptr<osg::Uniform> ks;
        osg::ref_ptr<osg::Uniform> shininess;
        osg::ref_ptr<osg::Uniform> lightPos;

        class Callback : public osg::NodeCallback {
          private:
            osg::Vec3 eyePos;

            osg::ref_ptr<osg::Uniform> eyePosUni;

          public:
            Callback(osg::ref_ptr<osg::Uniform> eyePosUni) : eyePosUni(eyePosUni) {}
            virtual void operator()(osg::Node *node, osg::NodeVisitor *nv) {
                eyePos = nv->getEyePoint();
                eyePosUni->set(eyePos);

                traverse(node, nv);
            }
        };

        PerRendererParam() {
            grp = new osg::Group;
            selectGrp = new osg::Group;

            osg::ref_ptr<osg::Shader> vertShader = osg::Shader::readShaderFile(
                osg::Shader::VERTEX,
                GetDataPathPrefix() + VIS4EARTH_SHADER_PREFIX "scalar_viser/misf_vert.glsl");
            osg::ref_ptr<osg::Shader> fragShader = osg::Shader::readShaderFile(
                osg::Shader::FRAGMENT,
                GetDataPathPrefix() + VIS4EARTH_SHADER_PREFIX "scalar_viser/misf_frag.glsl");
            program = new osg::Program;
            program->addShader(vertShader);
            program->addShader(fragShader);

            fragShader = osg::Shader::readShaderFile(osg::Shader::FRAGMENT,
                                                     GetDataPathPrefix() + VIS4EARTH_SHADER_PREFIX
                                                         "scalar_viser/misf_select_frag.glsl");
            selectProgram = new osg::Program;
            selectProgram->addShader(vertShader);
            selectProgram->addShader(fragShader);

#define STATEMENT(name, val) name = new osg::Uniform(#name, val)
            STATEMENT(eyePos, osg::Vec3());
            STATEMENT(dt, static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) * .008f);
            STATEMENT(maxStepCnt, 100);

            STATEMENT(useShading, 0);
            STATEMENT(ka, .5f);
            STATEMENT(kd, .5f);
            STATEMENT(ks, .5f);
            STATEMENT(shininess, 16.f);
            STATEMENT(lightPos, osg::Vec3());
#undef STATEMENT

            grp->setCullCallback(new Callback(eyePos));
            selectGrp->setCullCallback(new Callback(eyePos));
        }
    };
    PerRendererParam param;

    class PerVolParam {
        bool inSelectMode = false;
        bool isDisplayed;

        osg::ref_ptr<osg::Uniform> minLatitute;
        osg::ref_ptr<osg::Uniform> maxLatitute;
        osg::ref_ptr<osg::Uniform> minLongtitute;
        osg::ref_ptr<osg::Uniform> maxLongtitute;
        osg::ref_ptr<osg::Uniform> minHeight;
        osg::ref_ptr<osg::Uniform> maxHeight;
        osg::ref_ptr<osg::Uniform> volStartFromZeroLon;
        osg::ref_ptr<osg::Uniform> rotMat;
        osg::ref_ptr<osg::Uniform> dSamplePos;

        osg::ref_ptr<osg::Uniform> isosurfNum;
        osg::ref_ptr<osg::Uniform> sortedIsoVals;
        osg::ref_ptr<osg::Uniform> isosurfCols;
        osg::ref_ptr<osg::Uniform> selectedIsosurfIdx;

        osg::ref_ptr<osg::ShapeDrawable> sphere;
        osg::ref_ptr<osg::ShapeDrawable> selectSphere;
        osg::ref_ptr<osg::Texture3D> volTex;
        osg::ref_ptr<osg::Texture3D> volTexSmoothed;

      public:
        PerVolParam(osg::ref_ptr<osg::Texture3D> volTex,
                    osg::ref_ptr<osg::Texture3D> volTexSmoothed,
                    const std::vector<std::tuple<float, std::array<float, 4>>> &sortedIsosurfs,
                    const std::array<uint32_t, 3> &volDim, PerRendererParam *renderer)
            : volTex(volTex), volTexSmoothed(volTexSmoothed) {
            const auto MinHeight = static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) * 1.1f;
            const auto MaxHeight = static_cast<float>(osg::WGS_84_RADIUS_EQUATOR) * 1.3f;

            auto tessl = new osg::TessellationHints;
            tessl->setDetailRatio(10.f);
            sphere =
                new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.f, 0.f, 0.f), MaxHeight), tessl);
            selectSphere = new osg::ShapeDrawable(*sphere);

#define STATEMENT(name, val) name = new osg::Uniform(#name, val);
            STATEMENT(minLatitute, deg2Rad(-10.f));
            STATEMENT(maxLatitute, deg2Rad(+10.f));
            STATEMENT(minLongtitute, deg2Rad(-20.f));
            STATEMENT(maxLongtitute, deg2Rad(+20.f));
            STATEMENT(minHeight, MinHeight);
            STATEMENT(maxHeight, MaxHeight);
            STATEMENT(volStartFromZeroLon, 0);
            {
                osg::Matrix3 tmpMat;
                tmpMat.makeIdentity();
                STATEMENT(rotMat, tmpMat);
            }
            STATEMENT(dSamplePos, osg::Vec3(1.f / volDim[0], 1.f / volDim[1], 1.f / volDim[2]));

            auto volTexUni = new osg::Uniform(osg::Uniform::SAMPLER_3D, "volTex");
            volTexUni->set(0);

            STATEMENT(isosurfNum, 0);
            sortedIsoVals = new osg::Uniform(osg::Uniform::FLOAT, "sortedIsoVals", MaxIsoValNum);
            isosurfCols = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "isosurfCols", MaxIsoValNum);
            SetIsosurfaces(sortedIsosurfs);

            STATEMENT(selectedIsosurfIdx, -1);
#undef STATEMENT

            auto initStates = [&](osg::StateSet *states, bool isSelect) {
                if (!isSelect) {
                    states->addUniform(rotMat);
                    states->addUniform(dSamplePos);
                    states->addUniform(renderer->useShading);
                    states->addUniform(renderer->ka);
                    states->addUniform(renderer->kd);
                    states->addUniform(renderer->ks);
                    states->addUniform(renderer->shininess);
                    states->addUniform(renderer->lightPos);
                    states->addUniform(isosurfCols);
                    states->addUniform(selectedIsosurfIdx);
                }
                states->addUniform(minLatitute);
                states->addUniform(maxLatitute);
                states->addUniform(minLongtitute);
                states->addUniform(maxLongtitute);
                states->addUniform(minHeight);
                states->addUniform(maxHeight);
                states->addUniform(volStartFromZeroLon);

                states->addUniform(sortedIsoVals);
                states->addUniform(isosurfNum);

                states->addUniform(renderer->eyePos);
                states->addUniform(renderer->dt);
                states->addUniform(renderer->maxStepCnt);

                states->setTextureAttributeAndModes(0, volTex, osg::StateAttribute::ON);
                states->addUniform(volTexUni);

                osg::ref_ptr<osg::CullFace> cf = new osg::CullFace(osg::CullFace::BACK);
                states->setAttributeAndModes(cf);

                if (!isSelect) {
                    states->setAttributeAndModes(renderer->program, osg::StateAttribute::ON);
                    states->setMode(GL_BLEND, osg::StateAttribute::ON);
                    states->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
                } else
                    states->setAttributeAndModes(renderer->selectProgram, osg::StateAttribute::ON);
            };

            initStates(sphere->getOrCreateStateSet(), false);
            initStates(selectSphere->getOrCreateStateSet(), true);
        }
        /*
         * 函数: SetIsosurfaces
         * 功能: 设置该体绘制时的多个等值面
         * 参数:
         * -- sortedIsosurfs:
         * 多等值面的参数，包括值和颜色，所有元素的取值范围均为[0,1]。不同等值面需按值的非降序排序
         */
        void
        SetIsosurfaces(const std::vector<std::tuple<float, std::array<float, 4>>> &sortedIsosurfs) {
            isosurfNum->set(static_cast<int>(sortedIsosurfs.size()));
            for (unsigned int i = 0; i < MaxIsoValNum; ++i)
                if (i < sortedIsosurfs.size()) {
                    auto &isosurf = sortedIsosurfs[i];
                    sortedIsoVals->setElement(i, std::get<0>(isosurf));
                    osg::Vec4 osgCol(std::get<1>(isosurf)[0], std::get<1>(isosurf)[1],
                                     std::get<1>(isosurf)[2], std::get<1>(isosurf)[3]);
                    isosurfCols->setElement(i, osgCol);
                } else
                    sortedIsoVals->setElement(i, -1.f);
        }
        std::vector<std::tuple<float, std::array<float, 4>>> GetIsosurfaces() const {
            std::vector<std::tuple<float, std::array<float, 4>>> ret;
            for (unsigned int i = 0; i < MaxIsoValNum; ++i) {
                float isoVal;
                sortedIsoVals->getElement(i, isoVal);
                if (isoVal < 0.f)
                    break;

                osg::Vec4 osgCol;
                isosurfCols->getElement(i, osgCol);

                ret.emplace_back(
                    isoVal, std::array<float, 4>{osgCol.r(), osgCol.g(), osgCol.b(), osgCol.a()});
            }

            return ret;
        }
        /*
         * 函数: SelectIsosurface
         * 功能: 选择一个等值面并突出显示
         * 参数:
         * -- isoVal: 等值面的值
         * -- eps: 值的容差
         */
        void SelectIsosurface(float isoVal, float eps = 1.f / 255.f) {
            for (int i = 0; i < MaxIsoValNum; ++i) {
                float _isoVal;
                sortedIsoVals->getElement(i, _isoVal);
                if (_isoVal < 0.f)
                    break;

                if (abs(_isoVal - isoVal) <= eps) {
                    selectedIsosurfIdx->set(i);
                    break;
                }
            }
        }
        void UnselectIsosurface() { selectedIsosurfIdx->set(-1); }
        /*
         * 函数: SetUseSmoothedVolume
         * 功能: 设置是否使用平滑体数据
         * 参数:
         * -- useSmoothedVol: 为真时，使用平滑体数据
         */
        void SetUseSmoothedVolume(bool useSmoothedVol) {
            auto set = [&](osg::StateSet *states) {
                if (useSmoothedVol)
                    states->setTextureAttributeAndModes(0, volTexSmoothed, osg::StateAttribute::ON);
                else
                    states->setTextureAttributeAndModes(0, volTex, osg::StateAttribute::ON);
            };
            set(sphere->getOrCreateStateSet());
            set(selectSphere->getOrCreateStateSet());
        }
        /*
         * 函数: SetLongtituteRange
         * 功能: 设置该体绘制时的经度范围（单位为角度）
         * 参数:
         * -- minLonDeg: 经度最小值
         * -- maxLonDeg: 经度最大值
         * 返回值: 若输入的参数不合法，返回false。若设置成功，返回true
         */
        bool SetLongtituteRange(float minLonDeg, float maxLonDeg) {
            if (minLonDeg < -180.f)
                return false;
            if (maxLonDeg > +180.f)
                return false;
            if (minLonDeg >= maxLonDeg)
                return false;

            minLongtitute->set(deg2Rad(minLonDeg));
            maxLongtitute->set(deg2Rad(maxLonDeg));

            computeRotMat();
            return true;
        }
        std::array<float, 2> GetLongtituteRange() const {
            std::array<float, 2> ret;
            minLongtitute->get(ret[0]);
            maxLongtitute->get(ret[1]);
            return ret;
        }
        /*
         * 函数: SetLatituteRange
         * 功能: 设置该体绘制时的纬度范围（单位为角度）
         * 参数:
         * -- minLatDeg: 纬度最小值
         * -- maxLatDeg: 纬度最大值
         * 返回值: 若输入的参数不合法，返回false。若设置成功，返回true
         */
        bool SetLatituteRange(float minLatDeg, float maxLatDeg) {
            if (minLatDeg < -90.f)
                return false;
            if (maxLatDeg > +90.f)
                return false;
            if (minLatDeg >= maxLatDeg)
                return false;

            minLatitute->set(deg2Rad(minLatDeg));
            maxLatitute->set(deg2Rad(maxLatDeg));

            computeRotMat();
            return true;
        }
        std::array<float, 2> GetLatituteRange() const {
            std::array<float, 2> ret;
            minLatitute->get(ret[0]);
            maxLatitute->get(ret[1]);
            return ret;
        }
        /*
         * 函数: SetHeightFromCenterRange
         * 功能: 设置该体绘制时的高度（距球心）范围
         * 参数:
         * -- minH: 高度最小值
         * -- maxH: 高度最大值
         * 返回值: 若输入的参数不合法，返回false。若设置成功，返回true
         */
        bool SetHeightFromCenterRange(float minH, float maxH) {
            if (minH < 0.f)
                return false;
            if (minH >= maxH)
                return false;

            minHeight->set(minH);
            maxHeight->set(maxH);

            computeRotMat();
            sphere->setShape(new osg::Sphere(osg::Vec3(0.f, 0.f, 0.f), maxH));
            return true;
        }
        std::array<float, 2> GetHeightFromCenterRange() const {
            std::array<float, 2> ret;
            minHeight->get(ret[0]);
            maxHeight->get(ret[1]);
            return ret;
        }
        /*
         * 函数: SetVolumeStartFromLongtituteZero
         * 功能: 若全球体数据X=0对应的经度为0度，需要开启该功能
         * 参数:
         * -- flag: 为true时，开启该功能。为false时，关闭该功能
         */
        void SetVolumeStartFromLongtituteZero(bool flag) {
            if (flag)
                volStartFromZeroLon->set(1);
            else
                volStartFromZeroLon->set(0);
        }

      private:
        float deg2Rad(float deg) { return deg * osg::PI / 180.f; };
        void computeRotMat() {
            float minLon, maxLon;
            float minLat, maxLat;
            float minH, maxH;
            minLongtitute->get(minLon);
            maxLongtitute->get(maxLon);
            minLatitute->get(minLat);
            maxLatitute->get(maxLat);
            minHeight->get(minH);
            maxHeight->get(maxH);

            auto lon = .5f * (maxLon + minLon);
            auto lat = .5f * (maxLat + minLat);
            auto h = .5f * (maxH + minH);
            osg::Vec3 dir;
            dir.z() = h * sin(lat);
            h = h * cos(lat);
            dir.y() = h * sin(lon);
            dir.x() = h * cos(lon);
            dir.normalize();

            osg::Matrix3 rotMat;
            rotMat(2, 0) = dir.x();
            rotMat(2, 1) = dir.y();
            rotMat(2, 2) = dir.z();
            auto tmp = osg::Vec3(0.f, 0.f, 1.f);
            tmp = tmp ^ dir;
            rotMat(0, 0) = tmp.x();
            rotMat(0, 1) = tmp.y();
            rotMat(0, 2) = tmp.z();
            tmp = dir ^ tmp;
            rotMat(1, 0) = tmp.x();
            rotMat(1, 1) = tmp.y();
            rotMat(1, 2) = tmp.z();

            this->rotMat->set(rotMat);
        }

        friend class MultiIsosurfacesRenderer;
    };
    std::map<std::string, PerVolParam> vols;

  public:
    MultiIsosurfacesRenderer() {}

    /*
     * 函数: GetGroup
     * 功能: 获取该绘制组件的OSG节点
     * 返回值: OSG节点
     */
    osg::Group *GetGroup() { return param.grp.get(); }
    osg::Group *GetSelectGroup() { return param.selectGrp.get(); }
    /*
     * 函数: AddVolume
     * 功能: 向该绘制组件添加一个体
     * 参数:
     * -- name: 添加体的名称。不同体的名称需不同，用于区分
     * -- volTex: 体的OSG三维纹理
     * -- sortedIsosurfs:
     * 多等值面的参数，包括值和颜色，所有元素的取值范围均为[0,1]。不同等值面需按值的非降序排序
     * -- volDim: 体的三维尺寸
     * -- isDisplayed: 为true时，体被加入后会被绘制。否则体只被加入绘制组件，但不会被绘制
     */
    void AddVolume(const std::string &name, osg::ref_ptr<osg::Texture3D> volTex,
                   osg::ref_ptr<osg::Texture3D> volTexSmoothed,
                   const std::vector<std::tuple<float, std::array<float, 4>>> &sortedIsosurfs,
                   const std::array<uint32_t, 3> &volDim, bool isDisplayed = true) {
        auto itr = vols.find(name);
        if (itr != vols.end() && itr->second.isDisplayed) {
            param.grp->removeChild(itr->second.sphere);
            param.selectGrp->removeChild(itr->second.selectSphere);
            vols.erase(itr);
        }
        auto opt = vols.emplace(
            std::piecewise_construct, std::forward_as_tuple(name),
            std::forward_as_tuple(volTex, volTexSmoothed, sortedIsosurfs, volDim, &param));

        opt.first->second.isDisplayed = isDisplayed;
        if (isDisplayed) {
            param.grp->addChild(opt.first->second.sphere);
            param.selectGrp->addChild(opt.first->second.selectSphere);
        }
    }
    /*
     * 函数: DisplayVolume
     * 功能:
     * 绘制该绘制组件中的一个体，位于组件中的其他体将不被绘制。一般用于产生体动画。当输入的体不在该组件中时，所有体都不会被绘制。
     * 参数:
     * -- name: 体的名称
     */
    void DisplayVolume(const std::string &name) {
        for (auto itr = vols.begin(); itr != vols.end(); ++itr) {
            if (itr->first == name) {
                itr->second.isDisplayed = true;
                param.grp->addChild(itr->second.sphere);
                param.selectGrp->addChild(itr->second.selectSphere);
            } else if (itr->second.isDisplayed == true) {
                itr->second.isDisplayed = false;
                param.grp->removeChild(itr->second.sphere);
                param.selectGrp->removeChild(itr->second.selectSphere);
            }
        }
    }
    /*
     * 函数: GetVolumes
     * 功能: 获取该组件中，体在绘制时所需的所有数据
     */
    std::map<std::string, PerVolParam> &GetVolumes() { return vols; }
    /*
     * 函数: GetVolume
     * 功能: 获取该组件中，体在绘制时所需的数据
     * 参数:
     * -- name: 体的名称
     * 返回值: 体的绘制数据
     */
    PerVolParam *GetVolume(const std::string &name) {
        auto itr = vols.find(name);
        if (itr == vols.end())
            return nullptr;
        return &(itr->second);
    }
    /*
     * 函数: GetVolumeNum
     * 功能: 获取该绘制组件中体的数量
     * 返回值: 体的数量
     */
    size_t GetVolumeNum() const { return vols.size(); }
    /*
     * 函数: SetDeltaT
     * 功能: 设置体绘制时，光线传播的步长
     * 参数:
     * -- dt: 光线传播的步长
     */
    void SetDeltaT(float dt) { param.dt->set(dt); }
    float GetDeltaT() const {
        float ret;
        param.dt->get(ret);
        return ret;
    }
    /*
     * 函数: SetMaxStepCount
     * 功能: 设置体绘制时，光线传播的最大步数
     * 参数:
     * -- maxStepCnt: 光线传播的最大步数
     */
    void SetMaxStepCount(int maxStepCnt) { param.maxStepCnt->set(maxStepCnt); }
    int GetMaxStepCount() const {
        int ret;
        param.maxStepCnt->get(ret);
        return ret;
    }
    /*
     * 函数: SetShading
     * 功能: 设置体绘制中的光照着色参数
     * 参数:
     * -- param: Blinn-Phong光照着色参数
     */
    void SetShading(const ShadingParam &param) {
        if (!param.useShading)
            this->param.useShading->set(0);
        else {
            this->param.useShading->set(1);
            this->param.ka->set(param.ka);
            this->param.kd->set(param.kd);
            this->param.ks->set(param.ks);
            this->param.shininess->set(param.shininess);
            this->param.lightPos->set(param.lightPos);
        }
    }
};
} // namespace ScalarViser
} // namespace VIS4Earth

#endif // !SCIVIS_SCALAR_VISER_MULTI_ISOSURFACES_RENDERER
