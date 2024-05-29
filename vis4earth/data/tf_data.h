#ifndef VIS4EARTH_DATA_TF_DATA_H
#define VIS4EARTH_DATA_TF_DATA_H

#include <fstream>
#include <limits>
#include <string>

#include <array>
#include <map>
#include <vector>

#include <osg/Texture1D>
#include <osg/Texture2D>

#include <vis4earth/util.h>

namespace VIS4Earth {
class TransferFunctionData {
  public:
    enum class EFilterType { Linear, Spline };
    struct FromFileParameters {
        EFilterType filterTy;
        std::string filePath;
    };
    static ReteurnOrError<TransferFunctionData> LoadFromFile(const FromFileParameters &param) {
        std::ifstream is(param.filePath, std::ios::in);
        if (!is.is_open())
            return "Invalid filePath.";

        TransferFunctionData tf;
        tf.filterTy = param.filterTy;

        std::string buf;
        while (std::getline(is, buf)) {
            float scalar;
            std::array<float, 4> rgba;
            auto validRead =
                sscanf(buf.c_str(), "%f%f%f%f%f", &scalar, &rgba[0], &rgba[1], &rgba[2], &rgba[3]);
            if (validRead != 5)
                continue;

            for (uint8_t i = 0; i < 4; ++i)
                rgba[i] /= 255.f;
            tf.pnts.emplace(std::make_pair(static_cast<uint8_t>(scalar), rgba));
        }
        if (tf.pnts.empty())
            return "Invalid file content.";

        tf.fromPointsToFlatData();
        return tf;
    }

    void SetFilterType(EFilterType type) {
        if (filterTy == type)
            return;

        filterTy = type;
        fromPointsToFlatData();
    }

    const std::map<uint8_t, std::array<float, 4>> &GetPoints() const { return pnts; }
    const std::array<std::array<float, 4>, 256> &GetFlatData() const { return flatDat; }

    osg::ref_ptr<osg::Texture1D> ToOSGTexture() const {
        osg::ref_ptr<osg::Image> img = new osg::Image;
        img->allocateImage(256, 1, 1, GL_RGBA, GL_FLOAT);
        img->setInternalTextureFormat(GL_RGBA);
        std::memcpy(img->data(), flatDat.data(), sizeof(flatDat[0]) * flatDat.size());

        osg::ref_ptr<osg::Texture1D> tex = new osg::Texture1D;
        tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::FilterMode::LINEAR);
        tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::FilterMode::LINEAR);
        tex->setWrap(osg::Texture::WRAP_S, osg::Texture::WrapMode::CLAMP_TO_EDGE);
        tex->setInternalFormatMode(osg::Texture::InternalFormatMode::USE_IMAGE_DATA_FORMAT);
        tex->setImage(img);

        return tex;
    }

    osg::ref_ptr<osg::Texture2D> ToPreIntegratedOSGTexture() const {
        osg::ref_ptr<osg::Image> img = new osg::Image;
        img->allocateImage(256, 256, 1, GL_RGBA, GL_FLOAT);
        img->setInternalTextureFormat(GL_RGBA);

        decltype(flatDat) flatDatInt;
        flatDatInt[0][0] = flatDat[0][0];
        flatDatInt[0][1] = flatDat[0][1];
        flatDatInt[0][2] = flatDat[0][2];
        flatDatInt[0][3] = flatDat[0][3];
        for (int i = 1; i < 256; ++i) {
            auto a = .5f * (flatDat[i - 1][3] + flatDat[i][3]);
            auto r = .5f * (flatDat[i - 1][0] + flatDat[i][0]) * a;
            auto g = .5f * (flatDat[i - 1][1] + flatDat[i][1]) * a;
            auto b = .5f * (flatDat[i - 1][2] + flatDat[i][2]) * a;

            flatDatInt[i][0] = flatDatInt[i - 1][0] + r;
            flatDatInt[i][1] = flatDatInt[i - 1][1] + g;
            flatDatInt[i][2] = flatDatInt[i - 1][2] + b;
            flatDatInt[i][3] = flatDatInt[i - 1][3] + a;
        }

        auto tfPreIntDatPtr = reinterpret_cast<std::array<float, 4> *>(img->data());
        for (int sf = 0; sf < 256; ++sf)
            for (int sb = 0; sb < 256; ++sb) {
                auto sMin = sf;
                auto sMax = sb;
                if (sf > sb)
                    std::swap(sMin, sMax);

                if (sMin == sMax) {
                    auto a = flatDat[sMin][3];
                    (*tfPreIntDatPtr)[0] = flatDat[sMin][0] * a;
                    (*tfPreIntDatPtr)[1] = flatDat[sMin][1] * a;
                    (*tfPreIntDatPtr)[2] = flatDat[sMin][2] * a;
                    (*tfPreIntDatPtr)[3] = 1.f - std::exp(-a);
                } else {
                    auto factor = 1.f / (sMax - sMin);
                    (*tfPreIntDatPtr)[0] = (flatDatInt[sMax][0] - flatDatInt[sMin][0]) * factor;
                    (*tfPreIntDatPtr)[1] = (flatDatInt[sMax][1] - flatDatInt[sMin][1]) * factor;
                    (*tfPreIntDatPtr)[2] = (flatDatInt[sMax][2] - flatDatInt[sMin][2]) * factor;
                    (*tfPreIntDatPtr)[3] =
                        1.f - std::exp((flatDatInt[sMin][3] - flatDatInt[sMax][3]) * factor);
                }

                ++tfPreIntDatPtr;
            }

        osg::ref_ptr<osg::Texture2D> tex = new osg::Texture2D;
        tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::FilterMode::LINEAR);
        tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::FilterMode::LINEAR);
        tex->setWrap(osg::Texture::WRAP_S, osg::Texture::WrapMode::CLAMP_TO_EDGE);
        tex->setInternalFormatMode(osg::Texture::InternalFormatMode::USE_IMAGE_DATA_FORMAT);
        tex->setImage(img);

        return tex;
    }

  private:
    EFilterType filterTy;
    std::map<uint8_t, std::array<float, 4>> pnts;
    std::array<std::array<float, 4>, 256> flatDat;

    void fromPointsToFlatData() {
        auto pntItr = pnts.begin();
        auto lftPntItr = pntItr;
        auto lft2Rht = 1.f;

        for (int i = 0; i < 256; ++i) {
            auto assign = [&](float t) {
                flatDat[i][0] = (1.f - t) * lftPntItr->second[0] + t * pntItr->second[0];
                flatDat[i][1] = (1.f - t) * lftPntItr->second[1] + t * pntItr->second[1];
                flatDat[i][2] = (1.f - t) * lftPntItr->second[2] + t * pntItr->second[2];
                flatDat[i][3] = (1.f - t) * lftPntItr->second[3] + t * pntItr->second[3];
            };

            if (pntItr == pnts.end())
                assign(1.f);
            else if (i == static_cast<int>(pntItr->first)) {
                assign(1.f);
                lftPntItr = pntItr;
                ++pntItr;
                if (pntItr != pnts.end())
                    lft2Rht = pntItr->first - lftPntItr->first;
                else
                    lft2Rht = 1.f;
            } else
                assign((i - lftPntItr->first) / lft2Rht);
        }
    }
};
} // namespace VIS4Earth

#endif // !VIS4EARTH_DATA_TF_DATA_H
