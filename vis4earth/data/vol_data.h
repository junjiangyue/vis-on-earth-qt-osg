#ifndef VIS4EARTH_DATA_VOL_DATA_H
#define VIS4EARTH_DATA_VOL_DATA_H

#include <cassert>
#include <fstream>
#include <limits>
#include <string>

#include <array>
#include <tuple>
#include <vector>

#include <osg/Texture3D>

#include <vis4earth/util.h>

namespace VIS4Earth {
enum class ESupportedVoxelType { UInt8 = 0 };

class RAWVolumeData {
  public:
    struct FromFileParameters {
        std::array<uint32_t, 3> voxPerVol;
        ESupportedVoxelType voxTy;
        std::string filePath;
    };
    static ReteurnOrError<RAWVolumeData> LoadFromFile(const FromFileParameters &param) {
        if (param.voxPerVol[0] == 0 || param.voxPerVol[1] == 0 || param.voxPerVol[2] == 0)
            return "Invalid voxPerVol.";
        RAWVolumeData vol;
        vol.voxTy = param.voxTy;
        vol.voxPerVol = param.voxPerVol;
        vol.voxPerVolYxX =
            static_cast<decltype(vol.voxPerVolYxX)>(vol.voxPerVol[0]) * vol.voxPerVol[1];

        std::ifstream is(param.filePath, std::ios::binary | std::ios::in | std::ios::ate);
        if (!is.is_open())
            return "Invalid filePath.";
        auto readSz =
            GetVoxelSize(vol.voxTy) * vol.voxPerVol[0] * vol.voxPerVol[1] * vol.voxPerVol[2];
        {
            auto pos = is.tellg();
            is.seekg(0, std::ios::beg);
            if (pos - is.tellg() < readSz)
                return "Invalid file content, which is not enough for voxPerVol.";
        }

        vol.dat.resize(readSz);
        is.read(reinterpret_cast<char *>(vol.dat.data()), readSz);

        return vol;
    }

    enum class EFilterType { Linear = 0, Kriging };
    struct ResizeParameters {
        EFilterType filterType = EFilterType::Kriging;
        std::array<uint32_t, 3> targetVoxPerVol;
    };
    ReteurnOrError<RAWVolumeData> GetResized(const ResizeParameters &param) const {
        if (dat.empty())
            return "Invalid vol.";
        if (param.targetVoxPerVol[0] == 0 || param.targetVoxPerVol[1] == 0 ||
            param.targetVoxPerVol[2] == 0 || param.targetVoxPerVol[0] > 2 * voxPerVol[0] ||
            param.targetVoxPerVol[1] > 2 * voxPerVol[1] ||
            param.targetVoxPerVol[2] > 2 * voxPerVol[2])
            return "Invalid targetVoxPerVol.";

        RAWVolumeData volOut;
        volOut.voxTy = voxTy;
        volOut.voxPerVol = param.targetVoxPerVol;
        volOut.voxPerVolYxX =
            static_cast<decltype(volOut.voxPerVolYxX)>(volOut.voxPerVol[0]) * volOut.voxPerVol[1];

        std::array<float, 3> scale = {1.f * voxPerVol[0] / volOut.voxPerVol[0],
                                      1.f * voxPerVol[1] / volOut.voxPerVol[1],
                                      1.f * voxPerVol[2] / volOut.voxPerVol[2]};

        volOut.dat.resize(volOut.GetVoxelSize() * volOut.voxPerVolYxX * volOut.voxPerVol[2]);
        auto append = [&](uint32_t x, uint32_t y, uint32_t z) {
            auto offsOut = z * volOut.voxPerVolYxX + y * volOut.voxPerVol[0] + x;
            std::array<float, 3> posIn = {scale[0] * x, scale[1] * y, scale[2] * z};

            std::array<std::array<uint32_t, 2>, 3> posInRng = {
                std::array<uint32_t, 2>{static_cast<uint32_t>(std::floorf(posIn[0])),
                                        static_cast<uint32_t>(std::ceilf(posIn[0]))},
                std::array<uint32_t, 2>{static_cast<uint32_t>(std::floorf(posIn[1])),
                                        static_cast<uint32_t>(std::ceilf(posIn[1]))},
                std::array<uint32_t, 2>{static_cast<uint32_t>(std::floorf(posIn[2])),
                                        static_cast<uint32_t>(std::ceilf(posIn[2]))}};

            auto valIn = 0.f;

            // 使用 switch 语句来选择插值方法
            switch (param.filterType) {
            case EFilterType::Linear: {
                // 线性插值
                std::array<float, 3> omegas = {posInRng[0][1] - posInRng[0][0],
                                               posInRng[1][1] - posInRng[1][0],
                                               posInRng[2][1] - posInRng[2][0]};
                omegas[0] = omegas[0] == 0.f ? 0.f : (posIn[0] - posInRng[0][0]) / omegas[0];
                omegas[1] = omegas[1] == 0.f ? 0.f : (posIn[1] - posInRng[1][0]) / omegas[1];
                omegas[2] = omegas[2] == 0.f ? 0.f : (posIn[2] - posInRng[2][0]) / omegas[2];

                for (uint8_t zi = 0; zi < 2; ++zi)
                    for (uint8_t yi = 0; yi < 2; ++yi)
                        for (uint8_t xi = 0; xi < 2; ++xi) {
                            auto omega = (zi == 0 ? 1.f - omegas[2] : omegas[2]) *
                                         (yi == 0 ? 1.f - omegas[1] : omegas[1]) *
                                         (xi == 0 ? 1.f - omegas[0] : omegas[0]);
                            valIn += omega * Sample<uint8_t>(posInRng[0][xi], posInRng[1][yi],
                                                             posInRng[2][zi]);
                        }
                break;
            }

            case EFilterType::Kriging: {
                // 克里金插值相关的 Lambda 表达式
                auto variogram = [](double h, double sill = 1.0, double nugget = 0.0,
                                    double rangeParam = 1.0) {
                    return nugget + sill * (1 - std::exp(-h / rangeParam));
                };

                auto euclideanDistance = [](const std::array<float, 3> &p1,
                                            const std::array<float, 3> &p2) {
                    double sum = 0.0;
                    for (int i = 0; i < 3; ++i) {
                        sum += (p1[i] - p2[i]) * (p1[i] - p2[i]);
                    }
                    return std::sqrt(sum);
                };

                auto computeKrigingWeights =
                    [&](const std::array<std::array<uint32_t, 2>, 3> &posInRng,
                        const std::array<float, 3> &posIn) {
                        const int numPoints = 8; // 周围8个相邻点
                        std::array<std::array<float, 3>, 8> points;
                        std::array<double, 9> variogramValues;

                        // 获取相邻点坐标
                        int index = 0;
                        for (int zi = 0; zi < 2; ++zi) {
                            for (int yi = 0; yi < 2; ++yi) {
                                for (int xi = 0; xi < 2; ++xi) {
                                    points[index] = {static_cast<float>(posInRng[0][xi]),
                                                     static_cast<float>(posInRng[1][yi]),
                                                     static_cast<float>(posInRng[2][zi])};
                                    ++index;
                                }
                            }
                        }

                        // 计算相邻点的变异函数矩阵
                        std::array<std::array<double, 8>, 8> matrix;
                        for (int i = 0; i < numPoints; ++i) {
                            for (int j = 0; j < numPoints; ++j) {
                                matrix[i][j] = variogram(euclideanDistance(points[i], points[j]));
                            }
                        }

                        // 计算目标点与相邻点的变异函数向量
                        for (int i = 0; i < numPoints; ++i) {
                            variogramValues[i] = variogram(euclideanDistance(points[i], posIn));
                        }

                        // 求解克里金权重 (这里使用简单的伪逆法)
                        std::array<double, 8> weights;
                        double sumVariogramValues = 0.0;
                        for (int i = 0; i < numPoints; ++i) {
                            sumVariogramValues += variogramValues[i];
                        }
                        for (int i = 0; i < numPoints; ++i) {
                            weights[i] = variogramValues[i] / sumVariogramValues;
                        }

                        return weights;
                    };

                auto weights = computeKrigingWeights(posInRng, posIn);
                int index = 0;
                for (uint8_t zi = 0; zi < 2; ++zi)
                    for (uint8_t yi = 0; yi < 2; ++yi)
                        for (uint8_t xi = 0; xi < 2; ++xi) {
                            valIn +=
                                weights[index] *
                                Sample<uint8_t>(posInRng[0][xi], posInRng[1][yi], posInRng[2][zi]);
                            ++index;
                        }
                break;
            }

            default:
                assert(false); // 未知的插值类型
            }

            volOut.dat[offsOut] = static_cast<uint8_t>(std::round(valIn));
        };

        for (uint32_t z = 0; z < volOut.voxPerVol[2]; ++z)
            for (uint32_t y = 0; y < volOut.voxPerVol[1]; ++y)
                for (uint32_t x = 0; x < volOut.voxPerVol[0]; ++x)
                    append(x, y, z);

        return volOut;
    }

    enum class ESmoothType { Max = 0, Avg };
    enum class ESmoothDimension { XYZ = 0, XY };
    struct SmoothParameters {
        ESmoothType smoothType = ESmoothType::Max;
        ESmoothDimension smoothDim = ESmoothDimension::XYZ;
    };
    RAWVolumeData GetSmoothed(const SmoothParameters &param) const {
        switch (voxTy) {
        case ESupportedVoxelType::UInt8:
            return getSmoothed<uint8_t>(param);
        default:
            assert(false);
        }
    }

    const std::vector<uint8_t> &GetData() const { return dat; }
    const std::array<uint32_t, 3> GetVoxelPerVolume() const { return voxPerVol; }
    ESupportedVoxelType GetVoxelType() const { return voxTy; }
    size_t GetVoxelSize() const { return GetVoxelSize(voxTy); }
    template <typename VoxTy> VoxTy Sample(uint32_t x, uint32_t y, uint32_t z) const {
        x = std::min(x, voxPerVol[0] - 1);
        y = std::min(y, voxPerVol[1] - 1);
        z = std::min(z, voxPerVol[2] - 1);
        return *(reinterpret_cast<const VoxTy *>(dat.data()) + z * voxPerVolYxX + y * voxPerVol[0] +
                 x);
    }

    static size_t GetVoxelSize(ESupportedVoxelType Type) {
        switch (Type) {
        case ESupportedVoxelType::UInt8:
            return sizeof(uint8_t);
        default:
            assert(false);
        }
        return 0;
    }

    static std::tuple<float, float, float> GetVoxelMinMaxExtent(ESupportedVoxelType Type) {
        switch (Type) {
        case ESupportedVoxelType::UInt8:
            return std::make_tuple(0.f, static_cast<float>(std::numeric_limits<uint8_t>::max()),
                                   static_cast<float>(std::numeric_limits<uint8_t>::max()));
        default:
            assert(false);
        }
        return std::make_tuple(0.f, 0.f, 1.f);
    }

    osg::ref_ptr<osg::Texture3D> ToOSGTexture() const {
        std::array<uint32_t, 3> targetVoxPerVol = {1, 1, 1};
        for (int i = 0; i < 3; ++i)
            while (targetVoxPerVol[i] < voxPerVol[i])
                targetVoxPerVol[i] *= 2;

        auto volResized = GetResized(ResizeParameters{EFilterType::Kriging, targetVoxPerVol});

        osg::ref_ptr<osg::Image> img = new osg::Image;
        switch (voxTy) {
        case VIS4Earth::ESupportedVoxelType::UInt8:
            img->allocateImage(targetVoxPerVol[0], targetVoxPerVol[1], targetVoxPerVol[2], GL_RED,
                               GL_UNSIGNED_BYTE);
            break;
        default:
            assert(false);
        }
        img->setInternalTextureFormat(GL_RED);
        std::memcpy(img->data(), volResized.result.dat.GetData().data(),
                    GetVoxelSize() * targetVoxPerVol[2] * targetVoxPerVol[1] * targetVoxPerVol[0]);

        osg::ref_ptr<osg::Texture3D> tex = new osg::Texture3D;
        tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::FilterMode::LINEAR);
        tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::FilterMode::LINEAR);
        tex->setWrap(osg::Texture::WRAP_S, osg::Texture::WrapMode::CLAMP_TO_EDGE);
        tex->setWrap(osg::Texture::WRAP_T, osg::Texture::WrapMode::CLAMP_TO_EDGE);
        tex->setWrap(osg::Texture::WRAP_R, osg::Texture::WrapMode::CLAMP_TO_EDGE);
        tex->setInternalFormatMode(osg::Texture::InternalFormatMode::USE_IMAGE_DATA_FORMAT);
        tex->setImage(img);

        return tex;
    }

  private:
    std::array<uint32_t, 3> voxPerVol;
    size_t voxPerVolYxX;
    ESupportedVoxelType voxTy;
    std::vector<uint8_t> dat;

    template <typename T>
    T smoothKernelMax(const T *dat, const std::array<int32_t, 3> &pos,
                      ESmoothDimension smoothDim) const {
        auto scalar = std::numeric_limits<T>::min();
        std::array<int32_t, 3> dPos = {pos[0] == 0 ? 0 : -1, pos[1] == 0 ? 0 : -1,
                                       smoothDim != ESmoothDimension::XY || pos[2] == 0 ? 0 : -1};
        for (; dPos[2] < (smoothDim != ESmoothDimension::XY || pos[2] == voxPerVol[2] - 1 ? 1 : 2);
             ++dPos[2])
            for (; dPos[1] < (pos[1] == voxPerVol[1] - 1 ? 1 : 2); ++dPos[1])
                for (; dPos[0] < (pos[0] == voxPerVol[0] - 1 ? 1 : 2); ++dPos[0]) {
                    std::array<int32_t, 3> newPos = {pos[0] + dPos[0], pos[1] + dPos[1],
                                                     pos[2] + dPos[2]};
                    scalar = std::max(
                        scalar,
                        dat[newPos[2] * voxPerVolYxX + newPos[1] * voxPerVol[0] + newPos[0]]);
                }
        return scalar;
    }
    template <typename T>
    T smoothKernelAvg(const T *dat, const std::array<int32_t, 3> &pos,
                      ESmoothDimension smoothDim) const {
        float scalar = 0.f;
        uint32_t num = 0.f;
        std::array<int32_t, 3> dPos = {pos[0] == 0 ? 0 : -1, pos[1] == 0 ? 0 : -1,
                                       smoothDim != ESmoothDimension::XY || pos[2] == 0 ? 0 : -1};
        for (; dPos[2] < (smoothDim != ESmoothDimension::XY || pos[2] == voxPerVol[2] - 1 ? 1 : 2);
             ++dPos[2])
            for (; dPos[1] < (pos[1] == voxPerVol[1] - 1 ? 1 : 2); ++dPos[1])
                for (; dPos[0] < (pos[0] == voxPerVol[0] - 1 ? 1 : 2); ++dPos[0]) {
                    std::array<int32_t, 3> newPos = {pos[0] + dPos[0], pos[1] + dPos[1],
                                                     pos[2] + dPos[2]};
                    scalar += dat[newPos[2] * voxPerVolYxX + newPos[1] * voxPerVol[0] + newPos[0]];
                    ++num;
                }
        return static_cast<T>(std::roundf(1.f * scalar / num));
    }
    template <typename T> RAWVolumeData getSmoothed(const SmoothParameters &param) const {
        RAWVolumeData ret = *this;

        auto oldDat = reinterpret_cast<const T *>(dat.data());
        auto newDat = reinterpret_cast<T *>(ret.dat.data());

        size_t idx = 0;
        std::array<int32_t, 3> pos;
        for (pos[2] = 0; pos[2] < voxPerVol[2]; ++pos[2])
            for (pos[1] = 0; pos[1] < voxPerVol[1]; ++pos[1])
                for (pos[0] = 0; pos[0] < voxPerVol[0]; ++pos[0]) {
                    newDat[idx] = param.smoothType == ESmoothType::Avg
                                      ? smoothKernelAvg(oldDat, pos, param.smoothDim)
                                      : smoothKernelMax(oldDat, pos, param.smoothDim);
                    ++idx;
                }

        return ret;
    }
};

} // namespace VIS4Earth

#endif // !VIS4EARTH_DATA_VOL_DATA_H
