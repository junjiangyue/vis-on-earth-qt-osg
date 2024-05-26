#ifndef VIS4EARTH_REFLECTABLE_H
#define VIS4EARTH_REFLECTABLE_H

#include <osg/Uniform>

namespace VIS4Earth {

struct Reflectable {
    enum class ESupportedType : int { Int = 0, Bool, Float, Double, Undefined };
    static constexpr osg::Uniform::Type SupportedOSGTypes[] = {
        osg::Uniform::Type::INT, osg::Uniform::Type::BOOL, osg::Uniform::Type::FLOAT,
        osg::Uniform::Type::DOUBLE, osg::Uniform::Type::UNDEFINED};
    static constexpr const char *SupportedTypeNames[] = {"int", "bool", "float", "double",
                                                         "undefined"};
    struct Type {
        ESupportedType type;
        union {
            int asInt;
            bool asBool;
            float asFloat;
            double asDouble;
        } val;

        Type() : type(ESupportedType::Undefined) {}
        template <typename ValTy = int> Type(int val) : type(ESupportedType::Int) {
            this->val.asInt = val;
        }
        template <typename ValTy = bool> Type(bool val) : type(ESupportedType::Bool) {
            this->val.asBool = val;
        }
        template <typename ValTy = float> Type(float val) : type(ESupportedType::Float) {
            this->val.asFloat = val;
        }
        template <typename ValTy = double> Type(double val) : type(ESupportedType::Double) {
            this->val.asDouble = val;
        }
    };

    static osg::Uniform::Type SupportedTypeToOSGType(ESupportedType type) {
        return SupportedOSGTypes[static_cast<int>(type)];
    }
    static const char *SupportedTypeToName(ESupportedType type) {
        return SupportedTypeNames[static_cast<int>(type)];
    }
    static ESupportedType SupportedOSGTypeToType(osg::Uniform::Type osgType) {
        int i;
        for (i = 0; i < sizeof(SupportedOSGTypes) / sizeof(osg::Uniform::Type); ++i)
            if (osgType == SupportedOSGTypes[i])
                break;
        if (i == sizeof(SupportedTypeNames) / sizeof(const char *))
            --i;
        return static_cast<ESupportedType>(i);
    }
    static ESupportedType SupportedTypeNameToType(const char *name) {
        int i;
        for (i = 0; i < sizeof(SupportedTypeNames) / sizeof(const char *); ++i)
            if (std::strcmp(name, SupportedTypeNames[i]) == 0)
                break;
        if (i == sizeof(SupportedTypeNames) / sizeof(const char *))
            --i;
        return static_cast<ESupportedType>(i);
    }
};

} // namespace VIS4Earth

#endif // !VIS4EARTH_REFLECTABLE_H
