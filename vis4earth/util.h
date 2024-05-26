#ifndef VIS4EARTH_UTIL_H
#define VIS4EARTH_UTIL_H

#include <string>

#ifdef DEPLOY_ON_ZHONGDIAN15
#include <grid/common/base/AppEnv.h>
#endif // DEPLOY_ON_ZHONGDIAN15

namespace VIS4Earth {

template <typename T> struct Optional {
    bool ok;
    T val;

    Optional() : ok(false) {}
    Optional(const T &val) : ok(true), val(val) {}
};

template <typename T> struct ReteurnOrError {
    bool ok;
    union Result {
        std::string errMsg;
        T dat;

        Result(const char *errMsg) { new (&this->errMsg) std::string(errMsg); }
        Result(const T &dat) { new (&this->dat) T(dat); }
        ~Result() {}
    } result;

    ReteurnOrError(const char *errMsg) : ok(false), result(errMsg) {}
    ReteurnOrError(const T &dat) : ok(true), result(dat) {}
    ReteurnOrError(const ReteurnOrError &other) {
        ~ReteurnOrError();
        ok = other.ok;
        if (ok)
            result = other.result;
        else
            result = other.result.errMsg.c_str();
    }
    ~ReteurnOrError() {
        if (ok)
            result.dat.~T();
        else
            result.errMsg.~basic_string();
    }
};

inline std::string GetDataPathPrefix() {
#ifdef DEPLOY_ON_ZHONGDIAN15
    using namespace grid::common;

    auto p = AppEnv::instance().getPath(AppEnv::DATA_DIR);
    return std::string(p.c_str()) + "/";
#else
    return ""; // 使用宏绝对路径前缀，见CMakeLists.txt
#endif
}

} // namespace VIS4Earth

#endif // !VIS4EARTH_UTIL_H
