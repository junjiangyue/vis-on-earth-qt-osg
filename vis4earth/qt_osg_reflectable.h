#ifndef VIS4EARTH_QT_OSG_REFLECTOR_H
#define VIS4EARTH_QT_OSG_REFLECTOR_H

#include <functional>
#include <memory>

#include <tuple>
#include <unordered_map>
#include <vector>

#include <QtCore/QDebug>
#ifdef DEPLOY_ON_ZHONGDIAN15
#include <grid/ui/uicomctrl/ccusbasedlg.h>
#else
#include <QtWidgets/QWidget>
using CCusBaseDlg = QWidget;
#endif // DEPLOY_ON_ZHONGDIAN15
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QSpinBox>

#include <vis4earth/reflectable.h>
#include <vis4earth/util.h>

namespace VIS4Earth {

class Property {
  public:
    Property(const char *propName, Reflectable::Type defVal) {
        switch (defVal.type) {
        case Reflectable::ESupportedType::Int:
            uniform = new osg::Uniform(propName, defVal.val.asInt);
            break;
        case Reflectable::ESupportedType::Bool:
            uniform = new osg::Uniform(propName, defVal.val.asBool);
            break;
        case Reflectable::ESupportedType::Float:
            uniform = new osg::Uniform(propName, defVal.val.asFloat);
            break;
        case Reflectable::ESupportedType::Double:
            uniform = new osg::Uniform(propName, defVal.val.asDouble);
            break;
        default:
            assert(false);
        }
    }

    osg::Uniform *GetUniform() const { return uniform.get(); }

    void SetConvertor(std::function<Reflectable::Type(const Reflectable::Type &valIn)> convertor) {
        this->convertor = convertor;

        // 若更新转换器，应更新对应的属性值
        auto val = GetValue();
        SetValue(val);
    }
    void SetValue(const Reflectable::Type &valIn) {
        auto val = valIn;
        if (convertor)
            val = convertor(val);

        switch (val.type) {
        case Reflectable::ESupportedType::Int:
            uniform->set(val.val.asInt);
            break;
        case Reflectable::ESupportedType::Bool:
            uniform->set(val.val.asBool);
            break;
        case Reflectable::ESupportedType::Float:
            uniform->set(val.val.asFloat);
            break;
        case Reflectable::ESupportedType::Double:
            uniform->set(val.val.asDouble);
            break;
        default:
            assert(false);
        }
    }
    Reflectable::Type GetValue() const {
        Reflectable::Type ret;
        ret.type = Reflectable::SupportedOSGTypeToType(GetOSGType());

        switch (ret.type) {
        case Reflectable::ESupportedType::Int:
            uniform->get(ret.val.asInt);
            break;
        case Reflectable::ESupportedType::Bool:
            uniform->get(ret.val.asBool);
            break;
        case Reflectable::ESupportedType::Float:
            uniform->get(ret.val.asFloat);
            break;
        case Reflectable::ESupportedType::Double:
            uniform->get(ret.val.asDouble);
            break;
        default:
            assert(false);
        }

        return ret;
    }
    osg::Uniform::Type GetOSGType() const { return uniform->getType(); }
    template <typename ValTy> ValTy GetOSGValue() const {
        ValTy ret;
        uniform->get(ret);
        return ret;
    }

  private:
    osg::ref_ptr<osg::Uniform> uniform;
    std::function<Reflectable::Type(const Reflectable::Type &valIn)> convertor;
};

template <typename WidgetTy, typename ValTy, typename SignalParamTy, typename DefValTy,
          void (WidgetTy::*SignalMemPtr)(SignalParamTy),
          DefValTy (WidgetTy::*GetDefValMemPtr)(void) const>
struct SyncFromQtToOSGWhenSignaled {
    std::shared_ptr<Property> operator()(QWidget *wdgt, const char *propName) {
        auto from = reinterpret_cast<WidgetTy *>(wdgt);
        Reflectable::Type defVal = static_cast<ValTy>((from->*GetDefValMemPtr)());
        auto prop = std::make_shared<Property>(propName, defVal);

        QObject::connect(from, SignalMemPtr, [prop](SignalParamTy sigVal) {
            Reflectable::Type val(static_cast<ValTy>(sigVal));
            prop->SetValue(val);
        });

        return prop;
    }
};

class QtOSGReflectableWidget : public CCusBaseDlg {
    Q_OBJECT

  public:
    template <typename UITy>
    explicit QtOSGReflectableWidget(UITy &ui, QWidget *parent = nullptr) : CCusBaseDlg(parent) {
        ui.setupUi(this);

        syncPropertiesFromQtToOSG(this);
    }

    bool SetPropertyValue(const std::string &propName, const Reflectable::Type &defVal) {
        auto itr = properties.find(propName);
        if (itr == properties.end())
            return false;

        itr->second->SetValue(defVal);
        return true;
    }
    Optional<Reflectable::Type> GetPropertyValue(const std::string &propName) const {
        auto itr = properties.find(propName);
        if (itr == properties.end())
            return {};
        return itr->second->GetValue();
    }
    template <typename ValTy>
    Optional<ValTy> GetPropertyOSGValue(const std::string &propName) const {
        auto itr = properties.find(propName);
        if (itr == properties.end())
            return {};
        return itr->second->GetOSGValue<ValTy>();
    }
    template <typename FuncTy> void ForEachProperty(FuncTy fn) const {
        for (const auto &name_prop : properties)
            fn(name_prop.first, *name_prop.second);
    }

  protected:
    std::unordered_map<std::string, std::shared_ptr<Property>> properties;

    void debugProperties(const std::vector<const QtOSGReflectableWidget *> &wdgts) {
        for (auto wdgt : wdgts)
            wdgt->ForEachProperty([&](const std::string &name, const Property &prop) {
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

  private:
    void syncPropertiesFromQtToOSG(QWidget *wdgt) {
        auto pos = wdgt->objectName().indexOf("VIS4EarthReflectable");
        // 可同步UI属性的名称应为：<控件类型>_<成员名>_<成员类型>_VIS4EarthReflectable
        if (pos != -1) {
            auto strs = wdgt->objectName().split(QChar('_'));

            auto propTy = Reflectable::SupportedTypeNameToType(strs[2].toStdString().c_str());
            auto propName = strs[1].toStdString();

            auto sync = [&]() -> std::function<std::shared_ptr<Property>(QWidget *, const char *)> {
                if (strs[0] == "checkBox")
                    if (propTy == Reflectable::ESupportedType::Bool)
                        return SyncFromQtToOSGWhenSignaled<QCheckBox, bool, int, Qt::CheckState,
                                                           &QCheckBox::stateChanged,
                                                           &QCheckBox::checkState>();
                    else
                        assert(false);
                else if (strs[0] == "spinBox")
                    if (propTy == Reflectable::ESupportedType::Int)
                        return SyncFromQtToOSGWhenSignaled<
                            QSpinBox, int, int, int, &QSpinBox::valueChanged, &QSpinBox::value>();
                    else
                        assert(false);
                else if (strs[0] == "doubleSpinBox")
                    if (propTy == Reflectable::ESupportedType::Float)
                        return SyncFromQtToOSGWhenSignaled<QDoubleSpinBox, float, double, double,
                                                           &QDoubleSpinBox::valueChanged,
                                                           &QDoubleSpinBox::value>();
                    else if (propTy == Reflectable::ESupportedType::Double)
                        return SyncFromQtToOSGWhenSignaled<QDoubleSpinBox, double, double, double,
                                                           &QDoubleSpinBox::valueChanged,
                                                           &QDoubleSpinBox::value>();
                    else
                        assert(false);
                else if (strs[0] == "comboBox")
                    if (propTy == Reflectable::ESupportedType::Int)
                        return SyncFromQtToOSGWhenSignaled<QComboBox, int, int, int,
                                                           &QComboBox::currentIndexChanged,
                                                           &QComboBox::currentIndex>();
                    else
                        assert(false);
                else
                    assert(false);
            }();
            auto prop = sync(wdgt, propName.c_str());

            properties.emplace(std::piecewise_construct, std::forward_as_tuple(propName),
                               std::forward_as_tuple(prop));
        }

        for (const auto child : wdgt->children())
            syncPropertiesFromQtToOSG(reinterpret_cast<QWidget *>(child));
    }
};

} // namespace VIS4Earth

#endif // !VIS4EARTH_QT_OSG_REFLECTOR_H
