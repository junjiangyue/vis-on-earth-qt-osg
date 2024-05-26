#ifndef VIS4EARTH_VOLUME_CMPT_H
#define VIS4EARTH_VOLUME_CMPT_H

#include <ui_volume_cmpt.h>

#include <vector>

#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>

#include <osg/Texture1D>
#include <osg/Texture2D>
#include <osg/Texture3D>

#include <vis4earth/data/tf_data.h>
#include <vis4earth/data/vol_data.h>
#include <vis4earth/math.h>
#include <vis4earth/qt_osg_reflectable.h>

namespace Ui {
class VolumeComponent;
}

namespace VIS4Earth {

class VolumeComponent : public QtOSGReflectableWidget {
    Q_OBJECT

  public:
    VolumeComponent(QWidget *parent = nullptr) : QtOSGReflectableWidget(ui, parent) {
        connect(ui.pushButton_LoadRAWVolume, &QPushButton::clicked, this,
                &VolumeComponent::loadRAWVolume);
        connect(ui.pushButton_LoadTF, &QPushButton::clicked, this, &VolumeComponent::loadTF);
    }

    const Ui::VolumeComponent &GetUI() const { return ui; }

    uint32_t GetVolumeTimeNumber(uint32_t volID) const {
        if (volID > 1)
            return 0;
        return multiTimeVaryingVols[volID].size();
    }
    osg::ref_ptr<osg::Texture3D> GetVolume(uint32_t volID, uint32_t timeID) const {
        if (volID > 1 || timeID >= multiTimeVaryingVols[volID].size())
            return nullptr;
        return multiTimeVaryingVols[volID][timeID];
    }
    osg::ref_ptr<osg::Texture1D> GetTransferFunction(uint32_t volID) const {
        if (volID > 1)
            return nullptr;
        return multiTFs[volID];
    }
    osg::ref_ptr<osg::Texture2D> GetPreIntegratedTransferFunction(uint32_t volID) const {
        if (volID > 1)
            return nullptr;
        return multiTFPreInts[volID];
    }

  Q_SIGNALS:
    void VolumeChanged();
    void TransferFunctionChanged();

  private:
    Ui::VolumeComponent ui;
    std::array<osg::ref_ptr<osg::Texture1D>, 2> multiTFs;
    std::array<osg::ref_ptr<osg::Texture2D>, 2> multiTFPreInts;
    std::array<std::vector<osg::ref_ptr<osg::Texture3D>>, 2> multiTimeVaryingVols;

    void loadRAWVolume() {
        auto filePaths = QFileDialog::getOpenFileNames(this, tr("Open RAW Volume File"), "./",
                                                       tr("RAW Volume (*.bin;*.raw)"));
        if (filePaths.isEmpty())
            return;

        std::array<uint32_t, 3> voxPerVol = {ui.spinBox_voxPerVolX->value(),
                                             ui.spinBox_voxPerVolY->value(),
                                             ui.spinBox_voxPerVolZ->value()};

        auto &vols = multiTimeVaryingVols[ui.comboBox_currVolID->currentIndex()];
        vols.clear();
        vols.reserve(filePaths.size());
        for (const auto &filePath : filePaths) {
            auto volDat = RAWVolumeData::LoadFromFile(RAWVolumeData::FromFileParameters{
                voxPerVol, ESupportedVoxelType::UInt8, filePath.toStdString()});
            if (!volDat.ok) {
                QMessageBox::warning(this, tr("Error"), tr(volDat.result.errMsg.c_str()));
                continue;
            }

            vols.emplace_back(volDat.result.dat.ToOSGTexture());
        }

        emit VolumeChanged();
    }
    void loadTF() {
        auto filePath = QFileDialog::getOpenFileName(this, tr("Open TXT File"), "./",
                                                     tr("Transfer Function (*.txt)"));
        if (filePath.isEmpty())
            return;

        auto tfDat = TransferFunctionData::LoadFromFile(
            TransferFunctionData::FromFileParameters{filePath.toStdString()});
        if (!tfDat.ok) {
            QMessageBox::warning(this, tr("Error"), tr(tfDat.result.errMsg.c_str()));
            return;
        }

        multiTFs[ui.comboBox_currVolID->currentIndex()] = tfDat.result.dat.ToOSGTexture();
        multiTFPreInts[ui.comboBox_currVolID->currentIndex()] =
            tfDat.result.dat.ToPreIntegratedOSGTexture();

        emit TransferFunctionChanged();
    }
};

} // namespace VIS4Earth

#endif // !VIS4EARTH_VOLUME_CMPT_H
