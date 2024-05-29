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
#include <vis4earth/tf_editor.h>

namespace VIS4Earth {

class VolumeComponent : public QtOSGReflectableWidget {
    Q_OBJECT

  public:
    VolumeComponent(bool keepCPUData = false, bool keepVolSmoothed = false,
                    QWidget *parent = nullptr)
        : keepCPUData(keepCPUData), keepVolSmoothed(keepVolSmoothed),
          QtOSGReflectableWidget(ui, parent) {
        connect(ui.pushButton_loadRAWVolume, &QPushButton::clicked, this,
                &VolumeComponent::loadRAWVolume);
        connect(ui.pushButton_loadTF, &QPushButton::clicked, this, &VolumeComponent::loadTF);

        auto changeVolID = [&](int idx) {
            if (GetVolumeTimeNumber(idx) == 0)
                return;

            auto vol = multiTimeVaryingVols[idx][0];
            ui.spinBox_voxPerVolX->setValue(vol->getImage()->s());
            ui.spinBox_voxPerVolY->setValue(vol->getImage()->t());
            ui.spinBox_voxPerVolZ->setValue(vol->getImage()->r());

            reinterpret_cast<QGridLayout *>(ui.groupBox_tf->layout())
                ->addWidget(&tfEditors[idx], 2, 0, 2, 2);
        };
        connect(ui.comboBox_currVolID, QOverload<int>::of(&QComboBox::currentIndexChanged),
                changeVolID);
        changeVolID(ui.comboBox_currVolID->currentIndex());

        connect(ui.comboBox_smoothType, QOverload<int>::of(&QComboBox::currentIndexChanged),
                [&](int) { resmoothVolume(); });
        connect(ui.comboBox_smoothDim, QOverload<int>::of(&QComboBox::currentIndexChanged),
                [&](int) { resmoothVolume(); });

        for (int i = 0; i < 2; ++i)
            connect(&tfEditors[i], &TransferFunctionEditor::TransferFunctionChanged, this,
                    &VolumeComponent::resampleTF);
    }

    const Ui::VolumeComponent &GetUI() const { return ui; }

    ESupportedVoxelType GetVoxelType() const { return ESupportedVoxelType::UInt8; }

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
    osg::ref_ptr<osg::Texture3D> GetVolumeSmoothed(uint32_t volID, uint32_t timeID) const {
        if (volID > 1 || timeID >= multiTimeVaryingVolSmootheds[volID].size())
            return nullptr;
        return multiTimeVaryingVolSmootheds[volID][timeID];
    }
    const RAWVolumeData &GetVolumeCPU(uint32_t volID, uint32_t timeID) const {
        if (volID > 1 || timeID >= multiTimeVaryingVolCPUs[volID].size())
            return {};
        return multiTimeVaryingVolCPUs[volID][timeID];
    }
    const RAWVolumeData &GetVolumeCPUSmoothed(uint32_t volID, uint32_t timeID) const {
        if (volID > 1 || timeID >= multiTimeVaryingVolCPUSmootheds[volID].size())
            return {};
        return multiTimeVaryingVolCPUSmootheds[volID][timeID];
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
    bool keepCPUData;
    bool keepVolSmoothed;

    Ui::VolumeComponent ui;
    std::array<TransferFunctionEditor, 2> tfEditors;
    std::array<osg::ref_ptr<osg::Texture1D>, 2> multiTFs;
    std::array<osg::ref_ptr<osg::Texture2D>, 2> multiTFPreInts;
    std::array<std::vector<osg::ref_ptr<osg::Texture3D>>, 2> multiTimeVaryingVols;
    std::array<std::vector<osg::ref_ptr<osg::Texture3D>>, 2> multiTimeVaryingVolSmootheds;
    std::array<std::vector<RAWVolumeData>, 2> multiTimeVaryingVolCPUs;
    std::array<std::vector<RAWVolumeData>, 2> multiTimeVaryingVolCPUSmootheds;

    void loadRAWVolume() {
        auto filePaths = QFileDialog::getOpenFileNames(this, tr("Open RAW Volume File"), "./",
                                                       tr("RAW Volume (*.bin;*.raw)"));
        if (filePaths.isEmpty())
            return;

        std::array<uint32_t, 3> voxPerVol = {ui.spinBox_voxPerVolX->value(),
                                             ui.spinBox_voxPerVolY->value(),
                                             ui.spinBox_voxPerVolZ->value()};

        auto volID = ui.comboBox_currVolID->currentIndex();
        auto &vols = multiTimeVaryingVols[volID];
        auto &volSmootheds = multiTimeVaryingVolSmootheds[volID];
        auto &volCPUs = multiTimeVaryingVolCPUs[volID];
        auto &volCPUSmootheds = multiTimeVaryingVolCPUSmootheds[volID];

        vols.clear();
        vols.reserve(filePaths.size());
        if (keepVolSmoothed) {
            volSmootheds.clear();
            volSmootheds.reserve(filePaths.size());
        }
        if (keepCPUData) {
            volCPUs.clear();
            volCPUs.reserve(filePaths.size());
        }
        if (keepCPUData && keepVolSmoothed) {
            volCPUSmootheds.clear();
            volCPUSmootheds.reserve(filePaths.size());
        }

        for (const auto &filePath : filePaths) {
            auto volDat = RAWVolumeData::LoadFromFile(RAWVolumeData::FromFileParameters{
                voxPerVol, ESupportedVoxelType::UInt8, filePath.toStdString()});
            if (!volDat.ok) {
                QMessageBox::warning(this, tr("Error"), tr(volDat.result.errMsg.c_str()));
                continue;
            }

            vols.emplace_back(volDat.result.dat.ToOSGTexture());
            if (keepCPUData)
                volCPUs.emplace_back(volDat.result.dat);

            if (keepVolSmoothed) {
                auto volDatSmoothed = volDat.result.dat.GetSmoothed(RAWVolumeData::SmoothParameters{
                    static_cast<RAWVolumeData::ESmoothType>(ui.comboBox_smoothType->currentIndex()),
                    static_cast<RAWVolumeData::ESmoothDimension>(
                        ui.comboBox_smoothDim->currentIndex())});
                volSmootheds.emplace_back(volDat.result.dat.ToOSGTexture());

                if (keepCPUData)
                    volCPUSmootheds.emplace_back(volDatSmoothed);
            }
        }

        emit VolumeChanged();
    }

    void resmoothVolume() {
        if (!keepCPUData || !keepVolSmoothed)
            return;

        auto resmooth = [&](uint32_t volID, uint32_t timeID) {
            multiTimeVaryingVolCPUSmootheds[volID][timeID] =
                multiTimeVaryingVolCPUs[volID][timeID].GetSmoothed(RAWVolumeData::SmoothParameters{
                    static_cast<RAWVolumeData::ESmoothType>(ui.comboBox_smoothType->currentIndex()),
                    static_cast<RAWVolumeData::ESmoothDimension>(
                        ui.comboBox_smoothDim->currentIndex())});
            multiTimeVaryingVolSmootheds[volID][timeID] =
                multiTimeVaryingVolCPUs[volID][timeID].ToOSGTexture();
        };

        for (uint32_t vi = 0; vi < 2; ++vi) {
            auto tNum = GetVolumeTimeNumber(vi);
            if (tNum == 0)
                continue;

            for (uint32_t ti = 0; ti < tNum; ++ti)
                resmooth(vi, ti);
        }

        emit VolumeChanged();
    }

    void loadTF() {
        auto filePath = QFileDialog::getOpenFileName(this, tr("Open TXT File"), "./",
                                                     tr("Transfer Function (*.txt)"));
        if (filePath.isEmpty())
            return;

        auto tfDat = TransferFunctionData::LoadFromFile(TransferFunctionData::FromFileParameters{
            TransferFunctionData::EFilterType::Linear, filePath.toStdString()});
        if (!tfDat.ok) {
            QMessageBox::warning(this, tr("Error"), tr(tfDat.result.errMsg.c_str()));
            return;
        }

        tfEditors[ui.comboBox_smoothType->currentIndex()].SetTransferFunctionData(tfDat.result.dat);
    }

    void resampleTF() {
        auto resample = [&](uint32_t volID) {
            multiTFs[volID] = tfEditors[volID].GetTransferFunctionData().ToOSGTexture();
            multiTFPreInts[volID] =
                tfEditors[volID].GetTransferFunctionData().ToPreIntegratedOSGTexture();
        };

        for (uint32_t vi = 0; vi < 2; ++vi)
            resample(vi);

        emit TransferFunctionChanged();
    }
};

} // namespace VIS4Earth

#endif // !VIS4EARTH_VOLUME_CMPT_H
