#ifndef MISF_MAIN_WINDOW_H
#define MISF_MAIN_WINDOW_H

#include <memory>

#ifdef DEPLOY_ON_ZHONGDIAN15
#include <grid/ui/uicomctrl/ccusbasedlg.h>
#else
#include <QtWidgets/qwidget.h>
using CCusBaseDlg = QWidget;
#endif // DEPLOY_ON_ZHONGDIAN15

#include <QtCore/qstringlistmodel.h>
#include <QtWidgets/qcolordialog.h>
#include <QtWidgets/qinputdialog.h>

#include <ui_misf_main_window.h>
#include <vis4earth/io/tf_io.h>
#include <vis4earth/io/tf_osg_io.h>
#include <vis4earth/scalar_viser/multi_isosurfaces_renderer.h>

namespace Ui {
class MISFMainWindow;
}
class MISFMainWindow : public QWidget {
    Q_OBJECT

  private:
    std::shared_ptr<VIS4Earth::ScalarViser::MultiIsosurfacesRenderer> renderer;

    Ui::MISFMainWindow ui;
    std::vector<std::tuple<float, std::array<float, 4>>> sortedIsosurfs;

  public:
    MISFMainWindow(std::shared_ptr<VIS4Earth::ScalarViser::MultiIsosurfacesRenderer> renderer,
                   QWidget *parent = nullptr)
        : QWidget(parent), renderer(renderer) {
        ui.setupUi(this);

        {
            auto dt = renderer->GetDeltaT();
            ui.doubleSpinBox_DeltaT->setRange(dt * .1, dt * 10.);
            ui.doubleSpinBox_DeltaT->setSingleStep(dt * .1);
            ui.doubleSpinBox_DeltaT->setValue(dt);
        }
        ui.spinBox_MaxStepCnt->setValue(renderer->GetMaxStepCount());

        connect(ui.pushButton_AddIsosurf, &QPushButton::clicked, [&]() {
            bool ok;
            auto isoVal = QInputDialog::getDouble(this, tr("Input Isosurface Value"), tr("IsoVal"),
                                                  30., 0., 255., 2, &ok);
            if (!ok)
                return;

            auto color = QColorDialog::getColor(Qt::white, this, tr("Input Isosurface Color"),
                                                QColorDialog::ShowAlphaChannel);
            if (!color.isValid())
                return;

            sortedIsosurfs.emplace_back(static_cast<float>(isoVal / 255.),
                                        std::array<float, 4>{static_cast<float>(color.redF()),
                                                             static_cast<float>(color.greenF()),
                                                             static_cast<float>(color.blueF()),
                                                             static_cast<float>(color.alphaF())});

            updateIsosurfacesList();
            updateRenderer();
        });
        connect(ui.pushButton_EditIsosurf, &QPushButton::clicked, [&]() {
            auto idx = ui.listWidget_Isosurfs->currentIndex();
            if (!idx.isValid())
                return;

            auto &col = std::get<1>(sortedIsosurfs[idx.row()]);
            QColor qtCol(255.f * col[0], 255.f * col[1], 255.f * col[2], 255.f * col[3]);
            auto newCol = QColorDialog::getColor(qtCol, this, tr("Input Isosurface Color"),
                                                 QColorDialog::ShowAlphaChannel);
            if (!newCol.isValid())
                return;

            col[0] = newCol.redF();
            col[1] = newCol.greenF();
            col[2] = newCol.blueF();
            col[3] = newCol.alphaF();

            updateIsosurfacesList();
            updateRenderer();
        });
        connect(ui.pushButton_DelIsosurf, &QPushButton::clicked, [&]() {
            auto idx = ui.listWidget_Isosurfs->currentIndex();
            if (!idx.isValid())
                return;

            auto itr = sortedIsosurfs.begin();
            std::advance(itr, idx.row());

            sortedIsosurfs.erase(itr);

            updateIsosurfacesList();
            updateRenderer();
        });

        connect(ui.checkBox_UseSmoothedVol, &QCheckBox::stateChanged, this,
                &MISFMainWindow::updateRenderer);

        connect(ui.doubleSpinBox_DeltaT,
                static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,
                &MISFMainWindow::updateRenderer);
        connect(ui.spinBox_MaxStepCnt,
                static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this,
                &MISFMainWindow::updateRenderer);

        connect(ui.checkBox_UseShading, &QCheckBox::stateChanged, [&](int state) {
            if (state == Qt::Checked) {
                ui.doubleSpinBox_Ka->setEnabled(true);
                ui.doubleSpinBox_Kd->setEnabled(true);
                ui.doubleSpinBox_Ks->setEnabled(true);
                ui.doubleSpinBox_Shininess->setEnabled(true);
                ui.doubleSpinBox_LightPosLon->setEnabled(true);
                ui.doubleSpinBox_LightPosLat->setEnabled(true);
                ui.doubleSpinBox_LightPosH->setEnabled(true);

                updateRenderer();
            } else {
                ui.doubleSpinBox_Ka->setEnabled(false);
                ui.doubleSpinBox_Kd->setEnabled(false);
                ui.doubleSpinBox_Ks->setEnabled(false);
                ui.doubleSpinBox_Shininess->setEnabled(false);
                ui.doubleSpinBox_LightPosLon->setEnabled(false);
                ui.doubleSpinBox_LightPosLat->setEnabled(false);
                ui.doubleSpinBox_LightPosH->setEnabled(false);

                updateRenderer();
            }
        });
        connect(ui.doubleSpinBox_Ka,
                static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,
                &MISFMainWindow::updateRenderer);
        connect(ui.doubleSpinBox_Kd,
                static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,
                &MISFMainWindow::updateRenderer);
        connect(ui.doubleSpinBox_Ks,
                static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,
                &MISFMainWindow::updateRenderer);
        connect(ui.doubleSpinBox_Shininess,
                static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,
                &MISFMainWindow::updateRenderer);
        connect(ui.doubleSpinBox_LightPosLon,
                static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,
                &MISFMainWindow::updateRenderer);
        connect(ui.doubleSpinBox_LightPosLat,
                static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,
                &MISFMainWindow::updateRenderer);
        connect(ui.doubleSpinBox_LightPosH,
                static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,
                &MISFMainWindow::updateRenderer);
    }

    void UpdateFromRenderer() {
        if (renderer->GetVolumeNum() == 0)
            return;

        auto bgn = renderer->GetVolumes().begin();
        sortedIsosurfs = bgn->second.GetIsosurfaces();
        updateIsosurfacesList();

        auto lonRng = bgn->second.GetLongtituteRange();
        auto latRng = bgn->second.GetLatituteRange();
        auto hRng = bgn->second.GetHeightFromCenterRange();
        for (uint8_t i = 0; i < 2; ++i) {
            lonRng[i] = rad2Deg(lonRng[i]);
            latRng[i] = rad2Deg(latRng[i]);
            hRng[i] = rad2Deg(hRng[i]);
        }
        lonRng[0] = std::max(lonRng[0] - 30.f, -180.f);
        lonRng[1] = std::min(lonRng[1] + 30.f, 180.f);
        latRng[0] = std::max(latRng[0] - 30.f, -90.f);
        latRng[1] = std::min(latRng[1] + 30.f, +90.f);
        hRng[1] *= 2.f;

        ui.doubleSpinBox_LightPosLon->setMinimum(lonRng[0]);
        ui.doubleSpinBox_LightPosLon->setMaximum(lonRng[1]);
        ui.doubleSpinBox_LightPosLon->setValue(.5f * (lonRng[0] + lonRng[1]));
        ui.doubleSpinBox_LightPosLon->setSingleStep(10.f);

        ui.doubleSpinBox_LightPosLat->setMinimum(latRng[0]);
        ui.doubleSpinBox_LightPosLat->setMaximum(latRng[1]);
        ui.doubleSpinBox_LightPosLat->setValue(.5f * (latRng[0] + latRng[1]));
        ui.doubleSpinBox_LightPosLat->setSingleStep(10.f);

        ui.doubleSpinBox_LightPosH->setMinimum(hRng[0]);
        ui.doubleSpinBox_LightPosH->setMaximum(hRng[1]);
        ui.doubleSpinBox_LightPosH->setValue(.5f * (hRng[0] + hRng[1]));
        ui.doubleSpinBox_LightPosH->setSingleStep(.1f * (hRng[1] - hRng[0]));
    }

  private:
    void updateRenderer() {
        auto &vols = renderer->GetVolumes();
        for (auto itr = vols.begin(); itr != vols.end(); ++itr) {
            itr->second.SetIsosurfaces(sortedIsosurfs);
            itr->second.SetUseSmoothedVolume(ui.checkBox_UseSmoothedVol->isChecked());
        }

        renderer->SetDeltaT(ui.doubleSpinBox_DeltaT->value());
        renderer->SetMaxStepCount(ui.spinBox_MaxStepCnt->value());

        VIS4Earth::ScalarViser::MultiIsosurfacesRenderer::ShadingParam shadingParam;
        shadingParam.useShading = ui.checkBox_UseShading->isChecked();
        shadingParam.ka = ui.doubleSpinBox_Ka->value();
        shadingParam.kd = ui.doubleSpinBox_Kd->value();
        shadingParam.ks = ui.doubleSpinBox_Ks->value();
        shadingParam.shininess = ui.doubleSpinBox_Shininess->value();
        {
            auto lon = deg2Rad(ui.doubleSpinBox_LightPosLon->value());
            auto lat = deg2Rad(ui.doubleSpinBox_LightPosLat->value());
            auto h = ui.doubleSpinBox_LightPosH->value();

            shadingParam.lightPos.z() = h * sinf(lat);
            h = h * cosf(lat);
            shadingParam.lightPos.y() = h * sinf(lon);
            shadingParam.lightPos.x() = h * cosf(lon);
        }
        renderer->SetShading(shadingParam);
    }
    void updateIsosurfacesList() {
        std::sort(sortedIsosurfs.begin(), sortedIsosurfs.end(),
                  [](const decltype(sortedIsosurfs)::value_type &a,
                     const decltype(sortedIsosurfs)::value_type &b) -> bool {
                      return std::get<0>(a) < std::get<0>(b);
                  });

        ui.listWidget_Isosurfs->clear();

        QImage img(25, 25, QImage::Format_RGB32);
        for (auto &isosurf : sortedIsosurfs) {
            auto isoVal = std::get<0>(isosurf) * 255.f;
            auto &col = std::get<1>(isosurf);

            auto text = QString("%1 -> (%2, %3, %4, %5)")
                            .arg(static_cast<double>(isoVal), 0, 'f', 2)
                            .arg(static_cast<double>(col[0]), 0, 'f', 2)
                            .arg(static_cast<double>(col[1]), 0, 'f', 2)
                            .arg(static_cast<double>(col[2]), 0, 'f', 2)
                            .arg(static_cast<double>(col[3]), 0, 'f', 2);
            img.fill(QColor(255.f * col[0], 255.f * col[1], 255.f * col[2], 255.f * col[3]));

            auto listItem = new QListWidgetItem(QIcon(QPixmap::fromImage(img)), text);
            ui.listWidget_Isosurfs->addItem(listItem);
        }
    }

    float deg2Rad(float deg) { return deg * osg::PI / 180.f; };
    float rad2Deg(float rad) { return rad * 180.f / osg::PI; }
};

#endif
