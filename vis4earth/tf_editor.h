#ifndef VIS4EARTH_TF_CMPT_H
#define VIS4EARTH_TF_CMPT_H

#include <array>
#include <unordered_map>
#include <vector>

#include <QtCharts/QAreaSeries>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>

#include <vis4earth/data/tf_data.h>

namespace VIS4Earth {

class TransferFunctionEditor : public QWidget {
    Q_OBJECT

  public:
    TransferFunctionEditor(QWidget *parent = nullptr) : QWidget(parent) {
        setLayout(new QGridLayout());
        auto gridLayout = reinterpret_cast<QGridLayout *>(layout());
        gridLayout->addWidget(&chartView, 0, 0, 2, 2);

        tfAreas = new QtCharts::QAreaSeries();
        tfLines = new QtCharts::QLineSeries();
        chartView.chart()->addSeries(tfLines);
        chartView.chart()->addSeries(tfAreas);
    }

    const TransferFunctionData &GetTransferFunctionData() const { return tfDat; }
    void SetTransferFunctionData(const TransferFunctionData &tfDat) {
        this->tfDat = tfDat;
        updateSeries();

        emit TransferFunctionChanged();
    }

  Q_SIGNALS:
    void TransferFunctionChanged();

  private:
    TransferFunctionData tfDat;

    QComboBox comboBox_tfPreset;
    QtCharts::QChartView chartView;

    QtCharts::QAreaSeries *tfAreas;
    QtCharts::QLineSeries *tfLines;
    std::unordered_map<uint8_t, QtCharts::QScatterSeries *> tfScatters;

    void updateSeries() {
        tfLines->clear();

        for (auto &scalar_rgba : tfDat.GetPoints()) {
            auto scalar = std::get<0>(scalar_rgba);

            auto itr = tfScatters.find(scalar);
            if (itr == tfScatters.end()) {
            }
        }
    }
};

} // namespace VIS4Earth

#endif // !VIS4EARTH_TF_CMPT_H
