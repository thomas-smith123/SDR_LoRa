#include "spectrum_waterfall_widget.h"

#include "qcustomplot.h"

#include <QLabel>
#include <QPixmap>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <cstring>

SpectrumWaterfallWidget::SpectrumWaterfallWidget(QWidget *parent)
    : QWidget(parent)
{
    spectrumPlot_ = new QCustomPlot(this);
    waterfallLabel_ = new QLabel(this);

    spectrumPlot_->addGraph();
    spectrumPlot_->graph(0)->setPen(QPen(QColor(40, 130, 255)));
    spectrumPlot_->xAxis->setLabel("Frequency offset (kHz)");
    spectrumPlot_->yAxis->setLabel("Power (dB)");
    spectrumPlot_->yAxis->setRange(-100.0, 20.0);
    spectrumPlot_->setOpenGl(true);

    waterfallLabel_->setMinimumHeight(180);
    waterfallLabel_->setScaledContents(true);
    waterfallLabel_->setFrameShape(QFrame::StyledPanel);
    waterfallImage_ = QImage(fftPoints_, waterfallRows_, QImage::Format_RGB32);
    waterfallImage_.fill(Qt::black);
    waterfallLabel_->setPixmap(QPixmap::fromImage(waterfallImage_));

    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->addWidget(spectrumPlot_, 1);
    layout->addWidget(waterfallLabel_, 2);
    setLayout(layout);
    displayTimer_.start();
}

void SpectrumWaterfallWidget::configureDisplay(int maxDisplayFps, int snapshotPoints, int waterfallRows, int fftPoints)
{
    maxDisplayFps_ = std::max(1, maxDisplayFps);
    snapshotPoints_ = std::max(1024, snapshotPoints);
    waterfallRows_ = std::max(16, waterfallRows);
    fftPoints_ = std::max(128, fftPoints);
    waterfallImage_ = QImage(fftPoints_, waterfallRows_, QImage::Format_RGB32);
    waterfallImage_.fill(Qt::black);
    waterfallLabel_->setPixmap(QPixmap::fromImage(waterfallImage_));
}

void SpectrumWaterfallWidget::updateIqFrame(QVector<qint16> iSamples, QVector<qint16> qSamples, qint64 sampleRateHz)
{
    if (iSamples.isEmpty() || qSamples.isEmpty() || sampleRateHz <= 0) {
        return;
    }

    const qint64 minIntervalMs = 1000 / std::max(1, maxDisplayFps_);
    if (calculationPending_ || displayTimer_.elapsed() < minIntervalMs) {
        return;
    }

    calculationPending_ = true;
    displayTimer_.restart();

    if (iSamples.size() > snapshotPoints_) {
        iSamples = iSamples.mid(iSamples.size() - snapshotPoints_, snapshotPoints_);
        qSamples = qSamples.mid(qSamples.size() - snapshotPoints_, snapshotPoints_);
    }
    emit iqSnapshotReady(std::move(iSamples), std::move(qSamples), sampleRateHz);
}

void SpectrumWaterfallWidget::showSpectrum(QVector<double> spectrumDb, qint64 sampleRateHz)
{
    calculationPending_ = false;
    if (spectrumDb.isEmpty() || sampleRateHz <= 0) {
        return;
    }
    redrawSpectrum(spectrumDb, sampleRateHz);
}

void SpectrumWaterfallWidget::redrawSpectrum(const QVector<double>& spectrumDb, qint64 sampleRateHz)
{
    QVector<double> frequencyKhz(spectrumDb.size());
    const double fsKhz = static_cast<double>(sampleRateHz) / 1000.0;
    for (int n = 0; n < spectrumDb.size(); ++n) {
        frequencyKhz[n] = (static_cast<double>(n) / static_cast<double>(spectrumDb.size()) - 0.5) * fsKhz;
    }

    spectrumPlot_->graph(0)->setData(frequencyKhz, spectrumDb);
    spectrumPlot_->xAxis->setRange(-fsKhz / 2.0, fsKhz / 2.0);
    spectrumPlot_->yAxis->setRange(*std::min_element(spectrumDb.begin(), spectrumDb.end()) - 3.0,
                                   *std::max_element(spectrumDb.begin(), spectrumDb.end()) + 3.0);
    spectrumPlot_->replot(QCustomPlot::rpQueuedReplot);

    updateWaterfallImage(spectrumDb);
}

void SpectrumWaterfallWidget::updateWaterfallImage(const QVector<double>& spectrumDb)
{
    if (waterfallImage_.width() != spectrumDb.size() || waterfallImage_.height() != waterfallRows_) {
        waterfallImage_ = QImage(spectrumDb.size(), waterfallRows_, QImage::Format_RGB32);
        waterfallImage_.fill(Qt::black);
    } else if (waterfallImage_.height() > 1) {
        for (int row = waterfallImage_.height() - 1; row > 0; --row) {
            std::memcpy(waterfallImage_.scanLine(row), waterfallImage_.constScanLine(row - 1),
                        static_cast<size_t>(waterfallImage_.bytesPerLine()));
        }
    }

    auto *line = reinterpret_cast<QRgb*>(waterfallImage_.scanLine(0));
    for (int col = 0; col < spectrumDb.size(); ++col) {
        line[col] = waterfallColor(spectrumDb[col]);
    }
    waterfallLabel_->setPixmap(QPixmap::fromImage(waterfallImage_));
}

QRgb SpectrumWaterfallWidget::waterfallColor(double db)
{
    const double normalized = std::clamp((db + 110.0) / 80.0, 0.0, 1.0);
    const double r = std::clamp(1.5 - std::abs(4.0 * normalized - 3.0), 0.0, 1.0);
    const double g = std::clamp(1.5 - std::abs(4.0 * normalized - 2.0), 0.0, 1.0);
    const double b = std::clamp(1.5 - std::abs(4.0 * normalized - 1.0), 0.0, 1.0);
    return qRgb(static_cast<int>(r * 255.0), static_cast<int>(g * 255.0), static_cast<int>(b * 255.0));
}
