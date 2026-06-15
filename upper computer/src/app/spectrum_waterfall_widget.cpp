#include "spectrum_waterfall_widget.h"

#include "qcustomplot.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <cstring>

class WaterfallAxisWidget : public QWidget
{
public:
    explicit WaterfallAxisWidget(SpectrumWaterfallWidget *owner, QWidget *parent = nullptr)
        : QWidget(parent)
        , owner_(owner)
    {
        setMinimumHeight(210);
        setMouseTracking(true);
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter painter(this);
        painter.fillRect(rect(), QColor(12, 12, 16));

        const QRect plot = plotRect();
        painter.drawImage(plot, owner_->waterfallImage_);

        painter.setPen(QPen(QColor(170, 170, 170)));
        painter.drawRect(plot.adjusted(0, 0, -1, -1));

        const qint64 sampleRateHz = owner_->lastSampleRateHz_ > 0 ? owner_->lastSampleRateHz_ : 1000000;
        const double centerMHz = owner_->centerFrequencyHz_ > 0 ? static_cast<double>(owner_->centerFrequencyHz_) / 1.0e6 : 0.0;
        const double spanMHz = static_cast<double>(sampleRateHz) / 1.0e6;
        painter.setPen(QColor(220, 220, 220));
        painter.drawText(QRect(plot.left(), height() - 34, plot.width(), 18), Qt::AlignCenter, QStringLiteral("Frequency (MHz)"));
        painter.drawText(QRect(4, plot.top(), 18, plot.height()), Qt::AlignCenter, QStringLiteral("Time"));

        painter.setPen(QColor(150, 150, 150));
        for (int i = 0; i <= 4; ++i) {
            const int x = plot.left() + static_cast<int>(std::round(plot.width() * i / 4.0));
            painter.drawLine(x, plot.bottom(), x, plot.bottom() + 5);
            const double freqMHz = centerMHz + (static_cast<double>(i) / 4.0 - 0.5) * spanMHz;
            const QString label = owner_->centerFrequencyHz_ > 0
                                      ? QString::number(freqMHz, 'f', 3)
                                      : QStringLiteral("%1 k").arg(QString::number((static_cast<double>(i) / 4.0 - 0.5) * sampleRateHz / 1000.0, 'f', 0));
            painter.drawText(QRect(x - 42, plot.bottom() + 7, 84, 18), Qt::AlignCenter, label);
        }

        const double rowSeconds = 1.0 / std::max(1, owner_->maxDisplayFps_);
        const double historySeconds = rowSeconds * std::max(1, owner_->waterfallRows_ - 1);
        for (int i = 0; i <= 4; ++i) {
            const int y = plot.top() + static_cast<int>(std::round(plot.height() * i / 4.0));
            painter.drawLine(plot.left() - 5, y, plot.left(), y);
            const double secondsAgo = historySeconds * static_cast<double>(i) / 4.0;
            const QString label = i == 0 ? QStringLiteral("now") : QStringLiteral("-%1s").arg(secondsAgo, 0, 'f', 1);
            painter.drawText(QRect(20, y - 9, 46, 18), Qt::AlignRight | Qt::AlignVCenter, label);
        }

        if (dragging_) {
            painter.setPen(QPen(QColor(255, 220, 80), 1, Qt::DashLine));
            painter.drawLine(dragStart_.x(), plot.top(), dragStart_.x(), plot.bottom());
            painter.drawLine(lastPos_.x(), plot.top(), lastPos_.x(), plot.bottom());
        }
    }

    void mousePressEvent(QMouseEvent *event) override
    {
        if (event->button() != Qt::LeftButton || !plotRect().contains(event->pos())) {
            return;
        }
        dragging_ = true;
        dragStart_ = event->pos();
        lastPos_ = event->pos();
        update();
    }

    void mouseMoveEvent(QMouseEvent *event) override
    {
        if (!dragging_) {
            return;
        }
        lastPos_ = event->pos();
        update();
    }

    void mouseReleaseEvent(QMouseEvent *event) override
    {
        if (!dragging_ || event->button() != Qt::LeftButton) {
            return;
        }
        dragging_ = false;
        lastPos_ = event->pos();
        const QRect plot = plotRect();
        owner_->handleWaterfallDragFinished(lastPos_.x() - dragStart_.x(), plot.width());
        update();
    }

private:
    QRect plotRect() const
    {
        return rect().adjusted(70, 10, -15, -40);
    }

    SpectrumWaterfallWidget *owner_ = nullptr;
    bool dragging_ = false;
    QPoint dragStart_;
    QPoint lastPos_;
};

SpectrumWaterfallWidget::SpectrumWaterfallWidget(QWidget *parent)
    : QWidget(parent)
{
    spectrumPlot_ = new QCustomPlot(this);
    waterfallWidget_ = new WaterfallAxisWidget(this, this);

    spectrumPlot_->addGraph();
    spectrumPlot_->graph(0)->setPen(QPen(QColor(40, 130, 255)));
    spectrumPlot_->xAxis->setLabel("Frequency offset (kHz)");
    spectrumPlot_->yAxis->setLabel("Power (dB)");
    spectrumPlot_->yAxis->setRange(-100.0, 20.0);
    spectrumPlot_->setOpenGl(true);

    waterfallImage_ = QImage(fftPoints_, waterfallRows_, QImage::Format_RGB32);
    waterfallImage_.fill(Qt::black);

    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->addWidget(spectrumPlot_, 1);
    layout->addWidget(waterfallWidget_, 2);
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
    waterfallWidget_->update();
}

void SpectrumWaterfallWidget::configureSpectrumRange(double minDb, double maxDb)
{
    if (maxDb <= minDb) {
        return;
    }
    spectrumMinDb_ = minDb;
    spectrumMaxDb_ = maxDb;
    spectrumPlot_->yAxis->setRange(spectrumMinDb_, spectrumMaxDb_);
}

void SpectrumWaterfallWidget::setCenterFrequencyHz(qint64 centerFrequencyHz)
{
    centerFrequencyHz_ = centerFrequencyHz;
    waterfallWidget_->update();
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
    lastSampleRateHz_ = sampleRateHz;
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
    spectrumPlot_->yAxis->setRange(spectrumMinDb_, spectrumMaxDb_);
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
    waterfallWidget_->update();
}

void SpectrumWaterfallWidget::handleWaterfallDragFinished(int deltaPixels, int plotWidthPixels)
{
    if (plotWidthPixels <= 0 || lastSampleRateHz_ <= 0 || deltaPixels == 0) {
        return;
    }

    // 拖动含义：把图像向右拖，等价于中心频率向低频侧移动；灵敏度为当前可见带宽。
    const double deltaHz = -static_cast<double>(deltaPixels) / static_cast<double>(plotWidthPixels) * static_cast<double>(lastSampleRateHz_);
    emit centerFrequencyShiftRequested(deltaHz);
}

QRgb SpectrumWaterfallWidget::waterfallColor(double db)
{
    const double normalized = std::clamp((db + 110.0) / 80.0, 0.0, 1.0);
    const double r = std::clamp(1.5 - std::abs(4.0 * normalized - 3.0), 0.0, 1.0);
    const double g = std::clamp(1.5 - std::abs(4.0 * normalized - 2.0), 0.0, 1.0);
    const double b = std::clamp(1.5 - std::abs(4.0 * normalized - 1.0), 0.0, 1.0);
    return qRgb(static_cast<int>(r * 255.0), static_cast<int>(g * 255.0), static_cast<int>(b * 255.0));
}
