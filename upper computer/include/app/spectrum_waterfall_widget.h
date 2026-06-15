#ifndef SPECTRUM_WATERFALL_WIDGET_H
#define SPECTRUM_WATERFALL_WIDGET_H

#include <QVector>
#include <QWidget>
#include <QImage>

#include <QPoint>

#include <QElapsedTimer>

class QCustomPlot;
class WaterfallAxisWidget;

class SpectrumWaterfallWidget : public QWidget
{
    Q_OBJECT
public:
    explicit SpectrumWaterfallWidget(QWidget *parent = nullptr);
    void configureDisplay(int maxDisplayFps, int snapshotPoints, int waterfallRows, int fftPoints);
    void configureSpectrumRange(double minDb, double maxDb);
    void setCenterFrequencyHz(qint64 centerFrequencyHz);

public slots:
    void updateIqFrame(QVector<qint16> iSamples, QVector<qint16> qSamples, qint64 sampleRateHz);
    void showSpectrum(QVector<double> spectrumDb, qint64 sampleRateHz);

signals:
    void iqSnapshotReady(QVector<qint16> iSamples, QVector<qint16> qSamples, qint64 sampleRateHz);
    void centerFrequencyShiftRequested(double deltaHz);

private:
    friend class WaterfallAxisWidget;

    void redrawSpectrum(const QVector<double>& spectrumDb, qint64 sampleRateHz);
    void updateWaterfallImage(const QVector<double>& spectrumDb);
    void handleWaterfallDragFinished(int deltaPixels, int plotWidthPixels);
    static QRgb waterfallColor(double db);

    QCustomPlot *spectrumPlot_ = nullptr;
    WaterfallAxisWidget *waterfallWidget_ = nullptr;

    int fftPoints_ = 1024;
    int waterfallRows_ = 80;
    int maxDisplayFps_ = 20;
    int snapshotPoints_ = 4096;
    double spectrumMinDb_ = -120.0;
    double spectrumMaxDb_ = 20.0;
    qint64 lastSampleRateHz_ = 0;
    qint64 centerFrequencyHz_ = 0;
    bool calculationPending_ = false;
    QElapsedTimer displayTimer_;
    QImage waterfallImage_;
};

#endif // SPECTRUM_WATERFALL_WIDGET_H
