#ifndef SPECTRUM_WATERFALL_WIDGET_H
#define SPECTRUM_WATERFALL_WIDGET_H

#include <QVector>
#include <QWidget>
#include <QImage>

#include <QElapsedTimer>

class QCustomPlot;
class QLabel;

class SpectrumWaterfallWidget : public QWidget
{
    Q_OBJECT
public:
    explicit SpectrumWaterfallWidget(QWidget *parent = nullptr);
    void configureDisplay(int maxDisplayFps, int snapshotPoints, int waterfallRows, int fftPoints);

public slots:
    void updateIqFrame(QVector<qint16> iSamples, QVector<qint16> qSamples, qint64 sampleRateHz);
    void showSpectrum(QVector<double> spectrumDb, qint64 sampleRateHz);

signals:
    void iqSnapshotReady(QVector<qint16> iSamples, QVector<qint16> qSamples, qint64 sampleRateHz);

private:
    void redrawSpectrum(const QVector<double>& spectrumDb, qint64 sampleRateHz);
    void updateWaterfallImage(const QVector<double>& spectrumDb);
    static QRgb waterfallColor(double db);

    QCustomPlot *spectrumPlot_ = nullptr;
    QLabel *waterfallLabel_ = nullptr;

    int fftPoints_ = 1024;
    int waterfallRows_ = 80;
    int maxDisplayFps_ = 20;
    int snapshotPoints_ = 4096;
    bool calculationPending_ = false;
    QElapsedTimer displayTimer_;
    QImage waterfallImage_;
};

#endif // SPECTRUM_WATERFALL_WIDGET_H
