#ifndef SPECTRUM_CALC_WORKER_H
#define SPECTRUM_CALC_WORKER_H

#include <QObject>
#include <QVector>

#include "fftw3.h"

class SpectrumCalcWorker : public QObject
{
    Q_OBJECT
public:
    explicit SpectrumCalcWorker(QObject *parent = nullptr);
    ~SpectrumCalcWorker();
    void configure(int fftPoints);

public slots:
    void calculate(QVector<qint16> iSamples, QVector<qint16> qSamples, qint64 sampleRateHz);

signals:
    void spectrumReady(QVector<double> spectrumDb, qint64 sampleRateHz);

private:
    QVector<double> calculateSpectrum(const QVector<qint16>& iSamples, const QVector<qint16>& qSamples) const;
    void ensurePlan() const;
    static int normalizeFftPoints(int points);

    int fftPoints_ = 512;
    mutable int plannedFftPoints_ = 0;
    mutable fftw_complex *fftInput_ = nullptr;
    mutable fftw_complex *fftOutput_ = nullptr;
    mutable fftw_plan fftPlan_ = nullptr;
};

#endif // SPECTRUM_CALC_WORKER_H
