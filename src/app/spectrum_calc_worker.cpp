#include "spectrum_calc_worker.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {
constexpr double kAdcScale = 32768.0;
constexpr double kMinPower = 1e-12;
}

SpectrumCalcWorker::SpectrumCalcWorker(QObject *parent)
    : QObject(parent)
{
}

SpectrumCalcWorker::~SpectrumCalcWorker()
{
    if (fftPlan_) {
        fftw_destroy_plan(fftPlan_);
    }
    if (fftInput_) {
        fftw_free(fftInput_);
    }
    if (fftOutput_) {
        fftw_free(fftOutput_);
    }
}

void SpectrumCalcWorker::configure(int fftPoints)
{
    fftPoints_ = normalizeFftPoints(fftPoints);
}

void SpectrumCalcWorker::calculate(QVector<qint16> iSamples, QVector<qint16> qSamples, qint64 sampleRateHz)
{
    if (iSamples.isEmpty() || qSamples.isEmpty() || sampleRateHz <= 0) {
        return;
    }

    const QVector<double> spectrumDb = calculateSpectrum(iSamples, qSamples);
    if (!spectrumDb.isEmpty()) {
        emit spectrumReady(spectrumDb, sampleRateHz);
    }
}

QVector<double> SpectrumCalcWorker::calculateSpectrum(const QVector<qint16>& iSamples, const QVector<qint16>& qSamples) const
{
    const int available = std::min(iSamples.size(), qSamples.size());
    if (available <= 0) {
        return {};
    }

    ensurePlan();
    QVector<double> maxPower(fftPoints_, kMinPower);
    constexpr int kMaxFftWindowsPerFrame = 64;
    const int hop = std::max(fftPoints_ / 2, available / kMaxFftWindowsPerFrame);
    for (int start = 0; start < available; start += hop) {
        const int currentCount = std::min(fftPoints_, available - start);
        if (currentCount < std::min(fftPoints_ / 4, available)) {
            break;
        }

        std::memset(fftInput_, 0, sizeof(fftw_complex) * static_cast<size_t>(fftPoints_));
        for (int n = 0; n < currentCount; ++n) {
            const int src = start + n;
            const double phase = 2.0 * M_PI * static_cast<double>(n) / static_cast<double>(std::max(1, fftPoints_ - 1));
            const double hann = 0.5 - 0.5 * std::cos(phase);
            fftInput_[n][0] = hann * static_cast<double>(iSamples[src]) / kAdcScale;
            fftInput_[n][1] = hann * static_cast<double>(qSamples[src]) / kAdcScale;
        }

        fftw_execute(fftPlan_);
        const int half = fftPoints_ / 2;
        for (int n = 0; n < fftPoints_; ++n) {
            const int src = (n + half) % fftPoints_;
            const double real = fftOutput_[src][0];
            const double imag = fftOutput_[src][1];
            const double power = (real * real + imag * imag) / static_cast<double>(fftPoints_);
            maxPower[n] = std::max(maxPower[n], power);
        }
    }

    QVector<double> spectrumDb(fftPoints_);
    for (int n = 0; n < fftPoints_; ++n) {
        spectrumDb[n] = 10.0 * std::log10(std::max(maxPower[n], kMinPower));
    }
    return spectrumDb;
}

void SpectrumCalcWorker::ensurePlan() const
{
    if (fftPlan_ && plannedFftPoints_ == fftPoints_) {
        return;
    }

    if (fftPlan_) {
        fftw_destroy_plan(fftPlan_);
        fftPlan_ = nullptr;
    }
    if (fftInput_) {
        fftw_free(fftInput_);
        fftInput_ = nullptr;
    }
    if (fftOutput_) {
        fftw_free(fftOutput_);
        fftOutput_ = nullptr;
    }

    fftInput_ = static_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * static_cast<size_t>(fftPoints_)));
    fftOutput_ = static_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * static_cast<size_t>(fftPoints_)));
    fftPlan_ = fftw_plan_dft_1d(fftPoints_, fftInput_, fftOutput_, FFTW_FORWARD, FFTW_ESTIMATE);
    plannedFftPoints_ = fftPoints_;
}

int SpectrumCalcWorker::normalizeFftPoints(int points)
{
    if (points <= 0) {
        return 512;
    }
    return std::max(128, points);
}
