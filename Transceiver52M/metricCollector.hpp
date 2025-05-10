#ifndef METRIC_COLECTOR_HPP
#define METRIC_COLECTOR_HPP

#include <atomic>
#include <vector>
#include <complex>
#include "Transceiver.h"
#include "radioInterface.h"

#define METRIC_COLLECT

#ifdef METRIC_COLLECT
// FIXME Вынести сбор метрик в отдельный поток
// FIXME Доабавить doxygen 
class SignalMetricsCollector {
   public:
    // Конструктор с настройками частоты сбора
    SignalMetricsCollector(unsigned int samplingRate = 500)
        : mSamplingRate(samplingRate), mIsCollecting(false) {}

    // Структура для хранения текстовых метрик
    struct NumericMetrics {
        double rssi;
        double snr;
        double rxGain;
        double txPower;
        unsigned long rxDropEvents;
        unsigned long rxDropSamples;
        unsigned long txUnderruns;
        double timing;
        double frequency;
        double channelQuality;
    };

    // Структура для визуализации
    struct VisualizationData {
        std::vector<std::complex<float>> channelResponse;
        std::vector<std::complex<float>> constellation;
        std::vector<std::complex<float>> rawSignal;
        std::vector<float> spectrum;
    };

   private:
    unsigned int mSamplingRate;  // Частота сбора данных в мс
    std::atomic<bool> mIsCollecting;
    std::mutex mMetricsMutex;
    NumericMetrics mCurrentMetrics;
    VisualizationData mCurrentVisData;
    // Буферы для накопления данных
    std::vector<NumericMetrics> mMetricsHistory;
    std::vector<VisualizationData> mVisDataHistory;

   public:
    // Методы управления сбором
    void startCollection() {
        mIsCollecting = true;
        // Запуск потока сбора данных
    }
    void stopCollection() { mIsCollecting = false; }

    // Методы сбора данных из разных источников
    void collectFromTransceiver(Transceiver *trx, size_t chan) {
        if (!mIsCollecting) return;
        std::lock_guard<std::mutex> lock(mMetricsMutex);
        // Сбор метрик из Transceiver
        mCurrentMetrics.rxDropEvents = trx->getCounters().rx_drop_events;
        mCurrentMetrics.rxDropSamples = trx->getCounters().rx_drop_samples;
        mCurrentMetrics.txUnderruns = trx->getCounters().tx_underruns;
        // Сбор данных для визуализации
        auto burst = trx->getCurrentBurst(chan);
        if (burst) {
            mCurrentVisData.rawSignal.assign(burst->begin(), burst->end());
        }
    }

    void collectFromRadioInterface(RadioInterface *radio, size_t chan) {
        if (!mIsCollecting) return;
        std::lock_guard<std::mutex> lock(mMetricsMutex);
        // Сбор метрик из RadioInterface
        mCurrentMetrics.rssi = radio->rssiOffset(chan);
        mCurrentMetrics.rxGain = radio->getRxGain(chan);
        // Получение I/Q данных для созвездия
        auto samples = radio->getLastSamples(chan);
        if (samples) {
            mCurrentVisData.constellation.assign(samples->begin(),
                                                 samples->end());
        }
        LOG(INFO) << "collectFromRadioInterface\t"<< " chan: "<< chan << " rssi: " << mCurrentMetrics.rssi << "\n";
    }

    // Получение текущих метрик
    NumericMetrics getCurrentMetrics() {
        std::lock_guard<std::mutex> lock(mMetricsMutex);
        return mCurrentMetrics;
    }
    VisualizationData getCurrentVisData() {
        std::lock_guard<std::mutex> lock(mMetricsMutex);
        return mCurrentVisData;
    }
};

#endif // METRIC_COLLECT
#endif // !METRIC_COLECTOR_HPP