
#define METRIC_COLLECT

#ifndef METRIC_COLECTOR_HPP
#define METRIC_COLECTOR_HPP

#include <atomic>
#include <mutex>
#include <vector>
#include <complex>
#include <cstddef>
#include <iostream>
// #include "Transceiver.h"
// #include "radioInterface.h"

class RadioInterface;

#ifdef METRIC_COLLECT
// FIXME Вынести сбор метрик в отдельный поток
// FIXME Доабавить doxygen 
class SignalMetricsCollector {
   public:
    // Конструктор с настройками частоты сбора
    SignalMetricsCollector(unsigned int samplingRate = 500)
        : mSamplingRate(samplingRate), mIsCollecting(false) {
            std::cerr << "INIT INIT INIT\ncollectFromRadioInterface\n";
        }

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
    void startCollection();
    void stopCollection() { mIsCollecting = false; }

    // Методы сбора данных из разных источников
    // void collectFromTransceiver(Transceiver *trx, size_t chan);

    void collectFromRadioInterface(RadioInterface *radio, size_t chan);

    // Получение текущих метрик
    NumericMetrics getCurrentMetrics();
    
    VisualizationData getCurrentVisData();
};

// SignalMetricsCollector *metricsCollector;

#endif // METRIC_COLLECT
#endif // !METRIC_COLECTOR_HPP