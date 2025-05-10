#include "metricCollector.h"

#ifdef METRIC_COLLECT
#include "radioInterface.h"

void SignalMetricsCollector::startCollection() {
    mIsCollecting = true;
    LOG(ALERT) << "startCollection startCollection startCollection startCollection";
    // Запуск потока сбора данных
}

void SignalMetricsCollector::collectFromRadioInterface(RadioInterface *radio, size_t chan) {
    if (!mIsCollecting) return;
    std::lock_guard<std::mutex> lock(mMetricsMutex);
    // Сбор метрик из RadioInterface
    mCurrentMetrics.rssi = radio->rssiOffset(chan);
    // mCurrentMetrics.rxGain = radio->getRxGain(chan);
    // Получение I/Q данных для созвездия
    // auto samples = radio->getLastSamples(chan);
    // if (samples) {
    //     mCurrentVisData.constellation.assign(samples->begin(),
    //                                          samples->end());
    // }
    LOG(ALERT) << "collectFromRadioInterface\t"<< " chan: "<< chan << " rssi: " << mCurrentMetrics.rssi << "\n";
}

// void SignalMetricsCollector::collectFromTransceiver(Transceiver *trx, size_t chan) {
    //     if (!mIsCollecting) return;
    //     std::lock_guard<std::mutex> lock(mMetricsMutex);
    //     // Сбор метрик из Transceiver
    //     mCurrentMetrics.rxDropEvents = trx->getCounters().rx_drop_events;
    //     mCurrentMetrics.rxDropSamples = trx->getCounters().rx_drop_samples;
    //     mCurrentMetrics.txUnderruns = trx->getCounters().tx_underruns;
    //     // Сбор данных для визуализации
    //     auto burst = trx->getCurrentBurst(chan);
    //     if (burst) {
    //         mCurrentVisData.rawSignal.assign(burst->begin(), burst->end());
    //     }
    // }

SignalMetricsCollector::NumericMetrics SignalMetricsCollector::getCurrentMetrics() {
    std::lock_guard<std::mutex> lock(mMetricsMutex);
    return mCurrentMetrics;
}

SignalMetricsCollector::VisualizationData SignalMetricsCollector::getCurrentVisData() {
    std::lock_guard<std::mutex> lock(mMetricsMutex);
    return mCurrentVisData;
}

#endif // METRIC_COLLECT