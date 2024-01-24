//
// Created by meng on 24-1-17.
//
#ifndef GTSAM_TEST_PRE_INTEGRATION_DATA_PREPARER_H
#define GTSAM_TEST_PRE_INTEGRATION_DATA_PREPARER_H

#include "type.h"

#include <mutex>
#include <deque>

/*!
 * 用于从队列中去除一个时间段内的数据
 */
class PreIntegrationDataPreparer {
public:
    PreIntegrationDataPreparer() = default;

    void CacheData(IMUData imu_data);

    std::vector<IMUData> GetDataSegment(uint64_t timestamp_left, uint64_t timestamp_right);

private:
    std::mutex mutex_;
    std::deque<IMUData> imu_data_deque_;
};

#endif //GTSAM_TEST_PRE_INTEGRATION_DATA_PREPARER_H
