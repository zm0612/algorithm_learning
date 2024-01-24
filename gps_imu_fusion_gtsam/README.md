# GPS IMU Fusion By Gtsam

## 1. 介绍
改仓库主要是基于Gtsam，实现了GPS与IMU的融合。通过使用预积分的方法，实现了IMU积分对前后帧的约束。通过边缘化操作限制了优化窗口中始终只包含前后两帧的数据。主要包含了如下内容：
- IMU预积分因子的使用
- GPS因子的使用
- 边缘化操作
- bias的估计

## 2. 编译
```shell
cd gps_imu_fusion_gtsam
mkdir build
cd build 
cmake ..
make -j4
```

## 3. 运行
```shell
cd gps_imu_fusion_gtsam/build
./gps_imu_fusion_gtsam
```

## 4. 效果可视化
```shell
cd gps_imu_fusion_gtsam/resule
python display_path.py
```

## 5. 总结
在IMU的预积分方式下，实现GPS与IMU的融合，其中对于第一帧数据应当设置先验，包括位姿、速度、bias，如果状态置信度较差，需要将噪声设置大一些。另外如果GPS只有位置观测，那么陀螺仪的bias可能会无法收敛，如果依然想要其bias收敛，需要增大滑动窗口的大小。

而对于有旋转的GPS观测，加速度计和陀螺仪的bias均可以实现较好的估计。