# 模型预测控制(Model Predictive Control)

## 1.介绍

该仓库主要对MPC算法进行了一个简单地实现， 从而达到学习和理解MPC的目的。

对于MPC公式的推导和介绍，可以参考我的博客[《【附C++源代码】模型预测控制(MPC)公式推导以及算法实现，Model Predictive control介绍》](https://blog.csdn.net/u011341856/article/details/122799600)



## 2. 依赖库

**Eigen**

```shell
sudo apt-get install libeigen3-dev
```

**python**

```shell
sudo apt-get install python-matplotlib python-numpy python2.7-dev
```

## 3. 编译和运行

```shell
cd algorithm_learning/model_predictive_control
mkdir build
cd build
cmake ..
make 

# 运行
./model_predictive_control
```





