# Shopping_Car

## 机械臂

- 非传统连杆式机械臂，无法建立机械臂运动解算，只能通过初高中的知识对机械臂的几何分析，实现角度和位置的控制，最终封装为了一个输入(x,y)机械臂移动到某个点的函数。
- 机械臂的校准使用了python建立了一个伪上位机，可以通过方便的改变机械臂的状态从而更快速的调整机械臂参数，以此来校准机械臂。
- 机械臂动作采用缓慢改变pwm波的方式时间，具体的实现方式是通过一个定时器，将当前控制pwm波的通道值缓慢增加到预定值。
- 同时也写了一个机械臂仿真的程序来看预想和实际的区别。

## Nano

- 开机的自动启动脚本，这个是去除了sudo的密码实现的，不然还是要输入密码才能开始
- 需要自动开启tty1的权限许可
- 采用yolov5_6.1主要的原因在于这个版本可以直接导出为.engine文件，此格式为专门优化了tensorRT，对于jeston nano来说，大幅加快了运行速度，帧数近乎翻了一倍。(可惜的是，更快的帧数对实际效果没什么用，本来是想实现多帧融合提升准确性，但遇到无法区别此物品和上一帧此物品的id，很难实现；同时高帧率yo还想实现抓取过程中的实时反馈指导，但后续遇到靠近物品后无法识别到物品的问题，又取消)
- yolo的数据集是我直接用比赛用的小车仿实际场景拍摄的，一会拍一点，积累起来高达550张了(其中50张是比赛期间匆忙拍摄的)，我发现别的组很多拍摄方式都有问题，我的建议是尽可能还原实际，干扰物和目标并存。

## Stm32

### 路线设计
 - 先连接购物车
 - DCBA顺序抓取
 - 放好购物车
### 机器人状态设计
 - 1行驶中
 - 2购物车连接
 - 3停顿等待上位机信号
 - 4正在抓取
    - 一般货架
    - ~~仓库区~~
 - 5停放购物车

### 机器人函数设计

这里我有点累赘的写了大量或用得上，或用不上的函数。主要是基于之前的循迹挑战赛

#### 十字路口判定&旋转

思路流程图

![购物机器人.drawio](C:\Users\Arc\Desktop\购物机器人.drawio.svg)

#### 旋转

    流程图表述的已近较为清楚了
    可以根据需求，自己拓展出变种版本，如不涉及强制位置矫正的旋转，漂移式旋转，根据当前旋转角度和预设角度的pid旋转

#### 直线行驶
    根据amt14550传感器完成的循迹。同样也有多个变种，如：直走一格后退出，根据速度积分，直走固定的距离；取消白线循迹的盲走。
    后退则采用最基础的两个循迹模块，原因在于uart不够用了。也分为，退一格后退出，退一定距离。

#### 地图操作
    原先的设想是输入地图的位置，小车根据dfs深度优先搜索，自动行驶到指定位置。
    相关函数也已经也好，每次旋转和检测到白线信息后，更新小车位置坐标。采用dfs加某些非通行位置的判定实现。
    但最终也由于实际可操作性而放弃。
    最终采用的是将地图操作和目标操作位置放在一个数组中，根据当前已经过白线数量对应上地图操作实现(LOW).

## 无线调试

原定的计划是借助esp32来调试，但后续实现对我来说耗时较大。

采取蓝牙模块实现调试信息的无线接收。

采用usb拓展坞+usb延长线的方式，有效的加长了调试距离，同时连接电脑只需要一根线，优雅。

同时做了一个双头的uart线，实现了stm32既可以接受nano的uart信息，也可以接收电脑发过来的uart信息(python的pyserial模块)。
