# 基于 stm32 与 jetson Nano 的机械臂抓取目标方案



<p>过去时间</p>
<div class="container">
<div class="skills bar1">80%</div>
</div>

<p>完成进度</p>
<div class="container">
<div class="skills bar2">70%</div>
</div>

## 机械臂

- 机械臂自动校准
- 机械臂控制

## Nano

- yolo优化
- 数据发送
- 开机自启脚本

## Stm32

### 路线设计
 - 先连接购物车
 - DCBA顺序抓取
 - 放好购物车
 - 让仓库抓一圈
 - 放好购物车
 - 
### 机器人状态设计
 - 1行驶中
 - 2购物车连接
 - 3停顿等待上位机信号
 - 4正在抓取
    - 一般货架
    - 仓库区
 - 5停放购物车