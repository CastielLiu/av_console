# 自动驾驶控制台
## dependencies
- tinyxml2 xml解析器

```bash
git clone https://github.com/leethomason/tinyxml2.git
cd tinyxml2 && sudo make install
```

```c++
#include<tinyxml2.h>
```


2020.6.25 添加传感器状态指示灯
1. 若传感器驱动为官方驱动则订阅传感器消息判断传感器是否正在工作，对于更新频率比较高的传感器，需要限定更新间隔以减少资源消耗
2. 若传感器驱动为用户自定义驱动，则定时发布自诊断信息，av_cosole订阅传感器诊断信息判断其是否正在工作
3. 当前传感器状态与上次状态不同时触发状态灯改变信号
4. 利用sensor_msgs::NavSatFix消息判断GPS和RTK的有效性，收到消息则认为GPS有效，gps_fix->status.status　== 10时认为RTK有效，10为自定义值，应与GPS驱动程序保持一致
5. 将所有传感器ID以枚举类型表示，触发状态变化信号时传递传感器ID和status,用同一信号和槽解决所有传感器状态传递问题
6. 建立传感器指针容器，方便循环检测传感器状态

