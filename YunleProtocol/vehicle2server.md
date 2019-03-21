## smartcar车到云发送信息格式协议
### 目标：
> 在“多车仿真协议V0.1基础上”，制定该协议(暂定为V0.2版本)  
>  制定人: Lito  
>  制定时间: 2019-03-19

### 1. 协议包格式
> 沿用v0.1版本的协议包，定义如下:

```json
data{
	"version": "0.2", // 向后兼容
	"crcCode": 0x1234, // 默认值
	"type": string , // 协议标识
	"ack": int, // 响应标识，取值参考ack字段说明
	    请求：254
        回复： 
            成功：0   
            失败：1
	"requestId": int, // 请求唯一标识
	"carId": string, // 车辆唯一标识, 17位
	"timestamp": long long,// 时间戳，单位ms。
	"data": {} // 协议内容字段，根据type类型确定字段内容。
}
```
**Note**
> 1. 以`\n\r`作为包与包之间的分隔符
> 2. 每个包以JSON方式序列化及反序列化
> 3.  carId为`17位`字符串  --新协议下,重新定义vin
> 4.  timestamp: 时间戳，表示协议发出时间。
 
vin: 以第一位区分实际车和仿真车:0表示实际车, 1表示仿真车

### 2. 头部字段取值说明
type:
>
	1. login: // 车辆登入
	2. heartbeat: // 心跳
	3. startFight: // 准备开始
	4. fight: // 开始模拟，
	5. posSyn: // 同步位置
	6. logout: // 退出
    7. SyncInfo // 所有需要及时同步的信息
    8. Current_TrjPath  // 当前跟踪的局部轨迹
    9. Obstacle  // 当前障碍物信息
    10. video  // *video和PointCloud受网络带宽影响,在本版本中不进行定义和上传*
    11. PointCloud
    12. lazy_info // 用于非实时上传的信息,如里程/电量等,发送频率低

***type中1-6的定义请参考"多车仿真协议V0.1"版本,本版本中不做实现,只为向上兼容"***
### 3. type.data说明
#### 3.1 SyncInfo
```json
data:
{
    "pose":
    [{
        "position":
        [{
            "x":float,
            "y":float,
            "z":float

        }],
        "orientation":
        [{
            "x":float,
            "y":float,
            "z":float,
            "w":float
        }]

    }],
    "state":
    [{
        "shift":"N","D","R",
        "velocity": float,
        "angle": float(rad/angle)
    }]

}
```
#### 3.2 Current_TrjPath
```json
data:
{
    "path":{PoseStamped}   // 直接引用nav_msgs::Path
}
```

#### 3.3 Obstacle
Undefined

#### 3.4 video
Undefined

#### 3.5 PointCloud
Undefined

### 3.6 lazy_info
```json
data:
{
    "mileage":float,
    "remaining_battery":float
}
```