# :pencil: beginner_tutorials

MORAI SIM : Drive을 이용한 예제파일 입니다.

특정 파일마다, 공백처리된 파트가 있습니다.

아래 내용은 해당 blank 된 부분을 소개합니다.

*코드 업데이트 내역은 answer code repository를 참조*

<br>

<br>

## 1. s_drive.py 관련 내용

```python
from morai_msgs.msg import <<here!>>

class s_drive():
    def __init__(self):
        rospy.init_node('s_drive', anonymous=True)
        cmd_pub = rospy.Publisher(<<here!>>, <<here!>>, queue_size=1)
        rate = rospy.Rate(30)
        cmd = <<here!>>
        cmd.longlCmdType = <<here!>>
        cmd.velocity = <<here!>>
        steering_cmd = [ <<here!>>, <<here!>>]
        cmd_cnts = <<here!>>
```

학습내용을 바탕으로 <<here!>> 를 채우는 예제 입니다.

<br>

<br>

## 2. pure_pursuit 관련 내용

<br>

#### **`pure_pursuit.py` line 30 ~ 31**

```python
self.vehicle_length = None
self.lfd = None
```

self.vehicle_length : 차량의 축거 (m)

self.lfd : 차량의 전방주시거리 (m)

<br>

#### **`pure_pursuit.py` line 70**

```python
self.ctrl_cmd_msg.steering = None
```

purepursuit 알고리즘을 이용하여 제어할 조향각을 계산합니다.

<br>

#### **`pid_purepursuit.py` line 57 ~ 58**

```python
self.vehicle_length = None
self.lfd = None
```

purepursuit 알고리즘을 이용하여 제어할 조향각을 계산할때 필요한 차량의 축거와 전방주시거리 입니다.

<br>

#### **`pid_purepursuit.py` line 98**

```python
self.ctrl_cmd_msg.steering = None
```

purepursuit 알고리즘을 이용하여 제어할 조향각을 계산합니다.
<br>

#### `advanced_purepursuit.py` line 176

```python
steering = None
```

purepursuit 알고리즘을 이용하여 제어할 조향각을 계산합니다.

<br>

#### `lane_follower.py` line 28 ~ 29

```python
self.vehicle_length = None
self.lfd = None
```

purepursuit 알고리즘을 이용하여 제어할 조향각을 계산할때 필요한 vehicle_length (m) 와 차량의 전방주시거리 (m) 입니다.

<br>

#### `lane_follower.py` line 98

```python
steering_deg = None
```

purepursuit 알고리즘을 이용하여 제어할 조향각을 계산합니다.
<br>
<br>
<br>

## 3. 차선인지 관련 내용

<br>

#### **`lane_binarize.py` line 39 ~ 40**

```python
lower_wlane = np.array([0,0,0])
upper_wlane = np.array([0,0,0])
```

흰색 차선을 제외한 나머지 부분은 제외하기 위한 설정 입니다.

따라서 흰색에 해당하는 범위를 찾아 넣으면 됩니다. 

<br>

#### **`lane_roi.py` line 17 ~ 24**

```python
self.crop_pts = np.array(
    [[
        [0,0],
        [0,0],
        [0,0],
        [0,0]
    ]]
)
```

흰색 차선을 제외한 나머지 부분은 제외하기 위한 설정 입니다.

따라서 흰색에 해당하는 범위를 찾아 넣으면 됩니다. 

<br>

#### **`lane_fitting.py` line 33 ~ 39**

```python
self.lower_wlane = np.array([0,0,0])
self.upper_wlane = np.array([0,0,0])

self.lower_ylane = np.array([0,0,0])
self.upper_ylane = np.array([0,0,0])

self.crop_pts = np.array([[[0,0],[0,0],[0,0],[0,0]]])
```

흰색과 노란색 선을 제외한 나머지 부분은 제외하기 위한 설정 입니다.

crop_pts 는 관심영역(ROI) 외엔 마스킹(잘라내기 위한)처리를 위한 구간 설정 입니다.

<br>
