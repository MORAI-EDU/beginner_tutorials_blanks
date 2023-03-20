# 1. 버전 내역

이후의 버전은 Tag에서 확인

<br>

### [0308]

- **[수정]** 토픽 연결상태 실시간 진단 기능 추가

  - `pure_pursuit`

  - `pid_purepursuit`

  - `advanced_purepursuit`

  - `collision_avoid`
  - `gps`

  - `imu`

  - `gpsimu_parser`

  - `image_parser`

  - `lane_binarize`

  - `lane_roi`

  - `lane_bev`

  - `lane_fitting`

  - `lane_fillower`

<br>

### [0307]

- **[삭제]** 외부 스크립트 import 방식 제거
  - `util.py` 
- **[수정]** util.py에 대한 의존성 제거 및 필요 코드 추가
  - `lane_fitting`
  - `lane_follower`

<br>

<br>

# 2. 상세 내용

<br>

#### 1. s_drive

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

#### 2. pure_pursuit 관련 내용

<br>

**`pure_pursuit.py` line 30 ~ 31**

```python
self.vehicle_length = None
self.lfd = None
```

purepursuit 알고리즘을 이용하여 제어할 조향각을 계산할때 필요한 vehicle_length (m) 와 차량의 전방주시거리 (m) 입니다.

<br>

**`pure_pursuit.py` line 70**

```python
self.ctrl_cmd_msg.steering = None
```

purepursuit 알고리즘을 이용하여 제어할 조향각을 계산합니다.

<br>

**`pure_pursuit_pid.py` line 57 ~ 58**

```python
self.vehicle_length = None
self.lfd = None
```

purepursuit 알고리즘을 이용하여 제어할 조향각을 계산할때 필요한 vehicle_length (m) 와 차량의 전방주시거리 (m) 입니다.

<br>

**`pure_pursuit_pid.py` line 98**

```python
self.ctrl_cmd_msg.steering = None
```

<br>

**`pure_pursuit_pid_velocity_planning.py` line 57 ~ 58**

```python
self.vehicle_length = None
self.lfd = None
```

purepursuit 알고리즘을 이용하여 제어할 조향각을 계산할때 필요한 vehicle_length (m) 와 차량의 전방주시거리 (m) 입니다.

<br>

**`pure_pursuit_pid_velocity_planning.py` line 179**

```python
steering = None
```

purepursuit 알고리즘을 이용하여 제어할 조향각을 계산합니다.

<br>

**`lane_follower.py` line 28 ~ 29**

```python
self.vehicle_length = None
self.lfd = None
```

purepursuit 알고리즘을 이용하여 제어할 조향각을 계산할때 필요한 vehicle_length (m) 와 차량의 전방주시거리 (m) 입니다.

**`lane_follower.py` line 98**

```python
steering_deg = None
```

purepursuit 알고리즘을 이용하여 제어할 조향각을 계산합니다.<br>

<br>

<br>

#### 3. 차선인지 관련 내용

<br>

**`lane_binarize.py` line 39 ~ 40**

```python
lower_wlane = np.array([0,0,0])
upper_wlane = np.array([30,60,255])
```

흰색 차선을 제외한 나머지 부분은 제외하기 위한 설정 입니다.

따라서 흰색의 lower boundary에 해당하는 범위를 찾아 넣으면 됩니다. 

<br>

**`lane_roi.py` line 17 ~ 24**

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

관심영역 외 제거하기 위한 crop 포인트 입니다.

<br>

**`lane_fitting.py` line 33 ~ 39**

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

**`lane_roi.py` line 17 ~ 24**

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

crop_pts 는 관심영역(ROI) 외엔 마스킹(잘라내기 위한)처리를 위한 구간 설정 입니다.

<br>
