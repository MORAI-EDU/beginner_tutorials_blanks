# :pencil: beginner_tutorials

MORAI SIM Drive을 이용한 자율주행 학습 예제파일 입니다.

특정 파일마다, 공백처리된 파트가 있습니다.

학습한 내용을 바탕으로 공백처리된 부분을 채워 예제를 실행하면 됩니다.

아래 내용은 해당 blank 된 부분을 소개합니다.

<br>

<br>

## 1. s_drive.py 관련 내용

```python
from morai_msgs.msg import <<>>

class s_drive():
    def __init__(self):
        rospy.init_node('s_drive', anonymous=True)
        cmd_pub = rospy.Publisher(<<>>, <<>>, queue_size=1)
        rate = rospy.Rate(30)
        cmd = <<>>
        cmd.longlCmdType = <<>>
        cmd.velocity = <<>>
        steering_cmd = [ <<>>, <<>>]
        cmd_cnts = <<>>
```

학습내용을 바탕으로 <<>> 를 채우는 예제 입니다.

<br>

<br>

## 2. pure_pursuit 관련 내용

<br>

#### **`pure_pursuit.py` line 31 ~ 32**

```python
print("you need to change values at line 32~33 ,  self.vegicle_length , lfd")
self.vehicle_length=0
self.lfd=0
```

self.vehicle_length : 차량의 축거 (m)

self.lfd : 차량의 전방주시거리 (m)

<br>

#### **`pure_pursuit.py` line 69 ~ 70**

```python
print("you need to change the value at line 70")
self.ctrl_cmd_msg.steering=0
```

purepursuit 알고리즘을 이용하여 제어할 조향각을 계산합니다.

<br>

#### **`pid_purepursuit.py` line 59 ~ 61**

```python
print("you need to change values at line 60 ~ 61 :  vegicle_length , lfd")
self.vehicle_length = 0
self.lfd = 0
```

purepursuit 알고리즘을 이용하여 제어할 조향각을 계산할때 필요한 차량의 축거와 전방주시거리 입니다.

<br>

#### `advanced_purepursuit.py` line 176 ~ 177

```python
print("you need to change pure_pursuit_calcu_steering")
steering = 0
```

purepursuit 알고리즘을 이용하여 제어할 조향각을 계산합니다.

<br>

#### `lane_follower.py` line 27 ~ 28

```python
print("you need to set lfd, current value is 0  >>>>>>  ctrller = purePursuit(lfd=0)")
ctrller = purePursuit(lfd=0)
```

purepursuit 알고리즘을 이용하여 제어할 조향각을 계산할때 필요한 차량의 전방주시거리 (m) 입니다.

<br>

#### `util.py` line 481 ~ 482

```python
print("you need to change value at line 482 : purepursuit_steering")
steering_deg= 0
```

purepursuit 알고리즘을 이용하여 제어할 조향각을 계산합니다.

<br>

<br>

## 3. 차선인지 관련 내용

<br>

#### **`lane_binarize.py` line 27 ~ 29**

```python
print("you need to find the right value : line 28 ~ 29")
lower_wlane = np.array([0,0,0])
upper_wlane = np.array([0,0,0])
```

흰색 차선을 제외한 나머지 부분은 제외하기 위한 설정 입니다.

따라서 흰색에 해당하는 범위를 찾아 넣으면 됩니다. 

<br>

#### **`lane_fitting.py` line 23 ~ 29**

```python
print("you need to find the right value : line 23 ~ 29")
self.lower_wlane = np.array([0,0,0])
self.upper_wlane = np.array([0,0,0])

self.lower_ylane = np.array([0,0,0])
self.upper_ylane = np.array([0,0,0])

self.crop_pts = np.array([[[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]])
```

흰색과 노란색 선을 제외한 나머지 부분은 제외하기 위한 설정 입니다.

crop_pts 는 관심영역(ROI) 외엔 마스킹(잘라내기 위한)처리를 위한 구간 설정 입니다.

<br>
