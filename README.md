### **vip_gui2**

ROS based 차량 및 드론 어시스트 GUI 플랫폼 

**기존 사용중인 workspace 사용해도 무방**  
```
cd catkin_workspace/src
git clone https://github.com/kka-na/vip_gui2.git
```

**소스 파일**  
*icon folder*: GUI에 사용하는 아이콘 이미지 모음  

*map folder* : 송도 캠퍼스 맵 csv 파일 모음  

*d2v.py* : GUI및 노드 subscribe 하여 display 하기 위한 main window 클래스. controlros 클래스에서 넘어오는 signal 함수와 연결된 slot함수가 동작하면서 각각 텍스트 혹은 이미지를 gui를 통해 보여주게 된다. 

*d2v.ui* : GUI 레이아웃 파일  

*dialog_widget.ui* : rostopic csv 파일 생성을 위한 다이얼로그 위젯 레이아웃 파일. 

*default.rviz* : 송도 맵 rviz 파일  

*rviz_array.py* : rviz 프레임에 GPS, IMU 등의 데이터를 받아서 marker로써 표시하기 위한 subscribe, publish 코드. 

*settingros.py* : 처음에 GUI 초기화시에 ROS 토픽들 초기화 하기 위한 클래스. 해당 코드를 통해서 QT의 sub widget을 통해 새로운 토픽이름들을 저장할 .csv 파일을 생성할 수 있다. 같은 topic을 계속 사용한다면, 해당 클래스는 실행되지 않고 넘어간다. 

*01.csv* : 20-겨울학기 다학년 연구 프로젝트 개발에 사용중이던 토픽 파일. 

*controlros.py* : ros topic 이름들이 저장된 csv파일을 불러오고, 해당 토픽들을 subscribe하며 callback 함수를 실행시킨다. callback 함수는 데이터가 들어올때마다 qt의 signal-slot 함수 동작에 따라 emit으로 main window 클래스에 신호와 데이터를 보내게 된다. 

**(+)Update 중** imu 동작 하는거 따라서 gl 움직여야하는데 잘 움직이다가 프로그램이 죽어버리는 현상이 있어서 원인 파악 중 .. 
계속 죽으면 actionstart_triggered 함수에서 gl부분 주석처리하고 동작하면 잘 됨. 

*myobjloader.py* : .obj파일과 .mtl 파일을 읽기 위한 코드 

*imugl.py* : imu ros topic의 emit 신호를 받아서 imu의 움직임에 따라 3D 오브젝트의 움직임을 주는 QT Qopengl 위젯 클래스. 해당 클래스를 mainwindow에 frame에 추가하여 사용한다. 

*Drone.obj, Drone.mtl* : 드론의 자세에 따른 움직임을 보여주기 위한 3D obj 파일.

*Car.obj, Car.mtl* : 차량의 자세에 따른 움직임을 보여주기 위한 3D obj 파일. 


**실행방법**  
```
terminal #1 ) roscore
```
```
terminal #2 ) python rviz_array.py   
```
rviz 2D NAV GOAL사용해서 목적지 찍으면 정보 확인 가능   
```
terminal #3) python d2v.py  
```
GUI가 열림.   

Monitoring 모드엔 총 4가지 선택 
1. Ready : 레디를 누르면 rostopic list를 생성할 것인지 아니면 import 할 것인지 물어본다. 새로운 토픽을 추가할 것이라면 create를 선택하고, 아니라면 Cancel을 눌러서 다음 모드를 실행한다. 
2. Record (선택 사항) : Record 모드를 선택하면 bag 파일로 현재 주행 및 비행 상황을 녹화할 수 있다. ok를 누르면 bag 파일을 저장할 디렉토리를 선택하는 창이 열리고, 취소를 누르면 그 다음으로 넘어가면 된다. 
3. Start : Start를 누르면 topic list file을 import하라는 메시지가 뜨고, 위에서 생성한 파일이나 혹은 사용중이던 csv파일을 import 하여 ros 토픽들을 세팅할 수 있다. 
4. Stop : 모니터링을 그만두거나, 녹화중이던 bag파일을 정상적으로 저장하고 싶으면 stop 모드를 선택하면된다. 

Replaying 모드엔 총 3가지 선택
1. Ready : 저장된 bag 파일을 선택하고, 토픽들을 세팅하게된다. 
2. Save as Dataset : bag파일로 녹화된 데이터들 ( 현재는 image 데이터만 )을 raw 데이터 형식으로 timestamp 순서대로 ts text 파일과 함께 저장되게 된다. KITTI 데이터셋과 비슷한 데이터셋 구조를 가지며 저장된다. 
3. Start : monitoring 모드와 마찬가지로 topic list csv file을 불러오며, 불러고 난 후 저장된 bag파일을 관제할 수 있다. 또한 아래 play bar를 통해서 사용자가 원하는 시점의 데이터만 확인할 수 있다. 
