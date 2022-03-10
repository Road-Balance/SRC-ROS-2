우선은 불편하더라도 urdf 매번 생성해서 작업하자.

나중에 확실해지면 그때 또 바꾸면 됨

```
ros2 launch src_gazebo src_gazebo.launch.py
```

# odom 비교하기

/odom_rf2o/twist/twist/linear/x => 기본적으로 출렁인다.
cmd_vel하고 비교하려고 했는데 이건 뭐...

![image](https://user-images.githubusercontent.com/12381733/157583451-44f5861b-41ba-4504-a21e-3832a55183e8.png)

그런데, 가만히 있어도 tf가 마구 흔들리네

![image](https://user-images.githubusercontent.com/12381733/157586216-7c9e4a84-9b1a-4081-8c9b-43fd355b85f4.png)

pose를 비교해보자.

정확한 비교를 위해 plot의 크기와 높이를 맞춤

<p>
    <p align="center">
        <img src="https://user-images.githubusercontent.com/12381733/157586931-b348203c-6250-4267-9f8d-54e07bef3e5b.png" height="200">
        <img src="https://user-images.githubusercontent.com/12381733/157586971-979fbb12-d48b-49cc-8ff3-b2881c8fcb49.png" height="250">
    </p>
</p>

```
ros2 launch src_gazebo compare_odom.launch.py
```

이정도 차이난다.
계속 내비두면 laser odom은 낮아짐
![image](https://user-images.githubusercontent.com/12381733/157587300-ed5b3171-74fe-475d-b9d3-3310c795de90.png)

? 하다보니 또 맞을 때도 있네?

=> 일단 실제 로봇으로 가서 얼마나 잘되는지 실제 해보자.