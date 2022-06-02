# trajectory-interpolation
**SLAM trajectory interpolation for two sensors of different rate.** 

**SLAM轨迹按时间插值**

已有激光雷达的关键帧位姿，通过时间插补来获得与之一起运动的相机（高频率）的位姿。

把旋转和平移分开插值再合起来，其中平移插值使用线性插值，旋转插值转成旋转向量再进行插值。

## 使用方法
1. 修改interpolation.py中激光到相机的外参

```
# lidar to camera extrinsic
lidar_to_cam_tx = 0.05
lidar_to_cam_ty = -0.07
lidar_to_cam_tz = -0.07
lidar_to_cam_rx = 0.0
lidar_to_cam_ry = 0.0
lidar_to_cam_rz = -0.04
```
2. 命令行运行：
```
python3 interpolation.py --input_path_traj lidar_keyframe_trajectory.txt --input_path_timestamps camera_timestamp.txt 

```
其中lidar_keyframe_trajectory.txt修改为你的已有关键帧位姿文件，camera_timestamp.txt修改为你的待插值时间戳文件。

lidar_keyframe_trajectory.txt 采用tum格式存储轨迹，camera_timestamp.txt每一行为一个待插值的时间戳。可查看示例文件。

## 附
可使用evo来可视化插值效果，参考https://github.com/MichaelGrupp/evo
```
pip install evo --upgrade --no-binary evo
```
```
evo_traj tum --ref=lidar_keyframe_trajectory.txt lidar_keyframe_trajectory_interpolation.txt lidar_keyframe_trajectory_interpolation_camera.txt -p
```
```
evo_ape tum lidar_keyframe_trajectory_interpolation.txt lidar_keyframe_trajectory_interpolation_camera.txt -va --plot
```
![image](https://user-images.githubusercontent.com/78134664/171545521-aa8338ac-b60b-44cb-9402-031a9a371897.png)

