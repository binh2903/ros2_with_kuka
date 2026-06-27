# Hướng dẫn và Giải thích di chuyển Gói `kuka_kr3r540` từ ROS 1 sang ROS 2

Tài liệu này cung cấp hướng dẫn và giải thích chi tiết các bước cần thiết để sửa gói mô tả robot `kuka_kr3r540` (được xuất từ SolidWorks sang URDF cho ROS 1) nhằm xây dựng và hoạt động thành công trên môi trường **ROS 2** sử dụng công cụ `colcon build`.

---

## 1. Dọn dẹp cấu hình `package.xml`

### Thay đổi thực hiện:
- Loại bỏ toàn bộ phần cấu hình ROS 1 (catkin) đã bị chú thích (commented out).
- Đảm bảo định dạng XML định dạng `format="3"`.
- Định nghĩa kiểu build cho gói là `ament_cmake`.
- Khai báo các gói phụ thuộc ROS 2 chính xác.

### Giải thích chi tiết:
Trong ROS 1, định dạng package thường là `format="2"` và sử dụng `<buildtool_depend>catkin</buildtool_depend>`.
Đối với ROS 2:
- Hệ thống build tool chuyển thành `ament_cmake`. Vì vậy cần chỉ định tag `<build_type>` trong khối `<export>` là `ament_cmake`:
  ```xml
  <export>
    <build_type>ament_cmake</build_type>
  </export>
  ```
- Các gói phụ thuộc runtime để hiển thị Robot (như `joint_state_publisher`, `robot_state_publisher`, `rviz2`, `xacro`) được chỉ định bằng `<exec_depend>`.

---

## 2. Di chuyển `CMakeLists.txt` từ Catkin sang Ament CMake

### File `CMakeLists.txt` mới:
```cmake
cmake_minimum_required(VERSION 3.8)
project(kuka_kr3r540)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### Giải thích chi tiết:
1. **`cmake_minimum_required(VERSION 3.8)`**: ROS 2 yêu cầu phiên bản CMake tối thiểu cao hơn so với ROS 1 (thường từ 3.5 hoặc 3.8 trở lên).
2. **`find_package(ament_cmake REQUIRED)`**: Thay thế gói `catkin` của ROS 1 bằng `ament_cmake` của ROS 2.
3. **Loại bỏ `catkin_package()`**: Macros của catkin không còn tồn tại trong ROS 2.
4. **Hàm `install(DIRECTORY ...)`**: 
   - Trong ROS 1, các tài nguyên được cài đặt vào thư mục đích thông qua biến `${CATKIN_PACKAGE_SHARE_DESTINATION}`.
   - Trong ROS 2, vị trí tiêu chuẩn để lưu trữ tài nguyên tĩnh (URDF, meshes, launch files, config) là thư mục chia sẻ chung của hệ thống: `share/${PROJECT_NAME}` (ví dụ: `share/kuka_kr3r540`).
5. **`ament_package()`**: Hàm bắt buộc gọi cuối cùng trong gói `ament_cmake` để đăng ký gói với hệ thống ROS 2 ament.

---

## 3. Di chuyển Launch Files sang ROS 2 Python Launch

ROS 1 sử dụng các tệp tin cấu hình Launch dưới dạng XML (`.launch`). Mặc dù ROS 2 hỗ trợ XML launch, cách chính thống và linh hoạt hơn là viết bằng Python (`.launch.py`).

### A. Tệp `display.launch.py` (Hiển thị robot trên RViz2)
Thay thế cho `display.launch` cũ, tệp tin này nạp mô tả robot từ file URDF và chạy `robot_state_publisher` cùng với giao diện chỉnh khớp `joint_state_publisher_gui` và `rviz2`.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('kuka_kr3r540')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'kuka_kr3r540.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
```

### B. Tệp `gazebo.launch.py` (Mô phỏng robot trong Gazebo)
Thay thế cho `gazebo.launch` cũ. Tệp này sử dụng plugin `gazebo_ros` của ROS 2 để khởi chạy Gazebo và spawn thực thể robot từ file URDF thông qua service `/spawn_entity`.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('kuka_kr3r540')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'kuka_kr3r540.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    gazebo_pkg_dir = get_package_share_directory('gazebo_ros')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_dir, 'launch', 'gazebo.launch.py')
        )
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'kuka_kr3r540', '-file', urdf_file],
        output='screen'
    )
    
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        static_tf
    ])
```

---

## 4. Kiểm tra và Cài đặt

Để biên dịch gói mô hình này sau khi sửa đổi, chạy các lệnh sau tại root workspace của bạn (`ros2_ws`):

```bash
# 1. Biên dịch gói kuka_kr3r540
colcon build --packages-select kuka_kr3r540

# 2. Source môi trường
source install/setup.bash

# 3. Khởi chạy hiển thị mô hình robot trên RViz2
ros2 launch kuka_kr3r540 display.launch.py
```
