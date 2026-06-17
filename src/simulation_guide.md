# Hướng dẫn mô phỏng & cấu hình Robot Kuka (KR4 R600 & KR3 R540) trên ROS 2

Tài liệu này hướng dẫn cách chạy mô phỏng cho cả hai dòng robot **Kuka KR4 R600** và **Kuka KR3 R540**, đồng thời giải thích chi tiết ý nghĩa và cấu trúc của các file cấu hình đi kèm dự án.

---

## 1. Hướng dẫn chạy mô phỏng chi tiết

### Robot A: Kuka KR4 R600

*   **Terminal 1 (Publisher):**
    ```bash
    source /opt/ros/humble/setup.bash
    cd ~/ros2_ws
    source install/setup.bash
    cd src/kuka_robot_descriptions/kuka_agilus_support/urdf
    ros2 run xacro xacro kr4_r600.urdf.xacro > kr4_r600.urdf
    ros2 run robot_state_publisher robot_state_publisher kr4_r600.urdf
    ```
*   **Terminal 2 (RViz):**
    ```bash
    source ~/ros2_ws/install/setup.bash
    rviz2 # Cấu hình Fixed Frame = base_link, Add RobotModel & TF
    ```
*   **Terminal 3 (Điều khiển Quỹ đạo tự động):**
    ```bash
    source ~/ros2_ws/install/setup.bash
    /usr/bin/python3 ~/ros2_ws/src/kuka_robot_descriptions/kuka_agilus_support/move_robot_trajectory.py
    ```

---

### Robot B: Kuka KR3 R540

*   **Terminal 1 (Publisher):**
    ```bash
    source /opt/ros/humble/setup.bash
    cd ~/ros2_ws
    source install/setup.bash
    cd src/kr3_viewer/urdf/
    ros2 run xacro xacro kr3r540.xacro > kr3r540.urdf
    ros2 run robot_state_publisher robot_state_publisher kr3r540.urdf
    ```
*   **Terminal 2 (RViz):**
    ```bash
    source ~/ros2_ws/install/setup.bash
    rviz2 # Cấu hình Fixed Frame = base_link, Add RobotModel & TF
    ```
*   **Terminal 3 (Điều khiển Quỹ đạo tự động):**
    ```bash
    source ~/ros2_ws/install/setup.bash
    /usr/bin/python3 ~/ros2_ws/src/kr3_viewer/move_kr3r540_trajectory.py
    ```

---

## 2. Giải thích cấu trúc các file cấu hình dự án

Dưới đây là chi tiết các file cấu hình và vai trò của chúng trong việc định nghĩa và xuất bản mô hình 3D robot:

### A. File build hệ thống: `CMakeLists.txt`
*   **Đường dẫn trong dự án:**
    *   KR4 R600: [kuka_agilus_support/CMakeLists.txt](file:///home/binh/ros2_ws/src/kuka_robot_descriptions/kuka_agilus_support/CMakeLists.txt)
    *   KR3 R540: [kr3_viewer/CMakeLists.txt](file:///home/binh/ros2_ws/src/kr3_viewer/CMakeLists.txt)
*   **Vai trò:** Khai báo quy trình biên dịch gói (package) và chỉ định cài đặt thư mục tài nguyên (`urdf`, `meshes` chứa file 3D) vào không gian cài đặt chung (`install/`).
*   **Các mục cấu hình chính:**
    *   `project(kr3_viewer)` / `project(kuka_agilus_support)`: Đặt tên cho package.
    *   `install(DIRECTORY urdf meshes DESTINATION share/${PROJECT_NAME})`: **Cực kỳ quan trọng**. Lệnh này sao chép thư mục chứa file thiết kế 3D (`meshes`) và cấu trúc robot (`urdf`) vào thư mục `install/share/<tên_gói>`. Nhờ có dòng này, RViz2 mới có thể tìm thấy file mesh 3D qua giao thức `package://<tên_gói>/meshes/...`.
    *   `ament_package()`: Đăng ký package này vào hệ thống ROS 2.

### B. File định nghĩa Package: `package.xml`
*   **Đường dẫn trong dự án:**
    *   KR4 R600: [kuka_agilus_support/package.xml](file:///home/binh/ros2_ws/src/kuka_robot_descriptions/kuka_agilus_support/package.xml)
    *   KR3 R540: [kr3_viewer/package.xml](file:///home/binh/ros2_ws/src/kr3_viewer/package.xml)
*   **Vai trò:** Khai báo các thông tin siêu dữ liệu (Metadata) và các gói phụ thuộc (Dependencies) cần thiết của robot.
*   **Các mục cấu hình chính:**
    *   `<name>`: Định nghĩa tên gói ROS 2 (ví dụ: `<name>kr3_viewer</name>`).
    *   `<buildtool_depend>ament_cmake</buildtool_depend>`: Chỉ định công cụ build được dùng là `ament_cmake`.
    *   `<export> <build_type>ament_cmake</build_type> </export>`: Khai báo định dạng build để hệ thống `colcon` hiểu cách biên dịch.

### C. File Mô tả Robot: `*.urdf` và `*.xacro`
*   **Đường dẫn trong dự án:**
    *   KR4 R600: [kr4_r600.urdf](file:///home/binh/ros2_ws/src/kuka_robot_descriptions/kuka_agilus_support/urdf/kr4_r600.urdf)
    *   KR3 R540: [kr3r540.urdf](file:///home/binh/ros2_ws/src/kr3_viewer/urdf/kr3r540.urdf)
*   **Vai trò:** Chứa toàn bộ cấu hình vật lý, hình học và động học của robot để phục vụ cho việc tính toán cơ học và hiển thị 3D.
*   **Các mục cấu hình chính:**
    *   **Thẻ `<link>`:** Định nghĩa cấu trúc vật lý của từng đốt (phần thân) robot.
        *   `<visual>` -> `<geometry>` -> `<mesh filename="package://..."/>`: Chỉ đường dẫn tới file 3D `.dae` hoặc `.stl` dùng để hiển thị mô hình trực quan trong RViz.
        *   `<collision>`: Chỉ đường dẫn tới file 3D dạng đơn giản/stl dùng để tính toán va chạm vật lý.
        *   `<inertial>`: Định nghĩa khối lượng (`<mass value="..."/>`) và mô-men quán tính (`<inertia ixx="..." .../>`) của đốt robot đó (lưu ý: Robot KR3 R540 là mô hình động học trực quan đơn giản nên không chứa thẻ này, còn KR4 R600 có đầy đủ để mô phỏng động lực học).
    *   **Thẻ `<joint>`:** Định nghĩa liên kết (khớp nối) giữa các đốt robot.
        *   `type="revolute"` / `type="fixed"`: Loại khớp xoay (có giới hạn) hoặc khớp cố định.
        *   `<parent link="..."/>` & `<child link="..."/>`: Chỉ định mối quan hệ cha-con của chuỗi động học.
        *   `<limit lower="..." upper="..." velocity="..." effort="..."/>`: Định nghĩa giới hạn góc xoay tối đa/tối thiểu (radian), vận tốc xoay tối đa (`velocity` - rad/s) và momen lực tối đa (`effort` - N.m).
