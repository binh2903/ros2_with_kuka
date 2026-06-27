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
    # Không chạy xacro vì file nguồn là kr3r540.urdf (không có kr3r540.urdf.xacro). Chạy trực tiếp:
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
    /usr/bin/python3 ~/ros2_ws/src/move_kr3r540_b_trajectory.py
    ```

---
### Robot C: Kuka KR3 R540_URDF TU LAM

*   **Terminal 1 (Publisher):**
    ```bash
    source /opt/ros/humble/setup.bash
    cd ~/ros2_ws
    source install/setup.bash
    cd src/kuka_kr3r540/urdf/
    # Không chạy xacro vì file nguồn là kuka_kr3r540.urdf (không có kuka_kr3r540.urdf.xacro). Chạy trực tiếp:
    ros2 run robot_state_publisher robot_state_publisher kuka_kr3r540.urdf
    ```
*   **Terminal 2 (RViz):**
    ```bash
    source ~/ros2_ws/install/setup.bash
    rviz2 # Cấu hình Fixed Frame = base_link, Add RobotModel & TF
    ```
*   **Terminal 3 (Điều khiển Quỹ đạo tự động):**
    ```bash
    source ~/ros2_ws/install/setup.bash
    /usr/bin/python3 ~/ros2_ws/src/move_kr3r540_c_trajectory.py
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





////////// hien mau cac link
sudo apt install ros-<distro>-joint-state-publisher-gui
ros2 run joint_state_publisher_gui joint_state_publisher_gui

---

## 3. Hướng dẫn sử dụng Giao diện Web Điều khiển (CAD / Vẽ tay)

Ứng dụng web cho phép bạn vẽ tay trực tiếp một quỹ đạo trên mặt phẳng làm việc, hoặc kéo thả (import) file CAD dạng SVG, sau đó tự động tính toán động học ngược (IK) và gửi quỹ đạo khớp tới robot trên RViz2.

### Các bước chạy:

1. **Khởi chạy robot trên RViz2 (Robot B hoặc Robot C)**:
   Xem hướng dẫn ở phần 1 để chạy `robot_state_publisher` và `rviz2`.
   
2. **Khởi chạy Server Web & ROS 2 Node**:
   Mở một Terminal mới và chạy lệnh sau:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   /usr/bin/python3 ~/ros2_ws/src/move_kr3r540_web_gui.py
   ```
   *Server sẽ khởi động tại địa chỉ: `http://localhost:8080`*

3. **Điều khiển trên trình duyệt**:
   * Truy cập `http://localhost:8080` trên trình duyệt Chrome/Firefox.
   * Chọn dòng robot tương ứng (**Robot B** hoặc **Robot C**) ở thanh điều khiển bên trái để khớp đúng cấu hình tên khớp của robot trên RViz.
   * Nhập tọa độ offset ($X, Y, Z$) và góc xoay ($Roll, Pitch, Yaw$) để định vị vị trí và hướng của mặt phẳng làm việc 2D so với gốc robot. Giao diện 3D (Three.js) bên phải sẽ hiển thị trực quan mặt phẳng làm việc này.
   * Vẽ bằng cách nhấn giữ chuột trên bảng vẽ 2D ở giữa, hoặc kéo thả file SVG (CAD) của bạn vào vùng tải lên.
   * Nhấn **"Sim path"** để xem mô phỏng robot chạy thử quỹ đạo trực quan trên màn hình 3D.
   * Nhấn **"Run ROS2"** để ra lệnh cho robot thật/mô phỏng trên RViz di chuyển theo quỹ đạo bạn đã vẽ.
   * Trong trường hợp khẩn cấp, nhấn nút **"EMERGENCY STOP"** màu đỏ để dừng chuyển động ngay lập tức.

---

## 4. Quản lý các phiên bản Server Web & ROS 2 Node

Dự án cung cấp hai phiên bản backend để tính toán động học:
1. **Phiên bản Giải tích (Analytical)**: `move_kr3r540_web_gui.py` (Tính toán dựa trên công thức hình học tự viết).
2. **Phiên bản KDL (MoveIt Core)**: `move_kr3r540_web_gui_kdl.py` (Sử dụng thư viện `PyKDL` chính thức của MoveIt).

> [!IMPORTANT]
> Luôn luôn sử dụng Python hệ thống `/usr/bin/python3` để chạy các file python này. Các python cục bộ (như `/home/binh/.local/bin/python3.10`) có thể thiếu các thư viện liên kết quan trọng của ROS 2 (như `sensor_msgs` và `numpy`).

### A. Cách đóng hoàn toàn phiên bản cũ đang chạy nền (Orphan Process)
Nếu bạn thay đổi phiên bản backend hoặc chỉnh sửa code, hãy tắt tất cả các tiến trình chạy ngầm cũ để tránh xung đột cổng `8080` và topic `/joint_states`:

1. Tìm và tắt các tiến trình python của web gui đang chạy ngầm:
   ```bash
   pkill -f move_kr3r540_web_gui
   ```
2. (Tùy chọn) Kiểm tra xem có tiến trình nào còn sót lại:
   ```bash
   pgrep -af python3 | grep move_kr3r540
   ```
   Nếu hiển thị danh sách dạng `<PID> python3 ...`, tắt thủ công bằng lệnh:
   ```bash
   kill -9 <PID>
   ```

### B. So sánh giữa phiên bản Giải tích (Analytical) và phiên bản KDL (MoveIt)

| Đặc tính | Phiên bản Giải Tích (`move_kr3r540_web_gui.py`) | Phiên bản KDL (`move_kr3r540_web_gui_kdl.py`) |
| :--- | :--- | :--- |
| **Cơ chế tính toán** | Dùng các công thức ma trận thuần túy nhân tay bằng `numpy`. | Dùng cấu trúc `PyKDL.Chain` và các Solver số trị của KDL/MoveIt. |
| **Độ chính xác** | Rất cao, khớp 100% với hình học toán học nếu công thức đúng. | Rất cao, sử dụng các tham số trục xoay vật lý thực tế trong URDF. |
| **Xử lý giới hạn khớp** | Ép cứng góc bằng hàm `np.clip`. | Áp dụng giới hạn động trong solver thông qua bộ giải `ChainIkSolverPos_NR_JL`. |
| **Khả năng mở rộng** | Khó chỉnh sửa nếu thay đổi cấu trúc robot (phải tính lại toán). | Dễ dàng thay đổi kích thước khớp/thanh nối chỉ bằng việc cập nhật `PyKDL.Segment`. |

### C. Cách chạy phiên bản KDL
1. Tắt phiên bản cũ (xem mục A).
2. Chạy terminal mới:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   /usr/bin/python3 ~/ros2_ws/src/move_kr3r540_web_gui_kdl.py
   ```