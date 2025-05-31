# TurtleBot3 連続自律移動プログラム

ROS 2 Jazzy と Navigation 2 を用いて、TurtleBot3 が標準ワールド内で障害物にぶつからずに動き続けるためのPython制御プログラムです。

## 前提システム要件

* **OS**: Ubuntu 24.04 LTS
* **ROS 2**: Jazzy (LTS)
* **Gazebo**: Harmonic (LTS)

## 必要なパッケージのインストール

1.  **ROS 2 Jazzy (Desktop) のインストール:**
    ROS 2 の公式ドキュメントに従ってインストールしてください。
    https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

2.  **Gazeboと ROS 2-Gazebo連携パッケージのインストール:**
    Gazeboの公式ドキュメントに従ってインストールしてください。
    https://gazebosim.org/docs/harmonic/ros_installation/

3.  **Colcon とその他ツールのインストール:**
    ```bash
    sudo apt install -y python3-colcon-common-extensions python3-vcstool git
    ```

4.  **TurtleBot3 シミュレーションと Navigation 2 パッケージのインストール:**
    ```bash
    sudo apt install -y ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-simulations
    sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup
    sudo apt install ros-jazzy-tf-transformations
    ```

## ビルド手順

1.  **本レポジトリのクローン:**
    ```bash
    git clone https://github.com/tsfk9981/turtlebot3_nav2_ws ~/turtlebot3_nav2_ws

2.  **ビルドの実行:**
    ```bash
    cd turtlebot3_nav2_ws
    colcon build --symlink-install
    ```

## 動作確認方法

### ターミナル１
1.  **環境変数の設定：**
    ```bash
    source /opt/ros/jazzy/setup.bash
    source ~/turtlebot3_nav2_ws/install/setup.bash
    export TURTLEBOT3_MODEL=waffle
    ```

2.  **GazeboとTurtlebot3モデルの起動：**
    ```bash
    ros2 launch nav2_bringup tb3_simulation_launch.py use_sim_time:=True headless:=False slam:=True
    ```

3.  **GazeboとRvizが正常に起動していることを確認：**
    
    正常に起動できている場合、Rviz上でNav2 Goalを設定するとTurtleBot3が障害物を避けながら目的地まで移動します。

### ターミナル2
1.  **環境変数の設定**
    ```bash
    source /opt/ros/jazzy/setup.bash
    source ~/turtlebot3_nav2_ws/install/setup.bash
    export TURTLEBOT3_MODEL=waffle
    ```

2.  **コントローラーの起動：**
    ```bash
    ros2 run continuous_navigator navigator_node
    ```
    このコントローラーではNavigation 2上のNav2 Goalを定期的に更新し、TurtleBot3が障害物にぶつからず動き続ける動作を実現します。
