# TurtleBot3 連続自律移動プログラム (ROS 2 Jazzy / Navigation 2)

ROS 2 Jazzy と Navigation 2 を用いて、TurtleBot3 が Gazebo の標準ワールド内で障害物にぶつからずに動き続けるためのPython制御プログラムです。ラップトップでの動作を考慮し、軽量な処理を目指しています。

## 🤖 システム要件

* **OS**: Ubuntu 24.04 LTS
* **ROS 2**: Jazzy Jalisco
* **Gazebo**: Gazebo Harmonic (gz-sim 7) - ROS 2 Jazzy で推奨
* **TurtleBot3**: シミュレーションモデル (Burger, Waffle, Waffle Pi)
* **Navigation 2**: ROS 2 Jazzy に対応するバージョン

## 🛠️ 必要なパッケージのインストール

1.  **ROS 2 Jazzy (Desktop) のインストール:**
    ROS 2 の公式ドキュメントに従ってインストールしてください。
    ```bash
    sudo apt update && sudo apt upgrade -y
    sudo apt install -y ros-jazzy-desktop ros-dev-tools
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

2.  **Gazebo (gz-sim) と ROS 2-Gazebo連携パッケージ:**
    ```bash
    sudo apt install -y gz-harmonic # Gazebo Harmonic (gz-sim 7)
    sudo apt install -y ros-jazzy-ros-gzgarden # gz-sim と ROS 2 の連携
    ```

3.  **Colcon とその他ツール:**
    ```bash
    sudo apt install -y python3-colcon-common-extensions python3-vcstool git
    ```

4.  **TurtleBot3 シミュレーションと Navigation 2 パッケージ:**
    ```bash
    sudo apt install -y ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-simulations
    sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup
    sudo apt install -y python3-tf-transformations # Pythonでの座標変換用
    ```
    *注意:* `ros-jazzy-turtlebot3-simulations` が `gz-sim` (Gazebo Harmonic) を使用することを確認してください。

## 📦 プログラムのセットアップとビルド

1.  **ROS 2 ワークスペースの作成:**
    ```bash
    mkdir -p ~/turtlebot3_nav2_ws/src
    cd ~/turtlebot3_nav2_ws/src
    ```

2.  **本プログラムのソースコードを配置:**
    `continuous_navigator` という名前のパッケージを作成します。
    ```bash
    ros2 pkg create --build-type ament_python continuous_navigator --dependencies rclpy nav2_simple_commander geometry_msgs tf_transformations
    ```
    作成された `continuous_navigator` ディレクトリ内に、上記の `continuous_navigator.py` というファイル名でPythonスクリプトを保存します (例: `~/turtlebot3_nav2_ws/src/continuous_navigator/continuous_navigator/continuous_navigator.py`)。
    `setup.py` の `entry_points` を編集してスクリプトを実行可能にします。

    **`~/turtlebot3_nav2_ws/src/continuous_navigator/setup.py` の `entry_points` 部分を以下のように修正:**
    ```python
    entry_points={
        'console_scripts': [
            'navigator_node = continuous_navigator.continuous_navigator:main',
        ],
    },
    ```
    **`~/turtlebot3_nav2_ws/src/continuous_navigator/package.xml` に依存関係が正しく記述されているか確認:**
    `<exec_depend>rclpy</exec_depend>`
    `<exec_depend>nav2_simple_commander</exec_depend>`
    `<exec_depend>geometry_msgs</exec_depend>`
    `<exec_depend>python3-tf-transformations</exec_depend>` (または `tf_transformations` として記載し、`rosdep` で解決)

3.  **ビルド:**
    ```bash
    cd ~/turtlebot3_nav2_ws
    # rosdep install -i --from-path src -y --skip-keys "gz-harmonic gz-plugin python-tf-transformations" # 不足している依存関係をインストール
    colcon build --symlink-install
    ```

4.  **環境設定の読み込み:**
    ビルドしたワークスペースのセットアップファイルを実行します。
    ```bash
    echo "source ~/turtlebot3_nav2_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

## 🚀 動作確認方法

**重要:** TurtleBot3のモデル (burger, waffle, waffle_pi) に応じて、環境変数 `TURTLEBOT3_MODEL` を設定してください。
```bash
export TURTLEBOT3_MODEL=burger # 例: burgerモデルの場合
# または waffle, waffle_pi