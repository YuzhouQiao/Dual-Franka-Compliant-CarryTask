import os
import re
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# [新增] 引入 TimerAction 用于延时
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch_ros.parameter_descriptions import ParameterValue # 新增引用

def generate_launch_description():
    # 1. 获取包路径
    dual_arm_desc_path = get_package_share_directory('dual_arm_description')
    
    # 2. 解析 URDF
    xacro_file = os.path.join(dual_arm_desc_path, 'urdf', 'dual_arm.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    
    # [核心修复] 获取 XML 字符串
    robot_desc_xml = doc.toxml()
    
    # [新增] 将 XML 写入临时文件，避免命令行参数过长或 Topic 传输延迟导致的 Timeout
    # 使用临时文件传递给 spawn_entity 往往比 -topic 更稳定
    with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf') as tmp_file:
        tmp_file.write(robot_desc_xml)
        urdf_path = tmp_file.name

    # [优化] 使用 ParameterValue 包装，防止 YAML 解析报错
    robot_description = {'robot_description': ParameterValue(robot_desc_xml, value_type=str)}

    # 3. 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 4. 生成实体 (改为使用 -file)
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-file', urdf_path,
                                   '-entity', 'dual_panda'],
                        output='screen')

    # [新增] 加载 long_bar.urdf 并生成实体
    long_bar_path = os.path.join(dual_arm_desc_path, 'urdf', 'long_bar.urdf')
    spawn_long_bar = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-file', long_bar_path,
                                   '-entity', 'long_bar',
                                   '-x', '0.5', '-y', '0.0', '-z', '0.2'], # 坐标可根据需要调整
                        output='screen')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 5. 加载控制器
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    load_dual_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dual_arm_controller"], # 确保你在 ros2_controllers.yaml 里叫这个名字
        output="screen",
    )
    
    # [新增] 加载夹爪控制器
    load_hand_1_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_1_hand_controller"],
        output="screen",
    )
    load_hand_2_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_2_hand_controller"],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('QT_XCB_GL_INTEGRATION', 'none'),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        
        # [修复] 延迟生成 long_bar，避免与机械臂生成冲突
        TimerAction(
            period=3.0,  # 等待机械臂生成完成
            actions=[spawn_long_bar]
        ),
        
        # [核心修改] 使用 TimerAction 替代 Event Handler，强制延时 10 秒
        # 确保 Gazebo 完全加载了模型和插件后再启动控制器，这样最稳妥
        TimerAction(
            period=10.0,
            actions=[load_joint_state_broadcaster]
        ),
        TimerAction(
            period=12.0, # 稍微错开一点
            actions=[load_dual_arm_controller, load_hand_1_controller, load_hand_2_controller]
        ),
    ])