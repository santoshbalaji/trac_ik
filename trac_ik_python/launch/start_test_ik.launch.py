from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():    
    urdf_package = 'hrc_description'
    urdf_filename = 'ur.urdf.xacro'

    pkg_share_description = FindPackageShare(urdf_package)
    default_urdf_model_path = PathJoinSubstitution(
        [pkg_share_description, 'urdf', urdf_filename])

    urdf_model = LaunchConfiguration('urdf_model')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_model,
            " ",
            "name:=ur ur_type:=ur30 tool_type:=vg30",
        ]
    )

    # node with parameters
    ik_node = Node(
        package='trac_ik_python',
        executable='test_ik_node.py',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'base_link': 'base',
            'tip_link': 'tool0',
            'joint_names': [
                'shoulder_pan_joint',
                'shoulder_lift_joint',
                'elbow_joint',
                'wrist_1_joint',
                'wrist_2_joint',
                'wrist_3_joint']
        }]
    )

    ld = LaunchDescription()

    # declare the launch options
    ld.add_action(declare_urdf_model_path_cmd)

    # add any actions
    ld.add_action(ik_node)

    return ld