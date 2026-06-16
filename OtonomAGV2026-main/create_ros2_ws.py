#!/usr/bin/env python3
import os
import shutil

# Target workspace path
WS_ROOT = os.path.expanduser("~/sirius_amr_ws")
SRC_DIR = os.path.join(WS_ROOT, "src")

print(f"Creating ROS 2 Workspace at: {WS_ROOT}")

# Create directories
dirs = [
    os.path.join(SRC_DIR, "agv_bringup", "launch"),
    os.path.join(SRC_DIR, "agv_bringup", "config"),
    os.path.join(SRC_DIR, "agv_description", "urdf"),
    os.path.join(SRC_DIR, "agv_description", "rviz"),
    os.path.join(SRC_DIR, "agv_sensors", "agv_sensors"),
    os.path.join(SRC_DIR, "agv_navigation", "config"),
    os.path.join(SRC_DIR, "agv_navigation", "maps"),
]

for d in dirs:
    os.makedirs(d, exist_ok=True)
    print(f"-> Created: {d}")

# ----------------------------------------------------
# 1. agv_bringup files
# ----------------------------------------------------
bringup_xml = """<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>agv_bringup</name>
  <version>0.0.0</version>
  <description>Bringup launch and parameter files for SIRIUS AMR</description>
  <maintainer email="elif@example.com">elif</maintainer>
  <license>Apache License 2.0</license>

  <exec_depend>agv_description</exec_depend>
  <exec_depend>agv_sensors</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""

bringup_cmake = """cmake_minimum_required(VERSION 3.8)
project(agv_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
"""

bringup_launch = """from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    description_dir = get_package_share_directory('agv_description')
    urdf_file = os.path.join(description_dir, 'urdf', 'sirius_amr.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Robot State Publisher (URDF ve TF için)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # Arduino Bridge Node (JSON Serial)
        Node(
            package='agv_sensors',
            executable='encoder_odom_publisher',
            name='encoder_odom_publisher',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud': 115200,
                'wheel_base': 0.40,
                'pwm_multiplier': 185.0,
                'pulses_per_rev': 3000.0
            }]
        ),
        
        # Static Transform base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_broadcaster',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link']
        )
    ])
"""

with open(os.path.join(SRC_DIR, "agv_bringup", "package.xml"), "w") as f:
    f.write(bringup_xml)
with open(os.path.join(SRC_DIR, "agv_bringup", "CMakeLists.txt"), "w") as f:
    f.write(bringup_cmake)
with open(os.path.join(SRC_DIR, "agv_bringup", "launch", "bringup.launch.py"), "w") as f:
    f.write(bringup_launch)

# ----------------------------------------------------
# 2. agv_description files
# ----------------------------------------------------
desc_xml = """<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>agv_description</name>
  <version>0.0.0</version>
  <description>Robot description (URDF model) for SIRIUS AMR</description>
  <maintainer email="elif@example.com">elif</maintainer>
  <license>Apache License 2.0</license>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""

desc_cmake = """cmake_minimum_required(VERSION 3.8)
project(agv_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
"""

desc_urdf = """<?xml version="1.0"?>
<robot name="sirius_amr">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0.1 0.1 0.8 0.8"/>
      </material>
    </visual>
  </link>

  <!-- IMU Link -->
  <link name="imu_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.02"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.1 0.1 1.0"/>
      </material>
    </visual>
  </link>

  <joint name="base_to_imu" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

</robot>
"""

with open(os.path.join(SRC_DIR, "agv_description", "package.xml"), "w") as f:
    f.write(desc_xml)
with open(os.path.join(SRC_DIR, "agv_description", "CMakeLists.txt"), "w") as f:
    f.write(desc_cmake)
with open(os.path.join(SRC_DIR, "agv_description", "urdf", "sirius_amr.urdf"), "w") as f:
    f.write(desc_urdf)

# ----------------------------------------------------
# 3. agv_sensors files (Python package)
# ----------------------------------------------------
sensors_xml = """<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>agv_sensors</name>
  <version>0.0.0</version>
  <description>Sensor nodes (Arduino Serial JSON bridge) for SIRIUS AMR</description>
  <maintainer email="elif@example.com">elif</maintainer>
  <license>Apache License 2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
"""

sensors_setup = """from setuptools import find_packages, setup

package_name = 'agv_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elif',
    maintainer_email='elif@example.com',
    description='Arduino JSON serial publisher and driver',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_odom_publisher = agv_sensors.encoder_odom_publisher:main',
        ],
    },
)
"""

sensors_setup_cfg = """[develop]
script_dir=$base/lib/agv_sensors
[install]
install_scripts=$base/lib/agv_sensors
"""

# Create a resource folder for ament python index
os.makedirs(os.path.join(SRC_DIR, "agv_sensors", "resource"), exist_ok=True)
with open(os.path.join(SRC_DIR, "agv_sensors", "resource", "agv_sensors"), "w") as f:
    f.write("")

# Copy publisher script into python module directory
local_pub = "/home/elif/Downloads/OtonomAGV2026-main/encoder_odom_publisher.py"
dest_pub = os.path.join(SRC_DIR, "agv_sensors", "agv_sensors", "encoder_odom_publisher.py")

if os.path.exists(local_pub):
    shutil.copy(local_pub, dest_pub)
    print(f"-> Copied node to: {dest_pub}")
else:
    print(f"Warning: local publisher '{local_pub}' not found to copy!")

with open(os.path.join(SRC_DIR, "agv_sensors", "package.xml"), "w") as f:
    f.write(sensors_xml)
with open(os.path.join(SRC_DIR, "agv_sensors", "setup.py"), "w") as f:
    f.write(sensors_setup)
with open(os.path.join(SRC_DIR, "agv_sensors", "setup.cfg"), "w") as f:
    f.write(sensors_setup_cfg)
with open(os.path.join(SRC_DIR, "agv_sensors", "agv_sensors", "__init__.py"), "w") as f:
    f.write("")

# ----------------------------------------------------
# 4. agv_navigation files
# ----------------------------------------------------
nav_xml = """<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>agv_navigation</name>
  <version>0.0.0</version>
  <description>Navigation and mapping configuration for SIRIUS AMR</description>
  <maintainer email="elif@example.com">elif</maintainer>
  <license>Apache License 2.0</license>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""

nav_cmake = """cmake_minimum_required(VERSION 3.8)
project(agv_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
"""

with open(os.path.join(SRC_DIR, "agv_navigation", "package.xml"), "w") as f:
    f.write(nav_xml)
with open(os.path.join(SRC_DIR, "agv_navigation", "CMakeLists.txt"), "w") as f:
    f.write(nav_cmake)

print("\nAll package files and configurations created successfully!")
print("Run the following commands to build your workspace:")
print(f"  cd {WS_ROOT}")
print("  colcon build --symlink-install")
