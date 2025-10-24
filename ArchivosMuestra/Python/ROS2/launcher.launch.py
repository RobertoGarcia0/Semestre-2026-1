from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
  # Ruta del directorio actual
  current_dir = os.path.dirname(os.path.realpath(__file__))

  # Rutas a los scripts
  publisher_path = os.path.join(current_dir, 'publisher_params.py')
  subscriber_path = os.path.join(current_dir, 'subscriber_params.py')

  publisher_node = ExecuteProcess(
    cmd=['python3', publisher_path, '--ros-args',
      '-p', 'topic_param:=/test_topic'],
    output='screen'
  )

  subscriber_node = ExecuteProcess(
    cmd=['python3', subscriber_path, '--ros-args',
      '-p', 'topic_param:=/test_topic'],
    output='screen'
  )

  turtle_sim = Node(
    package='turtlesim',
    executable="turtlesim_node",
    name='turtlesim_node',
    output='screen'
  )

  return LaunchDescription([
    publisher_node, 
    subscriber_node,
    turtle_sim
    ])


"""# Definir nodos dentro de un paquete
  publisher_node = Node(
    package='pkg_name',
    executable=publisher_path,
    name='generic_publisher',
    output='screen',
    parameters=[
      {'topic_param': '/test_topic'}
    ]
  )

  subscriber_node = Node(
    package='pkg_name',
    executable=subscriber_path,
    name='generic_subscriber',
    output='screen',
    parameters=[
      {'topic_param': '/test_topic'}
    ]
  )"""