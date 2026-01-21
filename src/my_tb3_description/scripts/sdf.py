#!/usr/bin/env python3
"""SDF model generators for Gazebo."""


def wall_sdf(model_name: str, length: float, thickness: float, height: float):
    """Generate static wall box SDF."""
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>{length} {thickness} {height}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{length} {thickness} {height}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.7 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def cylinder_sdf(model_name: str, radius: float, height: float, mass: float = 5.0):
    """Generate dynamic cylinder SDF with inertia."""
    ixx = 0.5 * mass * radius * radius
    iyy = ixx
    izz = (1.0 / 12.0) * mass * (3 * radius * radius + height * height)

    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>{iyy}</iyy>
          <iyz>0</iyz>
          <izz>{izz}</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1 0.5 0.1 1</ambient>
          <diffuse>1 0.5 0.1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
