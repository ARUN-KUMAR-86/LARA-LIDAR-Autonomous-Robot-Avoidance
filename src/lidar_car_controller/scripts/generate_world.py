#!/usr/bin/env python3
"""
generate_world.py
─────────────────
Generates a Gazebo .world file with your custom obstacles.

Usage:
  python3 generate_world.py              → writes my_custom_world.world
  python3 generate_world.py --preview    → prints world to terminal only
"""

import math
import os
import argparse


# ════════════════════════════════════════════════════════════════
#  SDF BUILDER FUNCTIONS
#  Each function returns a string of valid SDF XML
# ════════════════════════════════════════════════════════════════

def box(name, x, y,
        sx=0.6, sy=0.6, sz=1.0,
        z=None,
        yaw=0.0,
        color=(0.8, 0.2, 0.2)):
    """
    Spawn a box obstacle.
    x, y     : position in world (metres)
    sx sy sz : size in metres (length, width, height)
    z        : Z centre (auto = sz/2 so it sits on ground)
    yaw      : rotation in radians (0=along X, pi/2=along Y)
    color    : (R, G, B) each 0.0–1.0
    """
    z   = z if z is not None else sz / 2.0
    r, g, b = color
    return f"""
  <model name="{name}">
    <static>true</static>
    <pose>{x} {y} {z} 0 0 {yaw:.6f}</pose>
    <link name="link">
      <collision name="col">
        <geometry>
          <box><size>{sx} {sy} {sz}</size></box>
        </geometry>
      </collision>
      <visual name="vis">
        <geometry>
          <box><size>{sx} {sy} {sz}</size></box>
        </geometry>
        <material>
          <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
          <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>"""


def cylinder(name, x, y,
             radius=0.3, height=1.0,
             z=None,
             color=(0.2, 0.4, 0.9)):
    """
    Spawn a cylinder obstacle (good for pillars, poles).
    radius : metres
    height : metres
    """
    z   = z if z is not None else height / 2.0
    r, g, b = color
    return f"""
  <model name="{name}">
    <static>true</static>
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="link">
      <collision name="col">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="vis">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
          <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>"""


def sphere(name, x, y,
           radius=0.4,
           z=None,
           color=(0.2, 0.9, 0.3)):
    """
    Spawn a sphere obstacle.
    """
    z   = z if z is not None else radius
    r, g, b = color
    return f"""
  <model name="{name}">
    <static>true</static>
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="link">
      <collision name="col">
        <geometry>
          <sphere><radius>{radius}</radius></sphere>
        </geometry>
      </collision>
      <visual name="vis">
        <geometry>
          <sphere><radius>{radius}</radius></sphere>
        </geometry>
        <material>
          <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
          <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
    </link>
  </model>"""


def wall(name, x, y,
         length=3.0, thickness=0.2, height=1.5,
         yaw=0.0,
         color=(0.55, 0.40, 0.25)):
    """
    Spawn a flat wall.
    length    : how long the wall is (metres)
    thickness : how thick (metres)
    height    : how tall (metres)
    yaw       : 0=horizontal along X, pi/2=vertical along Y
    """
    return box(name, x, y,
               sx=length, sy=thickness, sz=height,
               yaw=yaw, color=color)


def boundary(arena=6.0, color=(0.65, 0.65, 0.65)):
    """
    Four outer walls forming a square arena.
    arena : half-size (arena=6 → 12×12 metre space)
    """
    s  = arena
    t  = 0.2
    h  = 1.5
    L  = s * 2 + t
    walls = ''
    walls += wall('bound_N',  0,  s, length=L, height=h, yaw=0,            color=color)
    walls += wall('bound_S',  0, -s, length=L, height=h, yaw=0,            color=color)
    walls += wall('bound_E',  s,  0, length=L, height=h, yaw=math.pi / 2,  color=color)
    walls += wall('bound_W', -s,  0, length=L, height=h, yaw=math.pi / 2,  color=color)
    return walls


def ring_of_cylinders(name_prefix, cx, cy,
                      ring_radius=3.0, count=8,
                      cyl_radius=0.2, height=1.0,
                      color=(0.9, 0.3, 0.6)):
    """
    Place N cylinders evenly around a circle.
    cx, cy      : centre of the ring
    ring_radius : distance from centre to each cylinder
    count       : number of cylinders
    """
    models = ''
    for i in range(count):
        angle = i * (2 * math.pi / count)
        x     = cx + ring_radius * math.cos(angle)
        y     = cy + ring_radius * math.sin(angle)
        models += cylinder(f'{name_prefix}_{i}', x, y,
                           radius=cyl_radius, height=height,
                           color=color)
    return models


def slalom_gates(name_prefix, start_x, y_range=4.0,
                 count=5, gap=0.9,
                 wall_length=1.5, color=(0.2, 0.7, 0.7)):
    """
    Create a slalom course: alternating left/right walls.
    start_x  : X position of first gate
    y_range  : how far walls extend from centre
    count    : number of gates
    gap      : X spacing between gates
    """
    models = ''
    for i in range(count):
        x      = start_x + i * gap
        side   = 1 if i % 2 == 0 else -1   # alternate sides
        y      = side * (y_range / 2)
        models += wall(f'{name_prefix}_{i}', x, y,
                       length=wall_length,
                       yaw=math.pi / 2,     # perpendicular to path
                       color=color)
    return models


def maze_corridor(name_prefix, x, y,
                  corridor_length=4.0, corridor_width=1.2,
                  wall_height=1.5, color=(0.5, 0.3, 0.1)):
    """
    Create a U-shaped corridor (3 walls).
    Opens toward +X direction by default.
    """
    t   = 0.2
    cw  = corridor_width
    cl  = corridor_length
    models  = ''
    # Left wall
    models += wall(f'{name_prefix}_left',
                   x, y + cw / 2,
                   length=cl, thickness=t, height=wall_height,
                   yaw=0, color=color)
    # Right wall
    models += wall(f'{name_prefix}_right',
                   x, y - cw / 2,
                   length=cl, thickness=t, height=wall_height,
                   yaw=0, color=color)
    # Back wall
    models += wall(f'{name_prefix}_back',
                   x - cl / 2, y,
                   length=cw + t * 2, thickness=t, height=wall_height,
                   yaw=math.pi / 2, color=color)
    return models


# ════════════════════════════════════════════════════════════════
#  WORLD FILE ASSEMBLER
# ════════════════════════════════════════════════════════════════

def assemble_world(models: list, world_name='custom_world') -> str:
    header = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="{world_name}">

    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>

    <light name="ambient_fill" type="directional">
      <cast_shadows>false</cast_shadows>
      <diffuse>0.5 0.5 0.5 1</diffuse>
      <specular>0.0 0.0 0.0 1</specular>
      <direction>0 0 -1</direction>
    </light>

    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
"""
    footer = "\n  </world>\n</sdf>\n"
    return header + ''.join(models) + footer


# ════════════════════════════════════════════════════════════════
#  ✏️  EDIT THIS SECTION TO BUILD YOUR WORLD
# ════════════════════════════════════════════════════════════════

def build_my_world() -> list:
    """
    Add your obstacles here.
    Return a list of SDF model strings.
    All coordinates are in metres.
    Robot spawns at (0, 0) facing +X direction.
    """
    models = []

    # ── 1. Outer boundary (12×12 m arena) ────────────────────
    models.append(boundary(arena=6.0))

    # ── 2. Single boxes ──────────────────────────────────────
    #       box(name, x, y, sx, sy, sz, yaw, color)
    models.append(box('red_box',    3,  0,
                       sx=0.6, sy=0.6, sz=1.0,
                       color=(0.9, 0.1, 0.1)))

    models.append(box('green_box', -2,  3,
                       sx=0.5, sy=1.2, sz=1.0,
                       yaw=math.pi / 6,       # 30° angle
                       color=(0.1, 0.8, 0.1)))

    models.append(box('blue_box',   2, -3,
                       sx=1.0, sy=0.3, sz=1.0,
                       color=(0.1, 0.1, 0.9)))

    # ── 3. Cylinders ─────────────────────────────────────────
    #       cylinder(name, x, y, radius, height, color)
    models.append(cylinder('yellow_pillar',  1,  2,
                            radius=0.3, height=1.0,
                            color=(0.9, 0.8, 0.1)))

    models.append(cylinder('purple_pillar', -3, -2,
                            radius=0.25, height=1.0,
                            color=(0.6, 0.1, 0.8)))

    # ── 4. Spheres ────────────────────────────────────────────
    #       sphere(name, x, y, radius, color)
    models.append(sphere('orange_ball', -1, -4,
                          radius=0.4,
                          color=(0.9, 0.5, 0.1)))

    # ── 5. Walls ─────────────────────────────────────────────
    #       wall(name, x, y, length, thickness, height, yaw, color)
    models.append(wall('wall_horizontal', 0,  1,
                        length=2.5, yaw=0))

    models.append(wall('wall_vertical',   -1, -2,
                        length=2.0, yaw=math.pi / 2))

    models.append(wall('wall_diagonal',   4,  3,
                        length=2.0, yaw=math.pi / 4))  # 45°

    # ── 6. Ring of cylinders ──────────────────────────────────
    #       ring_of_cylinders(prefix, cx, cy, ring_radius, count)
    models.append(ring_of_cylinders('ring', 0, -3,
                                    ring_radius=1.5, count=6,
                                    cyl_radius=0.15,
                                    color=(0.9, 0.3, 0.3)))

    # ── 7. Slalom gates ───────────────────────────────────────
    #       slalom_gates(prefix, start_x, y_range, count, gap)
    models.append(slalom_gates('slalom', start_x=-5, y_range=3.0,
                                count=4, gap=1.5,
                                color=(0.2, 0.9, 0.9)))

    # ── 8. U-shaped corridor ──────────────────────────────────
    #       maze_corridor(prefix, x, y, length, width)
    models.append(maze_corridor('corridor', 2, 4,
                                 corridor_length=3.0,
                                 corridor_width=1.0,
                                 color=(0.7, 0.5, 0.2)))

    return models


# ════════════════════════════════════════════════════════════════
#  MAIN
# ════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Generate a Gazebo world file with custom obstacles')
    parser.add_argument('--preview', action='store_true',
                        help='Print to terminal instead of writing file')
    parser.add_argument('--output', type=str,
                        default=None,
                        help='Output file path (default: auto)')
    args = parser.parse_args()

    models     = build_my_world()
    world_sdf  = assemble_world(models, world_name='my_custom_world')

    if args.preview:
        print(world_sdf)

    else:
        # Auto output path: next to this script's worlds/ folder
        script_dir  = os.path.dirname(os.path.abspath(__file__))
        worlds_dir  = os.path.join(script_dir, '..', 'worlds')
        worlds_dir  = os.path.normpath(worlds_dir)
        output_path = args.output or os.path.join(
            worlds_dir, 'my_custom_world.world')

        os.makedirs(worlds_dir, exist_ok=True)

        with open(output_path, 'w') as f:
            f.write(world_sdf)

        print(f'✔  World written to: {output_path}')
        print(f'   Models included : {len(models)} blocks')
        print()
        print('Launch with:')
        print('  ros2 launch lidar_car_controller simulation.launch.py \\')
        print('    world:=my_custom_world.world')