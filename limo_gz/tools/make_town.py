#!/usr/bin/python3
"""town 맵 생성기: 바닥 그림, 월드 파일, 장소 목록을 한 번에 만든다.

    python3 limo_gz/tools/make_town.py

만드는 파일 (모두 이 스크립트가 덮어쓴다. 맵을 고칠 때는 이 스크립트를 고치고 다시 실행한다):
    models/town_ground/materials/textures/town_ground.png   바닥 그림 (도로, 차선, 횡단보도, 주차 칸, 구역)
    models/town_ground/model.sdf                            그 그림을 입힌 바닥
    worlds/town.sdf                                         벽, 건물, 장애물, 신호등을 놓은 월드
    config/town_locations.yaml                              이름 붙은 장소의 좌표

맵 구성: 벽으로 둘러싼 11 x 7.5 m 공간 안에 2차로 순환 도로와 가운데를 가로지르는 도로가
있다. 우측통행(반시계 방향) 기준으로 출발점, 정지선, 횡단보도, 신호등, 주차 칸 3개,
구역 A/B/C, 건물, 나무 2그루, 장애물 상자 2개를 놓았다.

좌표: 아래 배치값은 "방 가운데 = (0, 0), x 동쪽, y 북쪽"으로 적는다. 파일로 내보낼 때는
출발점이 (0, 0)이 되도록 옮긴다. 그래서 로봇을 (0, 0)에 놓고 SLAM 을 시작하면
Gazebo 좌표, odom, map 이 모두 같은 좌표가 된다.
"""
import math
import os

import cv2
import numpy as np

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')

# ----------------------------------------------------------------------------- 배치값 [m]
ROOM_X, ROOM_Y = 11.0, 7.5           # 벽 안쪽 크기
WALL_T, WALL_H = 0.1, 0.4
LOOP_A, LOOP_B, LOOP_R = 3.9, 2.4, 1.2   # 순환 도로 중심선: 반폭 x, 반폭 y, 모서리 반지름
ROAD_W = 0.9                          # 도로 폭 (차로 2개)
LANE_W = ROAD_W / 2
LINE_W = 0.03                         # 차선 굵기
FILLET_R = 0.3                        # 교차로 모서리 둥글기
PPM = 200                             # 바닥 그림 해상도 [픽셀/m]

LANE_Y = LOOP_B + LANE_W / 2          # 아래쪽 직선 구간에서 바깥(오른쪽) 차로 중심까지 거리
START = (-2.5, -LANE_Y)               # 출발점. 내보낼 때 여기가 (0, 0)이 된다. 방향은 +x

STOP_LINE_X = -1.40                   # 정지선 (아래쪽 직선, 오른쪽 차로)
CROSSWALK_X = (-1.25, -0.95)          # 횡단보도
LIGHT_XY = (-0.6, -(LOOP_B + LANE_W + 0.3))   # 신호등: 횡단보도 지나 오른쪽 길가
PARK_X0, PARK_W, PARK_N = 1.2, 0.5, 3         # 주차 칸: 아래쪽 직선의 안쪽(북쪽)에 붙어 있다
PARK_Y = (-(LOOP_B - LANE_W), -(LOOP_B - LANE_W) + 0.8)
ZONE_SIZE = 0.9
ZONES = {'A': ((4.85, 3.1), (200, 120, 40)),      # 이름: (중심, BGR 색)
         'B': ((-4.85, 3.1), (70, 150, 60)),
         'C': ((4.85, -3.1), (40, 170, 230))}
BUILDING = ((-1.95, 0.0), (1.6, 1.6, 0.6))        # 중심, 크기
TREES = [(1.3, 0.9), (2.7, 0.3)]                  # 원기둥 (반지름 0.12, 높이 0.5)
RED_BOX = (1.8, LANE_Y)                           # 위쪽 직선의 주행 차로를 막는 상자
BLUE_BOX = (LOOP_A - LANE_W / 2, 0.5)             # 동쪽 직선의 안쪽 차로에 있는 상자
BOX_SIZE = 0.3

# 색 (BGR)
CONCRETE, GRASS, ASPHALT = (165, 165, 165), (96, 150, 96), (58, 58, 58)
WHITE, YELLOW = (245, 245, 245), (0, 200, 240)


def world(x, y):
    """배치 좌표 -> 내보내는 좌표 (출발점이 원점)."""
    return x - START[0], y - START[1]


def px(x, y):
    """배치 좌표 -> 바닥 그림 픽셀."""
    return int(round((x + ROOM_X / 2) * PPM)), int(round((ROOM_Y / 2 - y) * PPM))


def loop_centerline(step=0.01):
    """순환 도로 중심선을 반시계 방향으로 따라가는 점들."""
    a, b, r = LOOP_A, LOOP_B, LOOP_R
    pts = []

    def straight(p, q):
        n = max(2, int(math.hypot(q[0] - p[0], q[1] - p[1]) / step))
        pts.extend((p[0] + (q[0] - p[0]) * i / n, p[1] + (q[1] - p[1]) * i / n) for i in range(n))

    def arc(cx, cy, a0):
        n = max(2, int(r * math.pi / 2 / step))
        pts.extend((cx + r * math.cos(a0 + math.pi / 2 * i / n),
                    cy + r * math.sin(a0 + math.pi / 2 * i / n)) for i in range(n))

    straight((-(a - r), -b), (a - r, -b)); arc(a - r, -(b - r), -math.pi / 2)
    straight((a, -(b - r)), (a, b - r));   arc(a - r, b - r, 0.0)
    straight((a - r, b), (-(a - r), b));   arc(-(a - r), b - r, math.pi / 2)
    straight((-a, b - r), (-a, -(b - r))); arc(-(a - r), -(b - r), math.pi)
    return pts


def rect(img, x0, y0, x1, y1, color):
    (u0, v0), (u1, v1) = px(x0, y1), px(x1, y0)
    cv2.rectangle(img, (u0, v0), (u1, v1), color, -1)


def draw_ground():
    w, h = int(ROOM_X * PPM), int(ROOM_Y * PPM)
    img = np.full((h, w, 3), CONCRETE, np.uint8)
    loop = loop_centerline()
    loop_px = np.array([px(*p) for p in loop], np.int32)

    # 도로 면: 순환 도로 + 가로지르는 도로. 닫기 연산으로 교차로 모서리를 둥글게 만든다.
    road = np.zeros((h, w), np.uint8)
    cv2.polylines(road, [loop_px], True, 255, int(ROAD_W * PPM))
    cv2.line(road, px(0, -LOOP_B), px(0, LOOP_B), 255, int(ROAD_W * PPM))
    k = int(FILLET_R * PPM)
    road = cv2.morphologyEx(road, cv2.MORPH_CLOSE,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1)))

    # 순환 도로 안쪽(섬)은 잔디색
    inside = np.zeros((h, w), np.uint8)
    cv2.fillPoly(inside, [loop_px], 255)
    img[(inside > 0) & (road == 0)] = GRASS
    img[road > 0] = ASPHALT

    # 도로 가장자리 흰 선: 도로 면의 테두리를 그대로 따라 그린다 (교차로에서는 저절로 끊긴다)
    line_px = max(2, int(LINE_W * PPM))
    edge = cv2.erode(road, np.ones((line_px, line_px), np.uint8))
    contours, _ = cv2.findContours(edge, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(img, contours, -1, WHITE, line_px)

    # 중앙선: 노란 점선. 교차로 안과 정지선~횡단보도 구간에서는 그리지 않는다.
    def in_junction(x, y):
        if abs(x) < ROAD_W / 2 + FILLET_R + 0.05 and abs(abs(y) - LOOP_B) < ROAD_W / 2 + 0.05:
            return True
        return STOP_LINE_X - 0.1 < x < CROSSWALK_X[1] + 0.1 and abs(y + LOOP_B) < 0.1

    def dashes(points, step=0.01, on=0.3, period=0.6):
        n_on, n_period = int(on / step), int(period / step)
        for i in range(0, len(points) - n_on, n_period):
            seg = points[i:i + n_on + 1]
            if not any(in_junction(*p) for p in seg):
                cv2.polylines(img, [np.array([px(*p) for p in seg], np.int32)], False, YELLOW, line_px)

    dashes(loop)
    y0 = -(LOOP_B - ROAD_W / 2)
    dashes([(0.0, y0 + 0.01 * i) for i in range(int(2 * abs(y0) / 0.01))])

    # 정지선과 횡단보도 (아래쪽 직선)
    rect(img, STOP_LINE_X - 0.03, -(LOOP_B + LANE_W) + LINE_W, STOP_LINE_X + 0.03, -LOOP_B - LINE_W / 2, WHITE)
    y = -(LOOP_B + LANE_W) + 0.05
    while y + 0.07 < -(LOOP_B - LANE_W) - 0.03:
        rect(img, CROSSWALK_X[0], y, CROSSWALK_X[1], y + 0.07, WHITE)
        y += 0.14

    # 주차 칸
    x1 = PARK_X0 + PARK_W * PARK_N
    rect(img, PARK_X0, PARK_Y[0] - 2 * LINE_W, x1, PARK_Y[1], ASPHALT)
    for i in range(PARK_N + 1):
        x = PARK_X0 + PARK_W * i
        rect(img, x - LINE_W / 2, PARK_Y[0] - 2 * LINE_W, x + LINE_W / 2, PARK_Y[1], WHITE)
    rect(img, PARK_X0, PARK_Y[1] - LINE_W / 2, x1, PARK_Y[1] + LINE_W / 2, WHITE)
    for i in range(PARK_N):
        u, v = px(PARK_X0 + PARK_W * i + 0.14, PARK_Y[1] - 0.12)
        cv2.putText(img, f'P{i + 1}', (u, v), cv2.FONT_HERSHEY_SIMPLEX, 0.9, WHITE, 2, cv2.LINE_AA)

    # 구역 A, B, C
    for name, ((cx, cy), color) in ZONES.items():
        s = ZONE_SIZE / 2
        rect(img, cx - s, cy - s, cx + s, cy + s, color)
        u, v = px(cx - 0.2, cy - 0.22)
        cv2.putText(img, name, (u, v), cv2.FONT_HERSHEY_SIMPLEX, 4.0, WHITE, 8, cv2.LINE_AA)
    return img


# ----------------------------------------------------------------------------- SDF 조각
def box_model(name, x, y, sx, sy, sz, rgb, yaw=0.0):
    wx, wy = world(x, y)
    c = ' '.join(f'{v:g}' for v in rgb)
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{wx:.3f} {wy:.3f} {sz / 2:.3f} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:g} {sy:g} {sz:g}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:g} {sy:g} {sz:g}</size></box></geometry>
          <material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def cylinder_model(name, x, y, radius, height, rgb):
    wx, wy = world(x, y)
    c = ' '.join(f'{v:g}' for v in rgb)
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{wx:.3f} {wy:.3f} {height / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{radius:g}</radius><length>{height:g}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{radius:g}</radius><length>{height:g}</length></cylinder></geometry>
          <material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material>
        </visual>
      </link>
    </model>"""


def world_sdf():
    hx, hy, t = ROOM_X / 2, ROOM_Y / 2, WALL_T
    wall = (0.85, 0.85, 0.8)
    parts = [
        box_model('wall_south', 0, -(hy + t / 2), ROOM_X + 2 * t, t, WALL_H, wall),
        box_model('wall_north', 0, hy + t / 2, ROOM_X + 2 * t, t, WALL_H, wall),
        box_model('wall_west', -(hx + t / 2), 0, t, ROOM_Y, WALL_H, wall),
        box_model('wall_east', hx + t / 2, 0, t, ROOM_Y, WALL_H, wall),
        box_model('building', *BUILDING[0], *BUILDING[1], (0.8, 0.7, 0.5)),
        box_model('obstacle_red_box', *RED_BOX, BOX_SIZE, BOX_SIZE, BOX_SIZE, (0.8, 0.1, 0.1)),
        box_model('obstacle_blue_box', *BLUE_BOX, BOX_SIZE, BOX_SIZE, BOX_SIZE, (0.1, 0.2, 0.8)),
    ]
    parts += [cylinder_model(f'tree_{i + 1}', x, y, 0.12, 0.5, (0.1, 0.45, 0.15))
              for i, (x, y) in enumerate(TREES)]
    gx, gy = world(0, 0)
    lx, ly = world(*LIGHT_XY)
    return f"""<?xml version="1.0" ?>
<!-- Generated by tools/make_town.py. Edit the script and run it again instead of editing this file.
     Town test map: a walled 11 x 7.5 m space with a two-lane loop road, a cross street, a traffic
     light with stop line and crosswalk, parking slots, zones A/B/C, a building, trees and two
     obstacle boxes. The start point is the origin (0, 0), heading +x. -->
<sdf version="1.9">
  <world name="town">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <!-- Needed for the lidar and the camera -->
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <!-- Needed for the IMU -->
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground with the road painted on it (models/town_ground) -->
    <include>
      <uri>model://town_ground</uri>
      <name>ground</name>
      <pose>{gx:.3f} {gy:.3f} 0 0 0 0</pose>
    </include>
{''.join(parts)}

    <!-- Traffic light past the crosswalk, on the right side of the road, facing the oncoming car.
         z as in empty.sdf: most of the pole is below the ground. -->
    <include>
      <uri>model://traffic_light</uri>
      <name>traffic_light</name>
      <pose>{lx:.3f} {ly:.3f} -0.739 0 0 {-math.pi / 2:.4f}</pose>
    </include>
  </world>
</sdf>
"""


GROUND_SDF = f"""<?xml version="1.0" ?>
<!-- Generated by tools/make_town.py. The flat ground of the town map; the road, lane lines,
     crosswalk, parking slots and zones are one picture (materials/textures/town_ground.png). -->
<sdf version="1.9">
  <model name="town_ground">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <plane><normal>0 0 1</normal><size>{ROOM_X:g} {ROOM_Y:g}</size></plane>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <plane><normal>0 0 1</normal><size>{ROOM_X:g} {ROOM_Y:g}</size></plane>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <pbr>
            <metal>
              <albedo_map>materials/textures/town_ground.png</albedo_map>
              <roughness>1.0</roughness>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

GROUND_CONFIG = """<?xml version="1.0"?>
<model>
  <name>town_ground</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Ground of the town test map, with the road painted on it. Generated by tools/make_town.py.</description>
</model>
"""


def locations_yaml():
    def entry(name, x, y, yaw, note):
        wx, wy = world(x, y)
        return f'{name}:\n  x: {wx:.3f}\n  y: {wy:.3f}\n  yaw: {yaw:.4f}\n  description: "{note}"\n'

    half_pi = math.pi / 2
    park_y = (PARK_Y[0] + PARK_Y[1]) / 2
    out = [
        '# Generated by tools/make_town.py. Named places of the town map (worlds/town.sdf).\n'
        '#\n'
        '# x, y [m] and yaw [rad] are where the robot should be to "be at" that place.\n'
        '# The start point is (0, 0) heading +x, so these values are the same in Gazebo, in odom,\n'
        '# and in a SLAM map that was started at the start point.\n'
        '# Traffic is right-hand: the loop is driven counter-clockwise in the outer lane.\n\n',
        entry('start', *START, 0.0, '출발점. 아래쪽 직선 구간의 바깥 차로'),
        entry('stop_line', STOP_LINE_X - 0.25, -LANE_Y, 0.0, '신호등 앞 정지선 바로 앞'),
        entry('intersection_south', 0.0, -LANE_Y, 0.0, '아래쪽 교차로 한가운데 (직진 방향)'),
        entry('intersection_north', 0.0, LANE_Y, math.pi, '위쪽 교차로 한가운데 (직진 방향)'),
    ]
    for i in range(PARK_N):
        out.append(entry(f'parking_{i + 1}', PARK_X0 + PARK_W * (i + 0.5), park_y, half_pi,
                         f'주차 칸 P{i + 1} 안. 머리가 북쪽'))
    for name, ((cx, cy), _) in ZONES.items():
        color = {'A': '파란', 'B': '초록', 'C': '노란'}[name]
        out.append(entry(f'zone_{name.lower()}', cx, cy, 0.0, f'{name} 구역. {color} 바닥'))
    bx, by = BUILDING[0]
    out += [
        entry('building_front', -(LOOP_A + LANE_W / 2), by, -half_pi, '건물 옆 도로 (서쪽 직선의 바깥 차로)'),
        entry('before_red_box', RED_BOX[0] + 0.9, LANE_Y, math.pi, '빨간 상자 0.9 m 앞 (위쪽 직선의 바깥 차로)'),
        '\n# Objects (not goals): where things stand.\n',
        entry('object_traffic_light', *LIGHT_XY, -half_pi, '신호등 기둥. 렌즈는 -x 방향을 본다'),
        entry('object_building', bx, by, 0.0, '건물 중심 (1.6 x 1.6 m)'),
        entry('object_red_box', *RED_BOX, 0.0, '빨간 장애물 상자 (0.3 m)'),
        entry('object_blue_box', *BLUE_BOX, 0.0, '파란 장애물 상자 (0.3 m)'),
    ]
    return ''.join(out)


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(text)


def main():
    tex = os.path.join(PKG, 'models', 'town_ground', 'materials', 'textures', 'town_ground.png')
    os.makedirs(os.path.dirname(tex), exist_ok=True)
    cv2.imwrite(tex, draw_ground(), [cv2.IMWRITE_PNG_COMPRESSION, 9])
    write(os.path.join(PKG, 'models', 'town_ground', 'model.sdf'), GROUND_SDF)
    write(os.path.join(PKG, 'models', 'town_ground', 'model.config'), GROUND_CONFIG)
    write(os.path.join(PKG, 'worlds', 'town.sdf'), world_sdf())
    write(os.path.join(PKG, 'config', 'town_locations.yaml'), locations_yaml())
    print('wrote town_ground.png, town_ground/model.sdf, worlds/town.sdf, config/town_locations.yaml')


if __name__ == '__main__':
    main()
