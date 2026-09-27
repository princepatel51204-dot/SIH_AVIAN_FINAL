#!/usr/bin/env python3
"""Minimal Gazebo world to test the detector on ONE decal (feasibility test; not the mission world).

A large concrete-grey wall (normal +x), one decal quad on it carrying a rendered defect texture, a
sun, and N static cameras at given distances in front of the decal (640x480, HFOV --hfov, default 1.3962634,
clip 0.05-300: the mission's gimbal camera). Cameras publish /decal_cam_<d> (d in cm).

Usage: make_test_world.py <texture.png> <decal_width_m> <out.sdf> [--dists 3.5,4.5,6.0] [--wall-rgb 0.55,0.55,0.55]
                          [--sun-dir -1,0.2,-0.4] [--decal thin_box|plane] [--hfov RAD]
"""
import argparse, os

ap = argparse.ArgumentParser()
ap.add_argument('texture'); ap.add_argument('width', type=float); ap.add_argument('out')
ap.add_argument('--dists', default='3.5,4.5,6.0')
ap.add_argument('--wall-rgb', default='0.55,0.55,0.55')
ap.add_argument('--sun-dir', default='-1,0.2,-0.4')
ap.add_argument('--decal', default='thin_box', choices=['thin_box', 'plane'])
ap.add_argument('--ambient', default='0.4,0.4,0.4')
ap.add_argument('--hfov', type=float, default=1.3962634, help='camera horizontal FOV (rad); 1.3962634 = the gimbal camera')
a = ap.parse_args()
tex = os.path.abspath(a.texture)
wr = a.wall_rgb.replace(',', ' ')
w = a.width
dec_geom = (f'<box><size>0.01 {w} {w}</size></box>' if a.decal == 'thin_box'
            else f'<plane><normal>1 0 0</normal><size>{w} {w}</size></plane>')
cams = ''
for d in [float(x) for x in a.dists.split(',')]:
    cams += f'''
    <model name="decal_cam_{int(round(d*100))}"><static>true</static><pose>{d + 0.1} 0 0 0 0 3.14159265</pose>
      <link name="l"><sensor name="cam" type="camera"><always_on>1</always_on><update_rate>5</update_rate><topic>decal_cam_{int(round(d*100))}</topic>
        <camera><horizontal_fov>{a.hfov}</horizontal_fov><image><width>640</width><height>480</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>300</far></clip></camera></sensor></link></model>'''
sdf = f'''<?xml version="1.0"?>
<sdf version="1.9"><world name="decal_test">
  <physics name="p" type="ignored"><max_step_size>0.01</max_step_size><real_time_factor>1</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <scene><ambient>{a.ambient.replace(',', ' ')} 1</ambient><background>0.8 0.85 0.9 1</background><shadows>false</shadows></scene>
  <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><specular>0.1 0.1 0.1 1</specular>
    <direction>{a.sun_dir.replace(',', ' ')}</direction></light>
  <model name="wall"><static>true</static><pose>-0.1 0 0 0 0 0</pose>
    <link name="l"><visual name="v"><geometry><box><size>0.2 14 10</size></box></geometry>
      <material><ambient>{wr} 1</ambient><diffuse>{wr} 1</diffuse><specular>0.05 0.05 0.05 1</specular></material></visual></link></model>
  <model name="decal"><static>true</static><pose>0.006 0 0 0 0 0</pose>
    <link name="l"><visual name="v"><geometry>{dec_geom}</geometry>
      <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
        <pbr><metal><albedo_map>{tex}</albedo_map><roughness>1.0</roughness><metalness>0.0</metalness></metal></pbr></material></visual></link></model>
  {cams}
</world></sdf>
'''
open(a.out, 'w').write(sdf)
print('wrote', a.out)
