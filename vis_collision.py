#!/usr/bin/env python3
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import os
import platform
import sys
import argparse
import math
import time
from collision_detection import RigidBody, SAT3D


isMacOS = (platform.system() == "Darwin")


class MyAppWindow():
    MOVE_SPEED = 0.1

    NORMAL_STATUS_COLOR = [0.0, 0.0, 0.9, 0.3]
    HIT_STATUS_COLOR = [0.9, 0.0, 0.0, 0.5]

    def __init__(self, width, height, obj1, obj2):
        self.window = gui.Application.instance.create_window("Collision Detection Visualizer", width, height)

        # Add 3D scene widget
        self._scene = gui.SceneWidget()
        self._scene.scene = rendering.Open3DScene(self.window.renderer)
        self.window.add_child(self._scene)

        # Create UIs
        self._create_uis()
        self.window.set_on_layout(self._on_layout)

        self._is_show_axes = True

        # Create two bodies
        self.bodyA = RigidBody(*obj1)
        self.bodyA.name = 'bodyA'
        self.bodyB = RigidBody(*obj2)
        self.bodyB.name = 'bodyB'
        self._initialize_objs_pos()

        # Initialize materials
        self._initialize_mat()

        # Add the two bodies to the scene
        self.add_rigid_body(self.bodyA)
        self.add_rigid_body(self.bodyB)

        # Setup the main camera
        self._scene.setup_camera(30, o3d.geometry.AxisAlignedBoundingBox(np.array([-8.0, -8.0, -8.0]), np.array([8.0, 8.0, 8.0])), np.array([0.0, 0.0, 0.0]))

        # Add settings for the scene
        self._scene.scene.show_axes(self._is_show_axes)
        self._scene.scene.scene.enable_sun_light(True)
        self._scene.scene.scene.set_indirect_light_intensity(20000.0)
        self._scene.scene.show_skybox(True)

        # Create a SAT
        self.sat_detector = SAT3D(self.bodyA, self.bodyB)

        # Listen to keyboard events
        self._scene.set_on_key(self._on_key_event)

    def _initialize_objs_pos(self):
        A_bb = self.bodyA.get_max_bound() - self.bodyA.get_min_bound()
        B_bb = self.bodyB.get_max_bound() - self.bodyB.get_min_bound()
        self.bodyA.translate(np.array([1 + A_bb[0] / 2, -A_bb[1] / 2, 0]))
        self.bodyB.translate(np.array([-B_bb[0] / 2 - 1, -B_bb[1] / 2, 0]))

    def _initialize_mat(self):
        self.standard_mat = o3d.visualization.rendering.Material()
        self.standard_mat.base_color = [0.9, 0.9, 0.9, 1.0]
        self.standard_mat.shader = 'defaultLit'
        self.standard_mat.base_metallic = 0.0
        self.standard_mat.base_roughness = 0.5
        self.standard_mat.base_reflectance = 0.5
        self.standard_mat.base_clearcoat = 0.5
        self.standard_mat.base_clearcoat_roughness = 0.2
        self.standard_mat.base_anisotropy = 0

        self.transparent_mat = o3d.visualization.rendering.Material()
        self.transparent_mat.base_color = self.NORMAL_STATUS_COLOR
        self.transparent_mat.shader = 'defaultUnlitTransparency'

    def _create_uis(self):
        em = self.window.theme.font_size
        separation_height = int(round(0.5 * em))

        self._settings_panel = gui.Vert(0, gui.Margins(em, em, em, em))

        panel_keycontrol = gui.Vert(0.5 * em, gui.Margins(0, 0, 0, em))
        panel_keycontrol.add_child(gui.Label("Keyboard Control:"))
        row = gui.Horiz(2 * em)
        panel_keycontrol.add_child(row)
        col1 = gui.Vert(0, gui.Margins(0.25 * em, 0, 0, 0))
        col1.add_child(gui.Label("'w' +y axis"))
        col1.add_child(gui.Label("'a' -x axis"))
        col1.add_child(gui.Label("'s' -y axis"))
        col1.add_child(gui.Label("'d' +x axis"))
        row.add_child(col1)
        col2 = gui.Vert(0, gui.Margins(0.25 * em, 0, 0, 0))
        col2.add_child(gui.Label("'q' -z axis"))
        col2.add_child(gui.Label("'e' +z axis"))
        col2.add_child(gui.Label("'ESC' quit"))
        row.add_child(col2)
        self._settings_panel.add_child(panel_keycontrol)

        panel_options = gui.Vert(0.5 * em, gui.Margins(0, 0, 0, 0))
        panel_options.add_child(gui.Label("Options:"))
        btn1 = gui.Button("Swap Kinetic Object")
        btn1.set_on_clicked(self._swap_kinect_object)
        panel_options.add_child(btn1)
        btn2 = gui.Button("Toggle Axes")
        btn2.set_on_clicked(self._toggle_axes)
        panel_options.add_child(btn2)
        btn3 = gui.Button("Capture Image")
        btn3.set_on_clicked(self._export_image)
        panel_options.add_child(btn3)
        self._settings_panel.add_child(panel_options)
        
        self.window.add_child(self._settings_panel)

    def _swap_kinect_object(self):
        self._update_rigid_body(self.bodyA, self.NORMAL_STATUS_COLOR)

        temp = self.bodyA
        self.bodyA = self.bodyB
        self.bodyB = temp

        # Keep focus on scene widget
        self.window.set_focus_widget(self._scene)

        return gui.Button.EventCallbackResult.HANDLED

    def _toggle_axes(self):
        self._is_show_axes = not self._is_show_axes
        self._scene.scene.show_axes(self._is_show_axes)

        # Keep focus on scene widget
        self.window.set_focus_widget(self._scene)

        return gui.Button.EventCallbackResult.HANDLED

    def _export_image(self):
        def on_image(image):
            path = 'collidetect_%s.png' % time.strftime('%Hh%Mm%Ss_%m%d%Y')
            quality = 9
            o3d.io.write_image(path, image, quality)
            print('Captured image saved to %s' % path)
        self._scene.scene.scene.render_to_image(on_image)

    def _on_layout(self, theme):
        r = self.window.content_rect
        self._scene.frame = r
        width = 17 * theme.font_size
        height = min(r.height, self._settings_panel.calc_preferred_size(theme).height)
        self._settings_panel.frame = gui.Rect(r.get_right() - width, r.y, width, height)

    def add_rigid_body(self, body):
        self._scene.scene.add_geometry(body.name + '_mesh', body.mesh, self.standard_mat)
        self._scene.scene.add_geometry(body.name + '_convhull', body.convhull, self.transparent_mat)

    def _on_key_event(self, e):
        if e.type == gui.KeyEvent.Type.DOWN:
            ret = gui.SceneWidget.EventCallbackResult.CONSUMED
            if e.key == 97:     # A
                self._move_rigid_body(np.array([-1, 0, 0]))
            elif e.key == 100:  # D 
                self._move_rigid_body(np.array([1, 0, 0]))
            elif e.key == 115:  # S
                self._move_rigid_body(np.array([0, -1, 0]))
            elif e.key == 119:  # W
                self._move_rigid_body(np.array([0, 1, 0]))
            elif e.key == 113:  # Q
                self._move_rigid_body(np.array([0, 0, -1]))
            elif e.key == 101:  # E
                self._move_rigid_body(np.array([0, 0, 1]))
            elif e.key == 27:   # ESC
                gui.Application.instance.quit()
            else:
                ret = gui.SceneWidget.EventCallbackResult.IGNORED

            return ret

        return gui.SceneWidget.EventCallbackResult.IGNORED

    def _move_rigid_body(self, dir):
        dir = dir.astype(np.float64)
        self.bodyA.translate(dir * self.MOVE_SPEED)

        t0 = time.time()
        is_hit = self.sat_detector.hit_test()
        print('Hit result: %s, time consumed: %s' % (is_hit, time.time() - t0))
        if is_hit:
            color = self.HIT_STATUS_COLOR
        else:
            color = self.NORMAL_STATUS_COLOR

        self._update_rigid_body(self.bodyA, color)

    def _update_rigid_body(self, body, color):
        self.transparent_mat.base_color = color

        self._scene.scene.remove_geometry(body.name + '_mesh')
        self._scene.scene.remove_geometry(body.name + '_convhull')
        self.add_rigid_body(body)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Visualize collision detection between convex hulls.')
    parser.add_argument('-m1', type=str, required=True, help='The first mesh  file.')
    parser.add_argument('-ch1', type=str, required=True, help='The convex hull file of the first mesh.')
    parser.add_argument('-m2', type=str, required=True, help='The second mesh  file.')
    parser.add_argument('-ch2', type=str, required=True, help='The convex hull file of the second mesh.')
    args = parser.parse_args()

    mesh1 = o3d.io.read_triangle_mesh(args.m1)
    convhull1 = o3d.io.read_triangle_mesh(args.ch1)

    mesh2 = o3d.io.read_triangle_mesh(args.m2)
    convhull2 = o3d.io.read_triangle_mesh(args.ch2)

    gui.Application.instance.initialize()
    w = MyAppWindow(1024, 768, (mesh1, convhull1), (mesh2, convhull2))
    gui.Application.instance.run()
