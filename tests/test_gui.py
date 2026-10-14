import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import unittest
import time
import tempfile
from pathlib import Path
import numpy as np
from PySide6.QtCore import QPoint, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QStyle, QStyleOptionSlider
from uvms.gui import KinematicsWindow

ROOT = Path(__file__).resolve().parents[1]


class GuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        cls.window = KinematicsWindow()
        cls.window.show()
        QTest.qWait(80)

    @classmethod
    def tearDownClass(cls):
        output = ROOT / "outputs"
        output.mkdir(exist_ok=True)
        cls.window.stop_motion()
        cls.window.arm.forward(cls.window.arm.home_q,unit="rad")
        cls.window.goal_q = cls.window.arm.home_q.copy()
        cls.window.sync_state()
        cls.window.align_target()
        cls.window.reject_contacts.setChecked(True)
        cls.window.grab().save(str(output / "kinematics_gui.png"))
        cls.window.close()
        cls.app.processEvents()

    def setUp(self):
        self.window.control_tabs.setCurrentIndex(0)
        self.window.stop_motion()
        self.window.reset_block_action.trigger()
        self.window.arm.forward(self.window.arm.home_q,unit="rad")
        self.window.goal_q = self.window.arm.home_q.copy()
        self.window.sync_state()
        self.window.align_target()
        self.window.frame_select.setCurrentIndex(0)
        self.window.reject_contacts.setChecked(False)

    def complete_motion(self):
        count = 0
        while self.window.motion is not None:
            self.window.motion_clock = time.monotonic()-0.051
            self.window.advance_motion()
            count += 1
            self.assertLess(count,1000)

    def test_slider_keyboard_updates_fk_and_coordinates(self):
        slider = self.window.sliders[0]
        before = self.window.arm.position().copy()
        old = slider.value()
        slider.setFocus()
        QTest.keyClick(slider, Qt.Key.Key_Right)
        self.assertEqual(slider.value(), old+10)
        np.testing.assert_allclose(self.window.arm.position(),before)
        self.complete_motion()
        self.assertNotEqual(self.window.arm.position().tolist(), before.tolist())
        self.assertAlmostEqual(self.window.arm.state()["q_deg"][0], slider.value()/100)
        for v in self.window.arm.position():
            self.assertIn(f"{v: .6f}", self.window.coordinates.text())

    def test_slider_mouse_drag_updates_joint(self):
        slider = self.window.sliders[1]
        option = QStyleOptionSlider()
        slider.initStyleOption(option)
        handle = slider.style().subControlRect(QStyle.ComplexControl.CC_Slider, option,
                                               QStyle.SubControl.SC_SliderHandle, slider)
        before = self.window.arm.state()["q_deg"][1]
        start = handle.center()
        QTest.mousePress(slider,Qt.MouseButton.LeftButton,pos=start)
        QTest.mouseMove(slider,start+QPoint(8,0))
        QTest.mouseRelease(slider,Qt.MouseButton.LeftButton,pos=start+QPoint(8,0))
        self.complete_motion()
        self.assertNotEqual(before,self.window.arm.state()["q_deg"][1])

    def test_position_inputs_button_and_failure_retains_pose(self):
        target = [0.11,-0.03,0.60]
        for spin,value in zip(self.window.xyz_inputs,target):
            spin.setValue(value)
        QTest.mouseClick(self.window.solve_button, Qt.MouseButton.LeftButton)
        self.assertTrue(self.window.viewport.target_ok)
        self.assertIsNotNone(self.window.motion)
        self.complete_motion()
        self.assertLess(np.linalg.norm(self.window.arm.position()-target),1e-5)
        saved = self.window.arm.data.qpos.copy()
        for spin in self.window.xyz_inputs:
            spin.setValue(10)
        QTest.mouseClick(self.window.solve_button, Qt.MouseButton.LeftButton)
        self.assertFalse(self.window.viewport.target_ok)
        np.testing.assert_array_equal(self.window.arm.data.qpos,saved)
        self.assertIn("保留当前姿态",self.window.status.text())

    def test_target_point_drag_uses_ik(self):
        viewport = self.window.viewport
        start = viewport.widget_point(viewport.target).toPoint()
        before = self.window.arm.position().copy()
        QTest.mousePress(viewport, Qt.MouseButton.LeftButton, pos=start)
        self.assertEqual(viewport.drag["mode"],"plane")
        QTest.mouseMove(viewport,start+QPoint(4, -3))
        QTest.qWait(70)
        QTest.mouseRelease(viewport,Qt.MouseButton.LeftButton,pos=start+QPoint(4,-3))
        self.complete_motion()
        self.assertTrue(viewport.target_ok)
        self.assertGreater(np.linalg.norm(self.window.arm.position()-before),0.001)
        self.assertLess(np.linalg.norm(self.window.arm.position()-viewport.target),1e-5)

    def test_world_axis_handles(self):
        for i in range(3):
            self.setUp()
            viewport = self.window.viewport
            before = viewport.target.copy()
            center = viewport.widget_point(before)
            end = viewport.widget_point(before+np.eye(3)[i]*0.065)
            start = (center+(end-center)*0.82).toPoint()
            delta = end-center
            direction = np.array([delta.x(), delta.y()])
            direction /= np.linalg.norm(direction)
            finish = start+QPoint(int(round(direction[0]*4)),int(round(direction[1]*4)))
            QTest.mousePress(viewport,Qt.MouseButton.LeftButton,pos=start)
            self.assertIsNotNone(viewport.drag)
            self.assertEqual(viewport.drag["mode"],i)
            QTest.mouseMove(viewport,finish)
            QTest.qWait(60)
            QTest.mouseRelease(viewport,Qt.MouseButton.LeftButton,pos=finish)
            self.complete_motion()
            movement = viewport.target-before
            self.assertGreater(abs(movement[i]),0.001)
            np.testing.assert_allclose(np.delete(movement,i),0,atol=1e-12)
            self.assertTrue(viewport.target_ok)

    def test_frame_change_preserves_target_world_location(self):
        target = self.window.viewport.target.copy()
        self.window.frame_select.setCurrentIndex(1)
        origin, rotation = self.window.arm._frame("base")
        values = np.array([spin.value() for spin in self.window.xyz_inputs])
        np.testing.assert_allclose(origin+rotation@values,target,atol=1e-6)
        QTest.mouseClick(self.window.solve_button,Qt.MouseButton.LeftButton)
        self.complete_motion()
        self.assertLess(np.linalg.norm(self.window.arm.position()-target),1e-5)

    def test_scene_reset_restores_free_block(self):
        arm = self.window.arm
        joint = arm.model.joint("block_free").id
        qadr,vadr = arm.model.jnt_qposadr[joint],arm.model.jnt_dofadr[joint]
        arm.data.qpos[qadr:qadr+3] = [.4,-.2,.1]
        arm.data.qvel[vadr:vadr+6] = 1
        self.window.reset_block_action.trigger()
        np.testing.assert_array_equal(arm.data.qpos[qadr:qadr+7],arm.model.key_qpos[0,qadr:qadr+7])
        np.testing.assert_array_equal(arm.data.qvel[vadr:vadr+6],np.zeros(6))

    def test_intermediate_motion_and_stop_action(self):
        start = self.window.arm.data.qpos.copy()
        self.window.set_joint(0,50)
        self.assertIsNotNone(self.window.motion)
        np.testing.assert_array_equal(self.window.arm.data.qpos,start)
        self.window.motion_clock = time.monotonic()-0.051
        self.window.advance_motion()
        intermediate = self.window.arm.data.qpos.copy()
        self.assertGreater(intermediate[0],start[0])
        self.assertLess(intermediate[0],np.deg2rad(50))
        self.window.stop_action.trigger()
        self.assertIsNone(self.window.motion)
        self.assertIn("已停止",self.window.status.text())
        np.testing.assert_array_equal(self.window.arm.data.qpos,intermediate)

    def test_physical_tracking_and_live_chart_window(self):
        self.window.control_tabs.setCurrentIndex(1)
        self.window.physics_medium.setCurrentIndex(0)
        with tempfile.TemporaryDirectory() as directory:
            started = self.window.start_tracking(ROOT/"trajectories/feasible_pose.json",output=directory,show_monitor=False)
            self.assertTrue(started,self.window.status.text())
            self.window.tracking_timer.stop()
            self.assertFalse(self.window.sliders[0].isEnabled())
            before = self.window.arm.data.qpos.copy()
            self.window.home()
            np.testing.assert_array_equal(self.window.arm.data.qpos,before)
            self.window.advance_tracking()
            self.assertGreater(self.window.arm.data.time,0)
            while self.window.tracking.status == "running":
                self.window.advance_tracking()
            self.assertEqual(self.window.tracking.status,"completed")
            self.assertTrue((Path(directory)/"telemetry.csv").exists())
            self.assertTrue((Path(directory)/"joint_torque.png").exists())
            self.assertTrue(self.window.sliders[0].isEnabled())
            self.window.show_tracking_monitor()
            QTest.qWait(100)
            (ROOT/"outputs").mkdir(exist_ok=True)
            self.window.tracking_monitor.grab().save(str(ROOT/"outputs/tracking_monitor.png"))
            self.window.tracking_monitor.hide()

    def test_pick_place_example_button_and_physical_completion(self):
        self.window.control_tabs.setCurrentIndex(1)
        self.window.physics_medium.setCurrentIndex(0)
        self.window.example_select.setCurrentIndex(3)
        self.window.reset_block_action.trigger()
        QTest.mouseClick(self.window.start_tracking_button,Qt.MouseButton.LeftButton)
        self.window.tracking_timer.stop()
        self.assertEqual(self.window.tracking.reference.source.mode,"pick_place",self.window.status.text())
        self.assertFalse(self.window.reset_block_action.isEnabled())
        output = ROOT/"outputs/gui_pick_place"
        self.window.tracking_output = output
        self.window.tracking_monitor.hide()
        captured = False
        while self.window.tracking.status == "running":
            for _ in range(500):
                self.window.tracking.step()
                if self.window.tracking.status != "running":
                    break
            if self.window.tracking.status == "running":
                self.window.advance_tracking()
            if not captured and self.window.tracking.last["phase"] == "transfer":
                self.window.viewport.refresh()
                self.window.grab().save(str(ROOT/"outputs/pick_place_lift_gui.png"))
                captured = True
        self.window.finish_tracking()
        self.assertTrue(self.window.tracking.summary()["pick_place"]["success"])
        self.assertIn("抓取放置成功",self.window.status.text())
        self.assertTrue((output/"compiled_sequence.json").exists())
        self.window.viewport.refresh()
        self.window.grab().save(str(ROOT/"outputs/pick_place_done_gui.png"))
        self.window.reset_block_action.trigger()
        self.window.example_select.setCurrentIndex(0)


if __name__ == "__main__":
    unittest.main()
