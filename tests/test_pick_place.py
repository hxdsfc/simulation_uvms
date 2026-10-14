import unittest
from pathlib import Path
import numpy as np
from uvms.kinematics import ArmKinematics
from uvms.pick_place import load_reference
from uvms.tracking import TrackingSession

ROOT = Path(__file__).resolve().parents[1]


class PickPlaceTests(unittest.TestCase):
    def test_air_and_water_contact_pick_lift_transfer_release(self):
        for medium in ("air","water"):
            with self.subTest(medium=medium):
                arm = ArmKinematics(medium)
                q,v = arm.data.qpos.copy(),arm.data.qvel.copy()
                reference = load_reference(arm,ROOT/"trajectories/pick_place.json")
                np.testing.assert_array_equal(arm.data.qpos,q)
                np.testing.assert_array_equal(arm.data.qvel,v)
                self.assertEqual(arm.model.neq,2)  # Source jaw mimics only; no object attachment.
                session = TrackingSession(arm,reference)
                result = session.run()
                self.assertEqual(result["status"],"completed")
                task = result["pick_place"]
                self.assertTrue(task["success"])
                self.assertGreater(task["lift_height_m"],.05)
                self.assertLess(task["place_xy_error_m"],.001)
                self.assertTrue(task["released"] and task["supported_by_table"])
                carried = [r for r in session.rows if r["phase"] == "transfer"]
                self.assertTrue(all(r["grasp_contact_sides"] == 2 for r in carried))
                self.assertTrue(all(r["object_position_m"][2] > reference.pick_position[2]+.05 for r in carried))
                self.assertLess(np.max(np.abs([r["tau_actual_nm"] for r in session.rows])),9)
                self.assertLess(np.max(np.abs([r["gripper_force_command_n"] for r in session.rows])),10.00001)
                released = [r for r in session.rows if r["phase"] == "release"]
                self.assertLess(np.max(np.abs([r["gripper_force_command_n"] for r in released])),1)
                self.assertEqual(result["saturation_control_evaluations"],0)
                for a,b in zip(reference.segments[:-1],reference.segments[1:]):
                    before = reference.sample(a["start"]+a["duration"]-1e-6)
                    after = reference.sample(b["start"]+1e-6)
                    np.testing.assert_allclose(before["q"],after["q"],atol=1e-8)
                    self.assertLess(np.max(np.abs(after["qd"])),1e-8)

    def test_wrong_object_position_is_rejected_without_state_mutation(self):
        arm = ArmKinematics("air")
        arm.data.qpos[arm.environment_q_indices[0]] += .05
        q,v = arm.data.qpos.copy(),arm.data.qvel.copy()
        with self.assertRaisesRegex(ValueError,"重置方块"):
            load_reference(arm,ROOT/"trajectories/pick_place.json")
        np.testing.assert_array_equal(arm.data.qpos,q)
        np.testing.assert_array_equal(arm.data.qvel,v)

    def test_intentional_grip_does_not_exempt_robot_table_contact(self):
        from types import SimpleNamespace
        arm = ArmKinematics("air")
        ref = load_reference(arm,ROOT/"trajectories/pick_place.json")
        pad = next(iter(ref.pad_ids))
        self.assertTrue(ref.allowed_contact(SimpleNamespace(geom1=pad,geom2=ref.block_geom),"close"))
        self.assertFalse(ref.allowed_contact(SimpleNamespace(geom1=pad,geom2=ref.block_geom),"approach"))
        self.assertFalse(ref.allowed_contact(SimpleNamespace(geom1=pad,geom2=ref.table_geom),"lower"))


if __name__ == "__main__":
    unittest.main()
