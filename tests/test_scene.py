import unittest
import json
from pathlib import Path
import numpy as np
import mujoco
from uvms.kinematics import ArmKinematics
from uvms.motion import MotionPlanner


class GraspSceneTests(unittest.TestCase):
    scene = json.loads((Path(__file__).resolve().parents[1]/"config/scene.json").read_text())
    def test_block_is_free_and_lands_on_table(self):
        arm = ArmKinematics("air")
        model,data = arm.model,arm.data
        self.assertEqual((model.nq,model.nv,model.nu),(14,13,5))
        self.assertEqual(model.joint("block_free").type,mujoco.mjtJoint.mjJNT_FREE)
        block = model.body("grasp_block").id
        self.assertAlmostEqual(model.body_mass[block],0.06)
        for _ in range(400):
            mujoco.mj_step(model,data)
        mujoco.mj_forward(model,data)
        np.testing.assert_allclose(data.xpos[block][:2],self.scene["block"]["position_xy_m"],atol=1e-4)
        self.assertAlmostEqual(data.xpos[block,2],self.scene["table"]["top_height_m"]+self.scene["block"]["edge_m"]/2,delta=1e-4)
        self.assertTrue(any({model.geom(c.geom1).name,model.geom(c.geom2).name} == {"table_top","block_geom"}
                            for c in data.contact))
        self.assertFalse(any(w.number for w in data.warning))

    def test_supported_block_contact_is_not_robot_collision(self):
        arm = ArmKinematics("air")
        # Move only the free block just into the tabletop, leaving the robot at home.
        qadr = arm.model.jnt_qposadr[arm.model.joint("block_free").id]
        arm.data.qpos[qadr+2] = self.scene["table"]["top_height_m"]+self.scene["block"]["edge_m"]/2-.00002
        mujoco.mj_forward(arm.model,arm.data)
        self.assertGreater(arm.data.ncon,0)
        self.assertFalse(any(arm.robot_contact(c) for c in arm.data.contact))
        self.assertTrue(arm.state()["collision_free"])
        result = arm.inverse(arm.position(),seed=arm.home_q,unit="rad",reject_collisions=True)
        self.assertTrue(result["success"])

    def test_kinematics_preserves_object_state_and_planner_copies_it(self):
        arm = ArmKinematics("air")
        qids,vids = arm.environment_q_indices,arm.environment_dof_indices
        arm.data.qpos[qids[0]] += .025
        arm.data.qvel[vids[0]] = .01
        saved_q,saved_v = arm.data.qpos[qids].copy(),arm.data.qvel[vids].copy()
        arm.forward(arm.home_q,unit="rad")
        np.testing.assert_array_equal(arm.data.qpos[qids],saved_q)
        np.testing.assert_array_equal(arm.data.qvel[vids],saved_v)
        planner = MotionPlanner(arm)
        planner.joint(arm.home_q)
        np.testing.assert_array_equal(planner.probe.data.qpos[qids],saved_q)

    def test_block_approach_point_has_a_limit_compliant_solution(self):
        arm = ArmKinematics("air")
        # Seed the side-grasp branch used by the checked pick/place sequence.
        target = arm.forward([1,.94,2.5707963268,2.0943951],unit="rad",gripper=1)["position_m"]
        result = arm.inverse(target,seed=[1,.94,2.5707963268,2.0943951],unit="rad",gripper=1,reject_collisions=True)
        self.assertTrue(result["success"])
        self.assertLess(result["error_m"],1e-5)


if __name__ == "__main__":
    unittest.main()
