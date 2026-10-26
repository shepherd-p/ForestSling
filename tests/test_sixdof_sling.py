import unittest
import numpy as np
from forestlab.sixdof_sling import SixDofSling,payload_asset


class MotorPlantTests(unittest.TestCase):
    def test_team_has_actual_free_aircraft_and_four_motors_each(self):
        for n in (3,4):
            p,h=payload_asset('irregular',n)
            e=SixDofSling(p,h)
            self.assertEqual(e.model.nq,7*(n+1))
            self.assertEqual(e.model.nv,6*(n+1))
            self.assertEqual(e.model.nu,4*n)
            self.assertEqual(set(e.cable_telemetry()),{'time','uav_position','uav_velocity','tension'})
            before=e.data.qpos.copy()
            e.advance(np.zeros((n,4)),.02)
            self.assertGreater(np.linalg.norm(e.data.qpos-before),0.)
            self.assertEqual(e.data.qpos.shape,before.shape)

    def test_slack_cable_never_pushes_and_motors_have_finite_response(self):
        p,h=payload_asset('rectangular',3)
        e=SixDofSling(p,h)
        import mujoco
        for i in range(3): e.data.qpos[7*(i+1)+2]-=.5
        mujoco.mj_forward(e.model,e.data)
        ext,ten=e.cable_state()
        self.assertTrue((ext<0).all());np.testing.assert_array_equal(ten,np.zeros(3))
        initial=e.data.act.copy();e.advance(np.zeros((3,4)),.02)
        self.assertTrue((e.data.act>0).all())
        np.testing.assert_allclose(e.data.act,initial*np.exp(-.02/.025),rtol=1e-10)

    def test_same_shape_different_hidden_ballast_shares_public_model(self):
        for n in (3,4):
            a,ha=payload_asset('irregular',n,'center',71)
            b,hb=payload_asset('irregular',n,'offset_z',71)
            for key in ('cells','half','anchors','nominal_theta'):
                np.testing.assert_array_equal(a[key],b[key])
            self.assertGreater(np.linalg.norm(ha['com']-hb['com']),.05)


if __name__=='__main__':unittest.main()
