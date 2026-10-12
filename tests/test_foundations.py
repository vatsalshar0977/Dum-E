import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from src.phase5_control.kinematics import forward, inverse, LIMITS
from src.phase5_control.trajectory import plan
from src.phase6_perception.camera_and_hand_eye_calib import intersect_table
from src.phase6_perception.vision_mobile_navigator import Reading, choose_wheels

ROOT=Path(__file__).resolve().parents[1]

class Foundations(unittest.TestCase):
    def test_urdf_chain_and_limits(self):
        root=ET.parse(ROOT/'urdf/bude_arm.urdf').getroot()
        links={x.attrib['name'] for x in root.findall('link')}
        joints={x.attrib['name']:x for x in root.findall('joint')}
        self.assertEqual(len(links),7)
        for name in ('base_yaw','shoulder_pitch','elbow_pitch','wrist_pitch','wrist_roll','tcp_fixed'):
            self.assertIn(name,joints)
            self.assertIn(joints[name].find('parent').attrib['link'],links)
            self.assertIn(joints[name].find('child').attrib['link'],links)
        for name, length in [('base_yaw',.055),('elbow_pitch',.08),('wrist_pitch',.075),('wrist_roll',.022),('tcp_fixed',.048)]:
            self.assertAlmostEqual(float(joints[name].find('origin').attrib['xyz'].split()[2]),length)
        for name, limits in zip(('base_yaw','shoulder_pitch','elbow_pitch','wrist_pitch','wrist_roll'),LIMITS):
            joint=joints[name]
            self.assertAlmostEqual(float(joint.find('limit').attrib['lower']),math.radians(limits[0]),places=5)
            self.assertAlmostEqual(float(joint.find('limit').attrib['upper']),math.radians(limits[1]),places=5)
        self.assertAlmostEqual(forward((0,0,0,0,0)).z,280)

    def test_fk_ik_roundtrip(self):
        for q in ((0,-15,40,-25,0),(20,45,35,0,15),(-25,55,50,25,-30)):
            p=forward(q)
            answers=inverse(p.x,p.y,p.z,p.pitch,p.roll)
            self.assertTrue(any(max(abs(x-y) for x,y in zip(q,a))<1e-5 for a in answers))
        with self.assertRaises(ValueError): inverse(500,0,20,0)
        with self.assertRaises(ValueError): inverse(float('nan'),0,20,0)
        with self.assertRaises(ValueError): inverse(20,0,20,0,100)

    def test_path_valid_and_invalid(self):
        home=(0,-15,40,-25,0)
        path=plan(home,(15,0,35,-15,20))
        self.assertEqual(path[0][1],home)
        self.assertTrue(all(t2>t1 for (t1,_),(t2,_) in zip(path,path[1:])))
        self.assertTrue(max(abs(a-b) for a,b in zip(path[-1][1],(15,0,35,-15,20)))<1e-8)
        with self.assertRaises(ValueError): plan(home,(90,85,90,85,0)) # desk collision
        with self.assertRaises(ValueError): plan(home,(100,0,0,0,0))

    def test_plane_projection_and_bad_calibration(self):
        K=dict(fx=500,fy=500,cx=320,cy=240)
        # Camera at (10,20,100) pointing down. Camera x=base x; camera y=-base y.
        ext=dict(R=[[1,0,0],[0,-1,0],[0,0,-1]], t_mm=[10,20,100])
        self.assertEqual(intersect_table(320,240,K,ext), (10.,20.,0.))
        x,y,z=intersect_table(370,265,K,ext,z_mm=20)
        self.assertAlmostEqual(x,18);self.assertAlmostEqual(y,16);self.assertAlmostEqual(z,20)
        with self.assertRaises(ValueError): intersect_table(320,240,K,dict(R=[[1,0,0]]*3,t_mm=[0,0,100]))
        with self.assertRaises(ValueError): intersect_table(320,240,K,dict(R=[[1,0,0],[0,1,0],[0,0,1]],t_mm=[0,0,100]))

    def test_mobile_fail_closed(self):
        good=Reading(2,1.95,0,1,True,True,True,True)
        self.assertNotEqual(choose_wheels(good),(0,0))
        for changed in (dict(sensor_time_s=1.0),dict(left_floor_ok=False),dict(estop_ok=False),
                        dict(arm_stowed=False),dict(obstacle_m=.2),dict(target_bearing_rad=None)):
            data={**vars(good),**changed}
            self.assertEqual(choose_wheels(Reading(**data)),(0,0))

if __name__=='__main__': unittest.main()
