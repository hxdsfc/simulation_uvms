"""Expand upstream Alpha 5 Xacro and import its URDF into MuJoCo."""
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
import trimesh
import xacro

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "vendor/reach_description"
MODELS = ROOT / "models"


def main():
    cfg = json.loads((ROOT / "config/alpha5.json").read_text(encoding="utf-8"))
    scene_cfg = json.loads((ROOT / "config/scene.json").read_text(encoding="utf-8"))
    MODELS.mkdir(exist_ok=True)
    expanded = MODELS / "source"
    expanded.mkdir(exist_ok=True)
    for filename in ("description/alpha_5/urdf.xacro", "description/alpha_5/standard_jaws/standard_jaws.urdf.xacro"):
        content = (SOURCE / filename).read_text(encoding="utf-8")
        content = content.replace("$(find reach_description)", SOURCE.as_posix())
        (expanded / Path(filename).name).write_text(content, encoding="utf-8")
    wrapper = f'''<robot name="reach_alpha_5" xmlns:xacro="http://ros.org/wiki/xacro">
      <xacro:include filename="urdf.xacro"/>
      <xacro:include filename="standard_jaws.urdf.xacro"/>
      <link name="world"/>
      <xacro:alpha_5_urdf parent="world" prefix="" joint_limits_file="{SOURCE.as_posix()}/config/alpha_5/joint_limits.yaml">
        <origin xyz="0.0374 0 {cfg['mount_height_m'] + 0.0225}" rpy="0 0 0"/>
      </xacro:alpha_5_urdf>
      <xacro:standard_jaws parent="rs1_124_link" prefix="" mimic_joint="axis_a"/>
    </robot>'''
    (expanded / "wrapper.xacro").write_text(wrapper, encoding="utf-8")
    urdf = ET.fromstring(xacro.process_file(str(expanded / "wrapper.xacro")).toxml())
    assets = MODELS / "assets"
    assets.mkdir(exist_ok=True)
    for mesh in urdf.findall(".//mesh"):
        source = Path(mesh.get("filename").removeprefix("file://"))
        dest = assets / (source.stem + ".obj")
        geometry = trimesh.load_scene(source, process=False).to_mesh()
        if not np.isfinite(geometry.vertices).all() or len(geometry.faces) == 0:
            raise ValueError(f"Invalid mesh: {source}")
        geometry.export(dest, include_texture=False)
        dest.write_text(dest.read_text(encoding="utf-8").rstrip("\r\n")+"\n",encoding="utf-8",newline="\n")
        mesh.set("filename", "assets/" + dest.name)
    ET.SubElement(urdf, "mujoco").append(ET.Element("compiler", discardvisual="false", fusestatic="false"))
    ET.indent(urdf)
    ET.ElementTree(urdf).write(MODELS / "alpha5.urdf", encoding="utf-8", xml_declaration=True)
    imported = mujoco.MjModel.from_xml_path(str(MODELS / "alpha5.urdf"))
    mujoco.mj_saveLastXML(str(MODELS / "alpha5_imported.xml"), imported)
    base = ET.parse(MODELS / "alpha5_imported.xml").getroot()
    base.set("model", "reach_alpha_5")
    base.find("compiler").set("meshdir", "assets")
    for mesh in base.findall("asset/mesh"):
        mesh.set("file", Path(mesh.get("file")).name)
    for geom in base.findall(".//geom"):
        if geom.get("type") == "mesh":
            geom.set("rgba", "0.3 0.38 0.43 1")
            geom.set("fluidshape", "none")
        else:
            geom.set("rgba", "0.2 0.7 0.8 0")
            geom.set("fluidshape", "ellipsoid")
    for joint in base.findall("worldbody/.//joint"):
        joint.set("damping", str(cfg["joint_damping"]))
        joint.set("armature", str(cfg["joint_armature"]))
    # Contact pads replace upstream bounding boxes which fill the jaw opening.
    # These are simulation contact approximations, not calibrated hardware pads.
    for name,sign in (("rs1_130_link",-1),("rs1_139_link",1)):
        body = base.find(f".//body[@name='{name}']")
        for geom in list(body.findall("geom")):
            if geom.get("type") != "mesh":
                body.remove(geom)
        ET.SubElement(body,"geom",name="pad_"+name,type="box",pos=f"0 {sign*.008} .05",
                      size=".011 .002 .015",rgba="0.12 0.15 0.17 1",friction="1.5 .01 .001",
                      quat=f"{math.cos(.17/2)} {-sign*math.sin(.17/2)} 0 0",
                      condim="4",solref=".005 1",solimp=".99 .99 .001",fluidshape="ellipsoid")
    ET.SubElement(base.find(".//body[@name='rs1_124_link']"),"site",name="grasp_center",
                  pos="0 0 .0569",size=".003",rgba="0.2 0.8 0.3 0")
    palm = base.find(".//body[@name='rs1_124_link']")
    for geom in list(palm.findall("geom")):
        if geom.get("type") != "mesh":
            palm.remove(geom)
    palm_collision = copy.deepcopy(palm.find("geom"))
    palm_collision.attrib.update(name="palm_collision",contype="1",conaffinity="1",group="3",rgba="0.2 0.7 0.8 0",fluidshape="ellipsoid")
    palm.append(palm_collision)
    world = base.find("worldbody")
    contact = ET.SubElement(base, "contact")
    # Upstream coarse collision proxies overlap at the base bearing and jaws.
    # Ignore these structural pairs; retain other self and environment collisions.
    ET.SubElement(contact, "exclude", body1="m3_inline_link", body2="m2_1_1_link")
    ET.SubElement(contact, "exclude", body1="rs1_130_link", body2="rs1_139_link")
    assets_xml = base.find("asset")
    ET.SubElement(assets_xml,"texture",name="floor_checker",type="2d",builtin="checker",
                  rgb1="0.14 0.18 0.23",rgb2="0.18 0.22 0.27",width="512",height="512")
    ET.SubElement(assets_xml,"material",name="floor_material",texture="floor_checker",texrepeat="8 8",reflectance="0.05")
    ET.SubElement(world, "geom", name="floor", type="plane", size="1 1 0.1", material="floor_material")
    pedestal = ET.SubElement(world, "body", name="pedestal", pos=f"0 0 {cfg['mount_height_m']/2}")
    ET.SubElement(pedestal, "geom", type="box", size=f"0.12 0.06 {cfg['mount_height_m']/2}", rgba="0.35 0.4 0.45 1",
                  contype="0", conaffinity="0")
    table,block = scene_cfg["table"],scene_cfg["block"]
    for value in (*table["size_xy_m"],table["top_height_m"],table["thickness_m"],block["edge_m"],block["mass_kg"]):
        if not math.isfinite(value) or value <= 0:
            raise ValueError("Scene dimensions and mass must be positive")
    tx,ty = table["center_xy_m"]
    sx,sy = np.array(table["size_xy_m"])/2
    height,thickness = table["top_height_m"],table["thickness_m"]
    platform = ET.SubElement(world,"body",name="work_table")
    ET.SubElement(platform,"geom",name="table_top",type="box",pos=f"{tx} {ty} {height-thickness/2}",
                  size=f"{sx} {sy} {thickness/2}",rgba="0.57 0.68 0.72 1",friction="0.8 0.005 0.0001")
    # Solid table edges and legs remain fixed to the world, with actual collisions.
    for i,(dx,dy) in enumerate(((-1,-1),(-1,1),(1,-1),(1,1))):
        ET.SubElement(platform,"geom",name=f"table_leg_{i}",type="box",
                      pos=f"{tx+dx*(sx-.025)} {ty+dy*(sy-.025)} {(height-thickness)/2}",
                      size=f"0.012 0.012 {(height-thickness)/2}",rgba="0.24 0.31 0.38 1")
    # Painted alignment marks are visual-only and do not alter contact geometry.
    bx,by = block["position_xy_m"]
    for index,offset in enumerate((-.038,.038)):
        ET.SubElement(platform,"geom",name=f"pickup_mark_{index}",type="box",
                      pos=f"{bx+offset} {by} {height+.00015}",size="0.001 0.038 0.0001",
                      rgba="0.92 0.79 0.35 1",contype="0",conaffinity="0",group="1")
    edge = block["edge_m"]
    block_position = [bx,by,height+edge/2+block["initial_clearance_m"]]
    yaw = block.get("yaw_rad",0)
    block_quaternion = [math.cos(yaw/2),0,0,math.sin(yaw/2)]
    cube = ET.SubElement(world,"body",name="grasp_block",pos=" ".join(map(str,block_position)),
                         quat=" ".join(map(str,block_quaternion)))
    if "place_position_xy_m" in scene_cfg:
        px,py = scene_cfg["place_position_xy_m"]
        ET.SubElement(platform,"geom",name="place_mark",type="box",pos=f"{px} {py} {height+.00015}",
                      size=".027 .027 .0001",rgba="0.25 0.75 0.48 1",contype="0",conaffinity="0",group="1")
    ET.SubElement(cube,"freejoint",name="block_free")
    ET.SubElement(cube,"geom",name="block_geom",type="box",size=" ".join([str(edge/2)]*3),mass=str(block["mass_kg"]),
                  rgba="0.98 0.48 0.12 1",friction=" ".join(map(str,block["friction"])),condim="4",
                  solref="0.005 1",solimp=".99 .99 .001",fluidshape="ellipsoid")
    ET.SubElement(cube,"site",name="block_center",size="0.001",rgba="1 0.75 0.2 0")
    ET.SubElement(world, "light", pos="0.3 -0.5 1.5", dir="0 0 -1", diffuse="0.9 0.9 0.9")
    ET.SubElement(world,"light",pos="-0.4 0.5 1.0",dir="0.3 -0.3 -1",diffuse="0.5 0.5 0.5",castshadow="false")
    camera_pos = np.array([0.95,-1.05,0.9])
    forward = np.array(scene_cfg["camera"]["lookat_m"])-camera_pos
    forward /= np.linalg.norm(forward)
    right = np.cross(forward,[0,0,1]); right /= np.linalg.norm(right)
    up = np.cross(right,forward)
    ET.SubElement(world,"camera",name="overview",pos=" ".join(map(str,camera_pos)),xyaxes=" ".join(map(str,np.r_[right,up])))
    ET.SubElement(base, "statistic", center="0.20 -0.08 0.33", extent="0.8")
    visual = ET.SubElement(base, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="960")
    ET.SubElement(base.find(".//body[@name='tcp']"), "site", name="tool_tip", size="0.007", rgba="1 0.2 0.1 1")
    # TCP is a coordinate frame, not an extra fluid-displacing link. Its source
    # placeholder mass/inertia otherwise creates a metres-wide inertial ellipsoid.
    ET.SubElement(base.find(".//body[@name='tcp']"),"geom",name="tcp_fluid_frame",type="sphere",size=".0001",
                  contype="0",conaffinity="0",density="0",rgba="0 0 0 0",group="3",
                  fluidshape="ellipsoid",fluidcoef="0 0 0 0 0")
    act = ET.SubElement(base, "actuator")
    sensors = ET.SubElement(base, "sensor")
    source_joints = {j.get("name"): j for j in urdf.findall("joint")}
    for name in cfg["actuated_joints"]:
        limit = source_joints[name].find("limit").get("effort")
        ET.SubElement(act, "motor", name="motor_" + name, joint=name, gear="1", ctrllimited="true",
                      ctrlrange=f"-{limit} {limit}", forcelimited="true", forcerange=f"-{limit} {limit}")
        ET.SubElement(sensors, "jointpos", name="q_"+name, joint=name)
        ET.SubElement(sensors, "jointvel", name="dq_"+name, joint=name)
    ET.SubElement(sensors, "framepos", name="tool_position", objtype="site", objname="tool_tip")
    # MuJoCo's URDF importer preserves mimic joints as equality constraints.
    if len(base.findall("equality/joint")) != 2:
        raise ValueError("Expected both gripper mimic constraints")
    for equality in base.findall("equality/joint"):
        equality.set("solref",".002 1")
        equality.set("solimp",".999 .999 .001")
    # Jaw upper angle / pushrod multiplier: reconcile upstream travel mismatch.
    base.find(".//joint[@name='axis_a']").set("range", f"0 {0.5/51:.10g}")
    home = [cfg["home"][mujoco.mj_id2name(imported, mujoco.mjtObj.mjOBJ_JOINT, i)] for i in range(imported.njnt)]
    home += block_position+block_quaternion
    ET.SubElement(ET.SubElement(base, "keyframe"), "key", name="home", qpos=" ".join(str(v) for v in home))
    for medium in ("air", "water"):
        root = copy.deepcopy(base)
        fluid = cfg["water"] if medium == "water" else {"density": 0, "viscosity": 0}
        ET.SubElement(root, "option", timestep=str(cfg["timestep_s"]), integrator="implicitfast", gravity="0 0 -9.81",
                      impratio="10",noslip_iterations="20",iterations="100",
                      **{k: str(v) for k,v in fluid.items()})
        ET.indent(root)
        dest = MODELS / f"arm_{medium}.xml"
        ET.ElementTree(root).write(dest, encoding="utf-8", xml_declaration=True)
        mujoco.MjModel.from_xml_path(str(dest))
        print(dest)


if __name__ == "__main__":
    main()
