from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pinocchio as pin
from assembly_policy import load_assembly_policy

def joint_distance(model: pin.Model, first: int, second: int) -> int:
    """Undirected distance in the articulated joint tree, treating universe as root."""
    first_chain, second_chain = [], []
    while first:
        first_chain.append(first)
        first = model.parents[first]
    while second:
        second_chain.append(second)
        second = model.parents[second]
    common = set(first_chain).intersection(second_chain)
    return min(i + second_chain.index(node) for i, node in enumerate(first_chain) if node in common) if common else len(first_chain) + len(second_chain)

def build_pairs(model: pin.Model, geometry: pin.GeometryModel, assembly_policy=None) -> tuple[dict[str, int], list[dict[str, object]]]:
    """Create every non-structural pair and retain a reviewable exclusion record."""
    included = 0
    exclusions: list[dict[str, object]] = []
    assembly_pairs = {frozenset(name + '_0' for name in pair) for pair in assembly_policy['link_pairs']} if assembly_policy else set()
    for first in range(geometry.ngeoms):
        for second in range(first + 1, geometry.ngeoms):
            a, b = geometry.geometryObjects[first], geometry.geometryObjects[second]
            if frozenset((a.name, b.name)) in assembly_pairs:
                exclusions.append({'first': a.name, 'second': b.name, 'reason': 'official_wuji_assembly_overlap',
                                   'source_commit': assembly_policy['official_source_commit']})
                continue
            if a.parentJoint == b.parentJoint:
                exclusions.append({"first": a.name, "second": b.name, "reason": "same_rigid_joint"})
                continue
            distance = joint_distance(model, a.parentJoint, b.parentJoint)
            if distance == 1:
                exclusions.append({"first": a.name, "second": b.name, "reason": "direct_parent_child_joint", "joint_tree_distance": distance})
                continue
            geometry.addCollisionPair(pin.CollisionPair(first, second))
            included += 1
    return {"included_pairs": included, "excluded_pairs": len(exclusions)}, exclusions

def family(name: str) -> str:
    """Classify geometry names without treating two finger branches as structural."""
    if name.startswith(("l_", "r_")):
        return "left_hand" if name.startswith("l_") else "right_hand"
    if any(token in name for token in ("_L_Link", "_R_Link", "elbow_", "wrist_", "proximal_")):
        return "arm"
    return "body"

def pair_family(first: str, second: str) -> str:
    left, right = family(first), family(second)
    if {left, right} == {"left_hand", "right_hand"}:
        return "interhand"
    if left == right and left in {"left_hand", "right_hand"}:
        return "intrahand"
    if "arm" in {left, right} and ("left_hand" in {left, right} or "right_hand" in {left, right}):
        return "arm_hand"
    if "arm" in {left, right} and "body" in {left, right}:
        return "arm_body"
    return "other"

def verify_penetration_depth_sign() -> dict[str, float]:
    """Check Coal's contact convention with two radius-one overlapping spheres."""
    import coal

    request = coal.CollisionRequest()
    request.enable_contact = True
    request.num_max_contacts = 1
    result = coal.CollisionResult()
    count = coal.collide(
        coal.Sphere(1.0), coal.Transform3s(np.eye(3), np.zeros(3)),
        coal.Sphere(1.0), coal.Transform3s(np.eye(3), np.asarray([1.5, 0.0, 0.0])),
        request, result,
    )
    if count != 1 or not result.isCollision() or len(result.getContacts()) != 1:
        raise RuntimeError("Coal sphere overlap convention test did not produce one contact")
    raw_depth = float(result.getContacts()[0].penetration_depth)
    if not np.isclose(raw_depth, -0.5, atol=1e-9):
        raise RuntimeError(f"unexpected Coal penetration depth convention: {raw_depth}")
    return {"overlap_m": 0.5, "raw_penetration_depth_m": raw_depth, "positive_depth_formula": "max(0, -raw_penetration_depth_m)"}

def collision_coverage(urdf: Path, geometry: pin.GeometryModel) -> dict[str, object]:
    root = ET.parse(urdf).getroot()
    links = root.findall("link")
    without_collision = sorted(link.attrib["name"] for link in links if not link.findall("collision"))
    return {
        "urdf_link_count": len(links),
        "urdf_links_without_collision": without_collision,
        "urdf_links_without_collision_count": len(without_collision),
        "pinocchio_collision_geometry_count": geometry.ngeoms,
    }
