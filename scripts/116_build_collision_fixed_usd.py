"""Build a standalone physics collision variant without modifying source meshes."""
import argparse,hashlib,json
from pathlib import Path
from pxr import Usd,UsdGeom,UsdPhysics,Sdf
p=argparse.ArgumentParser();p.add_argument('--sdf-resolution',type=int,default=512);p.add_argument('--hand-approximation',choices=['convexDecomposition','sdf'],default='convexDecomposition');p.add_argument('--source',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.parent.mkdir(parents=True,exist_ok=True)
if a.out.exists():raise FileExistsError(a.out)
stage=Usd.Stage.Open(str(a.source));root=stage.GetDefaultPrim();changes=[]
# Edit collision instances only; visual geometry and mounting stay unchanged.
for link in list(root.GetChildren()):
 collision=stage.GetPrimAtPath(str(link.GetPath())+'/collisions')
 if not collision:continue
 while True:
  instances=[x for x in Usd.PrimRange(collision) if x.IsInstanceable()]
  if not instances:break
  for prim in instances:prim.SetInstanceable(False)
 for prim in list(Usd.PrimRange(collision)):
  if not prim.HasAPI(UsdPhysics.CollisionAPI) or prim.IsA(UsdGeom.Gprim):continue
  old=str(UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get());children=[x for x in Usd.PrimRange(prim) if x.IsA(UsdGeom.Mesh)]
  if len(children)!=1:raise ValueError(str(prim.GetPath()))
  prim.RemoveAPI(UsdPhysics.CollisionAPI);prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
  mesh=children[0];UsdPhysics.CollisionAPI.Apply(mesh);UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr(a.hand_approximation if link.GetName().startswith(('l_','r_')) else old)
  if link.GetName().startswith(('l_','r_')):
   schemas=list(mesh.GetAppliedSchemas());schemas.append('PhysxConvexDecompositionCollisionAPI');mesh.SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(schemas))
   for key,val in [('hullVertexLimit',64),('maxConvexHulls',32),('voxelResolution',200000)]:mesh.CreateAttribute('physxConvexDecompositionCollision:'+key,Sdf.ValueTypeNames.Int).Set(val)
  if link.GetName().startswith(('l_','r_')) and a.hand_approximation=='sdf':
   schemas=[x for x in mesh.GetAppliedSchemas() if x!='PhysxConvexDecompositionCollisionAPI'];schemas.append('PhysxSDFMeshCollisionAPI');mesh.SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(schemas));mesh.CreateAttribute('physxSDFMeshCollision:sdfResolution',Sdf.ValueTypeNames.Int).Set(a.sdf_resolution)
  changes.append({'from':str(prim.GetPath()),'to':str(mesh.GetPath()),'source_approximation':old,'new_approximation':UsdPhysics.MeshCollisionAPI(mesh).GetApproximationAttr().Get()})
# Match the existing auditor's same-rigid/direct-joint exclusions in PhysX.
coverage=json.loads((Path(__file__).resolve().parents[1]/'evidence/baseline_coverage.json').read_text())
if hashlib.sha256(a.source.read_bytes()).hexdigest()!=coverage['usd_sha256']:raise ValueError('source model does not match reviewed collision exclusions')
exclusions=coverage['exclusions']
for pair in exclusions:
 first=stage.GetPrimAtPath(str(root.GetPath())+'/'+pair['first'].rsplit('_',1)[0]);second=stage.GetPrimAtPath(str(root.GetPath())+'/'+pair['second'].rsplit('_',1)[0])
 if not first or not second:raise ValueError(pair)
 UsdPhysics.FilteredPairsAPI.Apply(first).CreateFilteredPairsRel().AddTarget(second.GetPath())
for prim in stage.Traverse():
 if prim.HasAPI(UsdPhysics.ArticulationRootAPI):prim.CreateAttribute('physxArticulation:enabledSelfCollisions',Sdf.ValueTypeNames.Bool).Set(True)
stage.Flatten().Export(str(a.out))
(a.out.parent/'model_changes.json').write_text(json.dumps({'source':str(a.source.resolve()),'source_sha256':hashlib.sha256(a.source.read_bytes()).hexdigest(),'usd':str(a.out.resolve()),'usd_sha256':hashlib.sha256(a.out.read_bytes()).hexdigest(),'changes':changes,'self_collision_enabled':True,'filtered_pairs':exclusions,'mesh_points_changed':False,'mount_transforms_changed':False},indent=2))
print('collision declarations repaired:',len(changes))
