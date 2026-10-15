"""
Diagnoses and fixes the Husky wheel joint drives in Isaac Sim.

Usage: Window -> Script Editor -> paste -> Ctrl+Enter
Works with both STOP and PLAY, but apply changes while STOPPED.

Run first with APPLY_FIX = False and read the report.
Then run with APPLY_FIX = True.
"""

from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf
import omni.usd

# ============================ CONFIGURE ============================

ROBOT_PRIM = "/World/husky"

APPLY_FIX = False        # False = report only, True = fix

# Target values for the wheel velocity drives
WHEEL_DAMPING   = 1.0e4   # holds the commanded velocity; too low = sluggish
WHEEL_STIFFNESS = 0.0     # MUST be 0 for velocity control
WHEEL_MAX_FORCE = 1.0e5   # Nm; too low and it cannot move a 46 kg robot

# Wheel friction (skid-steer needs a fair amount, but not too much)
WHEEL_STATIC_FRICTION  = 0.8
WHEEL_DYNAMIC_FRICTION = 0.7
WHEEL_RESTITUTION      = 0.0

WHEEL_KEYWORD = "wheel"   # how to recognise wheels by prim name

# ===================================================================

stage = omni.usd.get_context().get_stage()
root = stage.GetPrimAtPath(ROBOT_PRIM)

if not root.IsValid():
    raise RuntimeError(f"Prim does not exist: {ROBOT_PRIM}  -- check the Stage panel")

print("=" * 70)
print(f"DIAGNOSTICS: {ROBOT_PRIM}")
print("=" * 70)

# ---------- 1. Articulation root ----------
art_roots = [p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
print(f"\n[1] ArticulationRootAPI found on {len(art_roots)} prim(s):")
for p in art_roots:
    print(f"    {p.GetPath()}")
if len(art_roots) == 0:
    print("    !! No articulation root -- the robot is not an articulation.")
elif len(art_roots) > 1:
    print("    !! More than one root -- physics will behave unpredictably.")

# ---------- 2. Is the base pinned to the world ----------
fixed_joints_to_world = []
for p in Usd.PrimRange(root):
    if p.IsA(UsdPhysics.FixedJoint):
        j = UsdPhysics.Joint(p)
        b0 = j.GetBody0Rel().GetTargets()
        b1 = j.GetBody1Rel().GetTargets()
        if not b0 or not b1:
            fixed_joints_to_world.append(p.GetPath())
print(f"\n[2] Fixed joints to world: {len(fixed_joints_to_world)}")
for fp in fixed_joints_to_world:
    print(f"    !! {fp}  -- this pins the robot in place, delete it")

# ---------- 3. Joints and drives ----------
print("\n[3] Revolute joints:")
wheel_joints = []
for p in Usd.PrimRange(root):
    if not p.IsA(UsdPhysics.RevoluteJoint):
        continue
    name = p.GetName()
    joint = UsdPhysics.RevoluteJoint(p)
    axis = joint.GetAxisAttr().Get()

    drive = UsdPhysics.DriveAPI.Get(p, "angular")
    if drive:
        stiff = drive.GetStiffnessAttr().Get()
        damp = drive.GetDampingAttr().Get()
        maxf = drive.GetMaxForceAttr().Get()
        dtype = drive.GetTypeAttr().Get()
    else:
        stiff = damp = maxf = dtype = None

    is_wheel = WHEEL_KEYWORD in name.lower()
    if is_wheel:
        wheel_joints.append(p)

    flag = ""
    if is_wheel and stiff is not None and stiff > 0.0:
        flag = "  <<< PROBLEM: stiffness > 0 acts as a brake"
    if is_wheel and drive is None:
        flag = "  <<< PROBLEM: no DriveAPI, the joint is free-spinning"

    print(f"    {name:32s} axis={axis} stiff={stiff} damp={damp} "
          f"maxForce={maxf} type={dtype}{flag}")

print(f"\n    Wheels detected: {len(wheel_joints)}")
if len(wheel_joints) != 4:
    print("    !! Expected 4. Check WHEEL_KEYWORD or the joint names.")

# ---------- 4. Masses ----------
print("\n[4] Rigid bodies and masses:")
total_mass = 0.0
for p in Usd.PrimRange(root):
    if not p.HasAPI(UsdPhysics.RigidBodyAPI):
        continue
    mass_api = UsdPhysics.MassAPI.Get(stage, p.GetPath())
    m = mass_api.GetMassAttr().Get() if mass_api else None
    if m:
        total_mass += m
    warn = "  <<< mass is 0 or undefined" if (m is None or m == 0.0) else ""
    print(f"    {str(p.GetName()):32s} mass={m}{warn}")
print(f"    Total: {total_mass:.2f} kg   (Husky should be ~46-50 kg)")

# ---------- 5. Wheel colliders ----------
# NOTE: Isaac marks imported meshes as instanceable, so a plain PrimRange
# will not see them. TraverseInstanceProxies is required.
print("\n[5] Wheel colliders:")
collider_count = 0
for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
    if WHEEL_KEYWORD not in str(p.GetPath()).lower():
        continue
    if p.HasAPI(UsdPhysics.CollisionAPI):
        collider_count += 1
        print(f"    {p.GetPath()}  (instance proxy: {p.IsInstanceProxy()})")
if collider_count == 0:
    print("    !! No colliders found on the wheels.")

# ===================================================================
#                              FIXES
# ===================================================================

if not APPLY_FIX:
    print("\n" + "=" * 70)
    print("APPLY_FIX = False -- nothing was changed.")
    print("Read the report, then set APPLY_FIX = True and run again.")
    print("=" * 70)
else:
    print("\n" + "=" * 70)
    print("APPLYING FIXES")
    print("=" * 70)

    # --- 5a. Velocity drive on the wheels ---
    for p in wheel_joints:
        drive = UsdPhysics.DriveAPI.Get(p, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(p, "angular")

        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(WHEEL_STIFFNESS)
        drive.CreateDampingAttr().Set(WHEEL_DAMPING)
        drive.CreateMaxForceAttr().Set(WHEEL_MAX_FORCE)
        drive.CreateTargetVelocityAttr().Set(0.0)

        # Without this, PhysX may clamp joint velocity to a low default
        pj = PhysxSchema.PhysxJointAPI.Apply(p)
        pj.CreateMaxJointVelocityAttr().Set(1000.0)   # deg/s
        pj.CreateJointFrictionAttr().Set(0.0)

        print(f"    velocity drive -> {p.GetName()}")

    # --- 5b. Friction material ---
    mat_path = Sdf.Path("/World/PhysicsMaterials/WheelMaterial")
    if not stage.GetPrimAtPath(mat_path).IsValid():
        UsdGeom.Scope.Define(stage, "/World/PhysicsMaterials")
        mat_prim = stage.DefinePrim(mat_path, "Material")
    else:
        mat_prim = stage.GetPrimAtPath(mat_path)

    mat = UsdPhysics.MaterialAPI.Apply(mat_prim)
    mat.CreateStaticFrictionAttr().Set(WHEEL_STATIC_FRICTION)
    mat.CreateDynamicFrictionAttr().Set(WHEEL_DYNAMIC_FRICTION)
    mat.CreateRestitutionAttr().Set(WHEEL_RESTITUTION)
    print(f"    material: {mat_path}")

    from pxr import UsdShade
    bound = 0
    # TraverseInstanceProxies again -- otherwise 0 prims get bound
    for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if WHEEL_KEYWORD not in str(p.GetPath()).lower():
            continue
        if not p.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if p.IsInstanceProxy():
            # Instance proxies are read-only. Uncheck "Instanceable" on the
            # parent prim in the Property panel, then re-run this script.
            print(f"    SKIPPED (instance proxy, read-only): {p.GetPath()}")
            continue
        binding = UsdShade.MaterialBindingAPI.Apply(p)
        binding.Bind(
            UsdShade.Material(mat_prim),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics",
        )
        bound += 1
    print(f"    material bound to {bound} collider prims")

    # --- 5c. Disable articulation self-collision ---
    for p in art_roots:
        pa = PhysxSchema.PhysxArticulationAPI.Apply(p)
        pa.CreateEnabledSelfCollisionsAttr().Set(False)
        pa.CreateSolverPositionIterationCountAttr().Set(32)
        pa.CreateSolverVelocityIterationCountAttr().Set(4)
        print(f"    self-collision off, solver iterations raised -> {p.GetPath()}")

    print("\nDONE. Press STOP then PLAY so physics picks up the new values.")
    print("=" * 70)
