"""
Dijagnostika i popravak pogona kotaca Huskyja u Isaac Simu.

Pokretanje: Window -> Script Editor -> zalijepi -> Ctrl+Enter
Radi i sa STOP i s PLAY, ali izmjene primijeni dok je simulacija STOPIRANA.

Prvo pokreni s APPLY_FIX = False i procitaj ispis.
Zatim s APPLY_FIX = True.
"""

from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf
import omni.usd

# ============================ PODESI ============================

ROBOT_PRIM = "/World/husky"

APPLY_FIX = False        # False = samo ispisi, True = popravi

# Ciljne vrijednosti za velocity drive na kotacima
WHEEL_DAMPING   = 1.0e4   # drzi zadanu brzinu; premalo = trom, previse = krut
WHEEL_STIFFNESS = 0.0     # MORA biti 0 za upravljanje brzinom
WHEEL_MAX_FORCE = 1.0e5   # Nm; premalo = ne moze pokrenuti 46 kg robota

# Trenje kotaca (skid-steer treba dosta, ali ne previse)
WHEEL_STATIC_FRICTION  = 0.8
WHEEL_DYNAMIC_FRICTION = 0.7
WHEEL_RESTITUTION      = 0.0

WHEEL_KEYWORD = "wheel"   # kako prepoznati kotace po imenu prima

# ================================================================

stage = omni.usd.get_context().get_stage()
root = stage.GetPrimAtPath(ROBOT_PRIM)

if not root.IsValid():
    raise RuntimeError(f"Prim ne postoji: {ROBOT_PRIM}  -- provjeri Stage panel")

print("=" * 70)
print(f"DIJAGNOSTIKA: {ROBOT_PRIM}")
print("=" * 70)

# ---------- 1. Articulation root ----------
art_roots = [p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
print(f"\n[1] ArticulationRootAPI pronaden na {len(art_roots)} prim(a):")
for p in art_roots:
    print(f"    {p.GetPath()}")
if len(art_roots) == 0:
    print("    !! NEMA articulation roota -- robot nije artikulacija.")
elif len(art_roots) > 1:
    print("    !! VISE od jednog roota -- fizika ce se ponasati nepredvidivo.")

# ---------- 2. Je li baza fiksirana ----------
fixed_joints_to_world = []
for p in Usd.PrimRange(root):
    if p.IsA(UsdPhysics.FixedJoint):
        j = UsdPhysics.Joint(p)
        b0 = j.GetBody0Rel().GetTargets()
        b1 = j.GetBody1Rel().GetTargets()
        if not b0 or not b1:
            fixed_joints_to_world.append(p.GetPath())
print(f"\n[2] Fiksni zglobovi prema svijetu: {len(fixed_joints_to_world)}")
for fp in fixed_joints_to_world:
    print(f"    !! {fp}  -- ovo drzi robota zalijepljenog, obrisi ga")

# ---------- 3. Zglobovi i pogoni ----------
print("\n[3] Rotacijski zglobovi:")
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
        flag = "  <<< PROBLEM: stiffness > 0 djeluje kao kocnica"
    if is_wheel and drive is None:
        flag = "  <<< PROBLEM: nema DriveAPI, zglob je slobodan"

    print(f"    {name:32s} axis={axis} stiff={stiff} damp={damp} "
          f"maxForce={maxf} type={dtype}{flag}")

print(f"\n    Prepoznato kotaca: {len(wheel_joints)}")
if len(wheel_joints) != 4:
    print("    !! Ocekivano 4. Provjeri WHEEL_KEYWORD ili imena zglobova.")

# ---------- 4. Mase ----------
print("\n[4] Kruta tijela i mase:")
total_mass = 0.0
for p in Usd.PrimRange(root):
    if not p.HasAPI(UsdPhysics.RigidBodyAPI):
        continue
    mass_api = UsdPhysics.MassAPI.Get(stage, p.GetPath())
    m = mass_api.GetMassAttr().Get() if mass_api else None
    if m:
        total_mass += m
    warn = "  <<< masa 0 ili nedefinirana" if (m is None or m == 0.0) else ""
    print(f"    {str(p.GetName()):32s} masa={m}{warn}")
print(f"    Ukupno: {total_mass:.2f} kg   (Husky bi trebao biti ~46-50 kg)")

# ---------- 5. Kolizije kotaca ----------
print("\n[5] Kolizije na kotacima:")
for p in Usd.PrimRange(root):
    if WHEEL_KEYWORD not in p.GetName().lower():
        continue
    if p.HasAPI(UsdPhysics.CollisionAPI):
        mat_api = UsdPhysics.MaterialAPI.Get(stage, p.GetPath())
        print(f"    {p.GetName():32s} ima CollisionAPI")

# ================================================================
#                            POPRAVAK
# ================================================================

if not APPLY_FIX:
    print("\n" + "=" * 70)
    print("APPLY_FIX = False -- nista nije promijenjeno.")
    print("Procitaj ispis, pa postavi APPLY_FIX = True i pokreni ponovno.")
    print("=" * 70)
else:
    print("\n" + "=" * 70)
    print("PRIMJENJUJEM POPRAVKE")
    print("=" * 70)

    # --- 5a. Velocity drive na kotacima ---
    for p in wheel_joints:
        drive = UsdPhysics.DriveAPI.Get(p, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(p, "angular")

        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(WHEEL_STIFFNESS)
        drive.CreateDampingAttr().Set(WHEEL_DAMPING)
        drive.CreateMaxForceAttr().Set(WHEEL_MAX_FORCE)
        drive.CreateTargetVelocityAttr().Set(0.0)

        # Bez ovoga PhysX zna ograniciti brzinu zgloba na nisku default vrijednost
        pj = PhysxSchema.PhysxJointAPI.Apply(p)
        pj.CreateMaxJointVelocityAttr().Set(1000.0)   # deg/s
        pj.CreateJointFrictionAttr().Set(0.0)

        print(f"    velocity drive -> {p.GetName()}")

    # --- 5b. Materijal trenja ---
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
    print(f"    materijal: {mat_path}")

    from pxr import UsdShade
    bound = 0
    for p in Usd.PrimRange(root):
        if WHEEL_KEYWORD not in p.GetName().lower():
            continue
        if not p.HasAPI(UsdPhysics.CollisionAPI):
            continue
        binding = UsdShade.MaterialBindingAPI.Apply(p)
        binding.Bind(
            UsdShade.Material(mat_prim),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics",
        )
        bound += 1
    print(f"    materijal vezan na {bound} kolizijskih prima")

    # --- 5c. Iskljuci self-collision na artikulaciji ---
    for p in art_roots:
        pa = PhysxSchema.PhysxArticulationAPI.Apply(p)
        pa.CreateEnabledSelfCollisionsAttr().Set(False)
        pa.CreateSolverPositionIterationCountAttr().Set(32)
        pa.CreateSolverVelocityIterationCountAttr().Set(4)
        print(f"    self-collision off, solver iteracije podignute -> {p.GetPath()}")

    print("\nGOTOVO. Pritisni STOP pa PLAY da fizika preuzme nove vrijednosti.")
    print("=" * 70)
