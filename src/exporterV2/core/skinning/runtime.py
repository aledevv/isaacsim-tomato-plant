"""Runtime synchronization for persisted vegetative SkelRoots."""

from dataclasses import dataclass
from typing import List

from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt

from .schema import (
    ANIMATION_REL,
    BRANCH_ID_ATTR,
    PHYSICS_LINKS_REL,
    VISUAL_AXIS_ID_ATTR,
)


GLOBAL_PARENT_INDICES_ATTR = "autotom:skinning:parentIndices"


@dataclass
class _RuntimeBranch:
    name: str
    link_prims: list
    skel_root_prim: object
    translations_attr: object
    rotations_attr: object
    parent_indices: list | None = None


def _quatf_from_matrix(matrix: Gf.Matrix4d) -> Gf.Quatf:
    quat = matrix.ExtractRotationQuat()
    imag = quat.GetImaginary()
    return Gf.Quatf(float(quat.GetReal()), Gf.Vec3f(*imag))


class SkinningRuntime:
    """Synchronize UsdSkel animation attributes from PhysX rigid links."""

    def __init__(self, stage: Usd.Stage, branches: List[_RuntimeBranch]):
        self.stage = stage
        self.branches = branches
        self._cache = UsdGeom.XformCache(Usd.TimeCode.Default())

    @classmethod
    def discover(cls, stage: Usd.Stage) -> "SkinningRuntime":
        """Reconstruct and validate runtime state from authored relations."""
        branches = []
        for prim in stage.Traverse():
            if not prim.IsA(UsdSkel.Root):
                continue
            links_rel = prim.GetRelationship(PHYSICS_LINKS_REL)
            animation_rel = prim.GetRelationship(ANIMATION_REL)
            if not links_rel or not animation_rel:
                continue
            link_targets = links_rel.GetTargets()
            animation_targets = animation_rel.GetTargets()
            name_attr = prim.GetAttribute(VISUAL_AXIS_ID_ATTR)
            if not name_attr or not name_attr.HasAuthoredValue():
                name_attr = prim.GetAttribute(BRANCH_ID_ATTR)
            name = name_attr.Get() if name_attr else prim.GetPath().pathString
            if not link_targets:
                raise ValueError(f"SkelRoot '{name}' has no physics link targets")
            if len(animation_targets) != 1:
                raise ValueError(f"SkelRoot '{name}' must target exactly one SkelAnimation")

            link_prims = [stage.GetPrimAtPath(path) for path in link_targets]
            if any(not link.IsValid() for link in link_prims):
                raise ValueError(f"SkelRoot '{name}' references a missing physics link")
            animation = UsdSkel.Animation(stage.GetPrimAtPath(animation_targets[0]))
            if not animation or not animation.GetPrim().IsValid():
                raise ValueError(f"SkelRoot '{name}' references an invalid SkelAnimation")
            joints = animation.GetJointsAttr().Get() or []
            if len(joints) != len(link_prims):
                raise ValueError(
                    f"SkelRoot '{name}' has {len(joints)} bones but {len(link_prims)} links"
                )
            translations_attr = animation.GetTranslationsAttr()
            rotations_attr = animation.GetRotationsAttr()
            if not translations_attr or not rotations_attr:
                raise ValueError(f"SkelRoot '{name}' is missing animation transforms")

            skeletons = [
                UsdSkel.Skeleton(descendant)
                for descendant in Usd.PrimRange(prim)
                if descendant.IsA(UsdSkel.Skeleton)
            ]
            if len(skeletons) != 1:
                raise ValueError(f"SkelRoot '{name}' must contain exactly one Skeleton")

            parent_indices = None
            parent_attr = prim.GetAttribute(GLOBAL_PARENT_INDICES_ATTR)
            if parent_attr and parent_attr.HasAuthoredValue():
                parent_indices = list(parent_attr.Get() or [])
                if len(parent_indices) != len(link_prims):
                    raise ValueError(
                        f"SkelRoot '{name}' global parent count does not match its links"
                    )

            branches.append(_RuntimeBranch(
                name=name,
                link_prims=link_prims,
                skel_root_prim=prim,
                translations_attr=translations_attr,
                rotations_attr=rotations_attr,
                parent_indices=parent_indices,
            ))
        return cls(stage, branches)

    @property
    def branch_count(self) -> int:
        return len(self.branches)

    @property
    def bone_count(self) -> int:
        return sum(len(branch.link_prims) for branch in self.branches)

    def sync(self) -> None:
        """Write current rigid-link transforms into every SkelAnimation."""
        self._cache.Clear()
        for runtime in self.branches:
            world = [
                Gf.Matrix4d(self._cache.GetLocalToWorldTransform(prim))
                for prim in runtime.link_prims
            ]
            root_world = Gf.Matrix4d(
                self._cache.GetLocalToWorldTransform(runtime.skel_root_prim)
            )
            root_inverse = root_world.GetInverse()

            if runtime.parent_indices is None:
                local = [world[0] * root_inverse]
                local.extend(
                    world[index] * world[index - 1].GetInverse()
                    for index in range(1, len(world))
                )
            else:
                local = []
                for index, parent_index in enumerate(runtime.parent_indices):
                    if parent_index < 0:
                        local.append(world[index] * root_inverse)
                    else:
                        local.append(
                            world[index] * world[parent_index].GetInverse()
                        )

            translations = []
            rotations = []
            for matrix in local:
                translation = matrix.ExtractTranslation()
                translations.append(Gf.Vec3f(*translation))
                rotations.append(_quatf_from_matrix(matrix))
            runtime.translations_attr.Set(Vt.Vec3fArray(translations))
            runtime.rotations_attr.Set(Vt.QuatfArray(rotations))


def configure_physx_mouse_interaction(simulation_app) -> None:
    """Enable grabbing of invisible capsule proxies after stage open/reset."""
    import carb.settings
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("omni.physx.ui", True)
    manager.set_extension_enabled_immediate("omni.physx.supportui", True)
    simulation_app.update()
    settings = carb.settings.get_settings()
    settings.set("/physics/mouseInteractionEnabled", True)
    settings.set("/physics/mouseGrab", True)
    settings.set("/physics/mouseGrabIgnoreInvisible", False)
    settings.set("/physics/forceGrab", False)
    settings.set("/physics/pickingForce", 10.0)
