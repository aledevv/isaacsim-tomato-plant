"""Shared USD materials for ExporterV2 scenes."""

from pxr import Gf, Sdf, UsdGeom, UsdShade


TOMATO_LEAF_MATERIAL_PATH = "/World/Looks/TomatoLeaf"
TOMATO_LEAF_OMNISURFACE_MATERIAL_PATH = "/World/Looks/TomatoLeafOmniSurface"
TOMATO_STEM_MATERIAL_PATH = "/World/Looks/TomatoStem"
TOMATO_STEM_OMNISURFACE_MATERIAL_PATH = "/World/Looks/TomatoStemOmniSurface"
TOMATO_TRUSS_MATERIAL_PATH = "/World/Looks/TomatoTrussRachis"
TOMATO_FRUIT_MATERIAL_ROOT = "/World/Looks/TomatoFruit"
TOMATO_FRUIT_MATURATION_BUCKETS = 8


_TOMATO_LEAF_PRESETS = {
    "realtime": {
        "diffuseColor": Gf.Vec3f(0.10, 0.30, 0.075),
        "roughness": 0.62,
        "metallic": 0.0,
    },
    "realistic": {
        "diffuse_reflection_weight": 0.85,
        "diffuse_reflection_color": Gf.Vec3f(0.10, 0.30, 0.055),
        "diffuse_reflection_roughness": 0.12,
        "metalness": 0.0,
        "specular_reflection_weight": 1.0,
        "specular_reflection_roughness": 0.50,
        "specular_reflection_ior": 1.42,
        "thin_walled": True,
        "enable_diffuse_transmission": True,
        "subsurface_weight": 0.30,
        "subsurface_transmission_color": Gf.Vec3f(0.30, 0.55, 0.08),
        "specular_retro_reflection_weight": 0.07,
        "specular_retro_reflection_color": Gf.Vec3f(0.35, 0.50, 0.22),
        "specular_retro_reflection_roughness": 0.50,
    },
}


_TOMATO_STEM_PRESETS = {
    "realtime": {
        "diffuseColor": Gf.Vec3f(0.22, 0.40, 0.18),
        "roughness": 0.68,
        "metallic": 0.0,
    },
    "realistic": {
        "diffuse_reflection_weight": 0.90,
        "diffuse_reflection_color": Gf.Vec3f(0.22, 0.40, 0.18),
        "diffuse_reflection_roughness": 0.20,
        "metalness": 0.0,
        "specular_reflection_weight": 0.45,
        "specular_reflection_roughness": 0.68,
        "specular_reflection_ior": 1.42,
        "thin_walled": False,
        "enable_diffuse_transmission": True,
        "subsurface_weight": 0.05,
        "specular_retro_reflection_weight": 0.12,
        "specular_retro_reflection_color": Gf.Vec3f(0.40, 0.55, 0.30),
        "specular_retro_reflection_roughness": 0.65,
    },
}


_TOMATO_FRUIT_PRESETS = {
    "realtime": {
        "roughness": 0.28,
        "metallic": 0.0,
        "clearcoat": 0.28,
        "clearcoatRoughness": 0.18,
    },
}


def _leaf_material_path_for_preset(preset: str) -> str:
    if preset == "realistic":
        return TOMATO_LEAF_OMNISURFACE_MATERIAL_PATH
    return TOMATO_LEAF_MATERIAL_PATH


def _stem_material_path_for_preset(preset: str) -> str:
    if preset == "realistic":
        return TOMATO_STEM_OMNISURFACE_MATERIAL_PATH
    return TOMATO_STEM_MATERIAL_PATH


def _set_shader_input(shader, name: str, value) -> None:
    if isinstance(value, bool):
        value_type = Sdf.ValueTypeNames.Bool
    elif isinstance(value, Gf.Vec3f):
        value_type = Sdf.ValueTypeNames.Color3f
    else:
        value_type = Sdf.ValueTypeNames.Float

    shader.CreateInput(name, value_type).Set(value)


def _tomato_color(maturation: float) -> Gf.Vec3f:
    t = max(0.0, min(1.0, float(maturation)))
    unripe = Gf.Vec3f(0.25, 0.65, 0.08)
    ripe = Gf.Vec3f(0.90, 0.17, 0.08)
    return unripe * (1.0 - t) + ripe * t


def _tomato_maturation_bucket(maturation: float) -> int:
    t = max(0.0, min(1.0, float(maturation)))
    return int(round(t * (TOMATO_FRUIT_MATURATION_BUCKETS - 1)))


def _tomato_bucket_maturation(bucket: int) -> float:
    bucket = max(0, min(TOMATO_FRUIT_MATURATION_BUCKETS - 1, int(bucket)))
    return bucket / float(TOMATO_FRUIT_MATURATION_BUCKETS - 1)


def _create_preview_surface_material(stage, material_path: str, inputs):
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")

    for name, value in inputs.items():
        _set_shader_input(shader, name, value)

    shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )
    return material


def _create_omnisurface_material(stage, material_path: str, inputs):
    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    shader.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    shader.SetSourceAsset("OmniSurface.mdl", "mdl")
    shader.SetSourceAssetSubIdentifier("OmniSurface", "mdl")
    shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput("mdl").ConnectToSource(
        shader.ConnectableAPI(), "out"
    )

    for name, value in inputs.items():
        _set_shader_input(shader, name, value)
    return material


def _create_realtime_tomato_leaf_material(stage, material_path: str):
    return _create_preview_surface_material(
        stage, material_path, _TOMATO_LEAF_PRESETS["realtime"]
    )


def _create_realtime_tomato_stem_material(stage, material_path: str):
    return _create_preview_surface_material(
        stage, material_path, _TOMATO_STEM_PRESETS["realtime"]
    )


def _create_realtime_tomato_fruit_material(
    stage,
    material_path: str,
    maturation: float,
):
    inputs = {
        "diffuseColor": _tomato_color(maturation),
        **_TOMATO_FRUIT_PRESETS["realtime"],
    }
    return _create_preview_surface_material(
        stage, material_path, inputs
    )


def _create_realistic_tomato_leaf_material(stage, material_path: str):
    return _create_omnisurface_material(
        stage, material_path, _TOMATO_LEAF_PRESETS["realistic"]
    )


def _create_realistic_tomato_stem_material(stage, material_path: str):
    return _create_omnisurface_material(
        stage, material_path, _TOMATO_STEM_PRESETS["realistic"]
    )


def get_or_create_tomato_leaf_material(stage, preset: str = "realtime"):
    """Return the shared tomato leaf material for a stage.

    The default uses UsdPreviewSurface to keep full-plant scenes responsive.
    The heavier OmniSurface preset remains available for isolated look-dev.
    """
    if preset not in _TOMATO_LEAF_PRESETS:
        raise ValueError(f"Unknown tomato leaf material preset: {preset!r}")

    material_path = _leaf_material_path_for_preset(preset)
    existing = stage.GetPrimAtPath(material_path)
    if existing.IsValid():
        return UsdShade.Material(existing)

    UsdGeom.Scope.Define(stage, "/World/Looks")

    if preset == "realistic":
        return _create_realistic_tomato_leaf_material(stage, material_path)
    return _create_realtime_tomato_leaf_material(stage, material_path)


def get_or_create_tomato_stem_material(stage, preset: str = "realtime"):
    """Return the shared tomato stem material for vegetative visual meshes."""
    if preset not in _TOMATO_STEM_PRESETS:
        raise ValueError(f"Unknown tomato stem material preset: {preset!r}")

    material_path = _stem_material_path_for_preset(preset)
    existing = stage.GetPrimAtPath(material_path)
    if existing.IsValid():
        return UsdShade.Material(existing)

    UsdGeom.Scope.Define(stage, "/World/Looks")

    if preset == "realistic":
        return _create_realistic_tomato_stem_material(stage, material_path)
    return _create_realtime_tomato_stem_material(stage, material_path)


def get_or_create_tomato_truss_material(stage):
    """Return the historical muted-green rachis material."""

    existing = stage.GetPrimAtPath(TOMATO_TRUSS_MATERIAL_PATH)
    if existing.IsValid():
        return UsdShade.Material(existing)
    UsdGeom.Scope.Define(stage, "/World/Looks")
    return _create_preview_surface_material(
        stage,
        TOMATO_TRUSS_MATERIAL_PATH,
        {
            "diffuseColor": Gf.Vec3f(0.263, 0.424, 0.227),
            "roughness": 0.68,
            "metallic": 0.0,
        },
    )


def get_or_create_tomato_fruit_material(
    stage,
    maturation: float,
    preset: str = "realtime",
):
    """Return a shared tomato fruit material bucketed by maturation."""
    if preset not in _TOMATO_FRUIT_PRESETS:
        raise ValueError(f"Unknown tomato fruit material preset: {preset!r}")

    bucket = _tomato_maturation_bucket(maturation)
    material_path = f"{TOMATO_FRUIT_MATERIAL_ROOT}/Maturation_{bucket}"
    existing = stage.GetPrimAtPath(material_path)
    if existing.IsValid():
        return UsdShade.Material(existing)

    UsdGeom.Scope.Define(stage, "/World/Looks")
    UsdGeom.Scope.Define(stage, TOMATO_FRUIT_MATERIAL_ROOT)

    return _create_realtime_tomato_fruit_material(
        stage,
        material_path,
        _tomato_bucket_maturation(bucket),
    )
