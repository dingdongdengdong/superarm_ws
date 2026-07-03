"""Import upstream AmazingHand MJCF into a USD asset for Isaac Sim validation."""
from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from isaacsim import SimulationApp


CONTAINER_ROOT = "/workspace/superarm_ws"
DEFAULT_MJCF = f"{CONTAINER_ROOT}/AmazingHand/Demo/AHSimulation/AHSimulation/AH_Right/mjcf/robot.xml"
DEFAULT_USD = (
    f"{CONTAINER_ROOT}/isaacsim_test/outputs/simready/echo_full/sitl/"
    "amazinghand_right_from_mjcf.usd"
)
DEFAULT_SANITIZED_MJCF = (
    f"{CONTAINER_ROOT}/isaacsim_test/outputs/simready/echo_full/sitl/"
    "amazinghand_right_isaac_sanitized.xml"
)


def _set_if_available(config, method_name: str, value) -> None:
    method = getattr(config, method_name, None)
    if callable(method):
        method(value)


def sanitize_mjcf_for_isaac_import(source_mjcf: str, output_mjcf: str) -> dict[str, object]:
    """Write an Isaac MJCF importer compatible copy while preserving source meshes."""
    source = Path(source_mjcf)
    output = Path(output_mjcf)
    tree = ET.parse(source)
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.attrib["meshdir"] = str((source.parent / "assets").resolve())

    defaults = root.findall("default")
    top_level_defaults_before = len(defaults)
    if len(defaults) > 1:
        first_index = list(root).index(defaults[0])
        merged = ET.Element("default")
        for default in defaults:
            for child in list(default):
                default.remove(child)
                merged.append(child)
            root.remove(default)
        root.insert(first_index, merged)

    mesh_names_added = 0
    for mesh in root.findall("./asset/mesh"):
        if not mesh.attrib.get("name") and mesh.attrib.get("file"):
            mesh.attrib["name"] = Path(mesh.attrib["file"]).stem
            mesh_names_added += 1

    equality_connect_names_added = 0
    for index, connect in enumerate(root.findall("./equality/connect"), start=1):
        if not connect.attrib.get("name"):
            connect.attrib["name"] = f"closing_connect_{index:02d}"
            equality_connect_names_added += 1

    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    output.write_text(output.read_text(encoding="utf-8") + "\n", encoding="utf-8")
    sanitized_root = ET.parse(output).getroot()
    return {
        "input_mjcf": str(source),
        "output_mjcf": str(output),
        "meshdir": compiler.attrib["meshdir"],
        "top_level_defaults_before": top_level_defaults_before,
        "top_level_defaults_after": len(sanitized_root.findall("default")),
        "mesh_names_added": mesh_names_added,
        "equality_connect_names_added": equality_connect_names_added,
        "equality_connect_count": len(sanitized_root.findall("./equality/connect")),
        "status": "PASS" if len(sanitized_root.findall("default")) == 1 else "FAIL",
    }


parser = argparse.ArgumentParser()
parser.add_argument(
    "--mjcf",
    default=os.environ.get("AMAZINGHAND_MJCF_PATH", DEFAULT_MJCF),
)
parser.add_argument(
    "--usd",
    default=os.environ.get("AMAZINGHAND_USD_PATH", DEFAULT_USD),
)
parser.add_argument(
    "--sanitized-mjcf",
    default=os.environ.get("AMAZINGHAND_SANITIZED_MJCF_PATH", DEFAULT_SANITIZED_MJCF),
)
parser.add_argument("--no-sanitize", action="store_true")
parser.add_argument("--prim-path", default=os.environ.get("AMAZINGHAND_USD_PRIM_PATH", "/AmazingHandRight"))
args, _ = parser.parse_known_args()

import_mjcf_path = args.mjcf
sanitize_report = None
if not args.no_sanitize:
    sanitize_report = sanitize_mjcf_for_isaac_import(args.mjcf, args.sanitized_mjcf)
    if sanitize_report["status"] != "PASS":
        print(
            "[import_amazinghand_mjcf_to_usd] ERROR: MJCF sanitization failed: "
            f"{sanitize_report}",
            flush=True,
        )
        sys.exit(1)
    import_mjcf_path = args.sanitized_mjcf
    print(
        "[import_amazinghand_mjcf_to_usd] Sanitized AmazingHand MJCF for Isaac import: "
        f"{sanitize_report}",
        flush=True,
    )

simulation_app = SimulationApp({"headless": True})

import omni.kit.commands  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("isaacsim.asset.importer.mjcf")
simulation_app.update()

if not os.path.isfile(import_mjcf_path):
    print(f"[import_amazinghand_mjcf_to_usd] ERROR: MJCF not found: {import_mjcf_path}", flush=True)
    simulation_app.close()
    sys.exit(1)

os.makedirs(os.path.dirname(args.usd), exist_ok=True)
status, import_config = omni.kit.commands.execute("MJCFCreateImportConfig")
if not status:
    print("[import_amazinghand_mjcf_to_usd] ERROR: MJCFCreateImportConfig failed", flush=True)
    simulation_app.close()
    sys.exit(1)

_set_if_available(import_config, "set_fix_base", True)
_set_if_available(import_config, "set_import_inertia_tensor", True)
_set_if_available(import_config, "set_make_default_prim", True)

status, imported_prim_path = omni.kit.commands.execute(
    "MJCFCreateAsset",
    mjcf_path=import_mjcf_path,
    import_config=import_config,
    prim_path=args.prim_path,
    dest_path=args.usd,
)
if not status:
    print(
        f"[import_amazinghand_mjcf_to_usd] ERROR: MJCFCreateAsset failed for {args.mjcf}",
        flush=True,
    )
    simulation_app.close()
    sys.exit(1)

print(
    "[import_amazinghand_mjcf_to_usd] Imported AmazingHand MJCF: "
    f"{import_mjcf_path} -> {args.usd} at {imported_prim_path or args.prim_path}",
    flush=True,
)
simulation_app.close()
