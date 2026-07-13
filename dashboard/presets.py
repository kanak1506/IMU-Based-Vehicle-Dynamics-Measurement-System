import json
import re
from pathlib import Path

PRESETS_DIR = Path(__file__).parent.parent / "presets"

# Characters that are invalid in filenames on Windows, macOS, and Linux.
_INVALID_CHARS = re.compile(r'[\\/:*?"<>|]')


def list_presets() -> list[str]:
    """Return sorted list of saved preset names (without .json extension)."""
    PRESETS_DIR.mkdir(exist_ok=True)
    return sorted(p.stem for p in PRESETS_DIR.glob("*.json"))


def save_preset(vehicle: dict) -> None:
    """Write vehicle dict to presets/<name>.json."""
    PRESETS_DIR.mkdir(exist_ok=True)
    name      = str(vehicle.get("name", "")).strip()
    safe_name = _INVALID_CHARS.sub("", name).strip()
    if not safe_name:
        raise ValueError(
            f"Vehicle name '{name}' cannot be used as a filename. "
            "Avoid characters: \\ / : * ? \" < > |"
        )
    path = PRESETS_DIR / f"{safe_name}.json"
    path.write_text(json.dumps(vehicle, indent=2), encoding="utf-8")


def load_preset(name: str) -> dict:
    """Read and return vehicle dict from presets/<name>.json."""
    path = PRESETS_DIR / f"{name}.json"
    if not path.exists():
        raise FileNotFoundError(f"Preset '{name}' not found.")
    return json.loads(path.read_text(encoding="utf-8"))
