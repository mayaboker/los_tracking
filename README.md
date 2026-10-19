# LOS tracking

This project is a member of the [Bullet workspace](../README.md). All six
projects share `../.venv` and `../uv.lock`; run uv commands with the
workspace flags below to retain every project and optional dependency.

Python 3.12+ uv library for detecting the largest red object in a BGR image.
Camera reception and the live example moved to the sibling
[`bullet_app`](../bullet_app/README.md), which also publishes detection results
for MCAP. This library does not calculate LOS angles or maintain temporal tracks.

```bash
cd ~/projects/bullet/los_tracking
uv sync --locked --all-packages --all-extras --all-groups
uv run --locked --all-packages --all-extras --all-groups pytest -q
```

```python
from los_tracking import detect_red_box

bbox = detect_red_box(bgr)  # (x, y, width, height), or None.
```

Input must be a nonempty H×W×3 uint8 BGR NumPy array. The detector leaves it
unchanged and rejects invalid inputs with ValueError. Coordinates are pixels
from the top-left corner, x right and y down. The detector follows `fly_smart`:
HSV hue 0–10 or 170–180, saturation >=100, value >=80, 3×3 morphological opening,
largest contour with area >=4 pixels² and dimensions >=3×3 pixels.
HSV detection depends on lighting and may select unrelated red objects.

Run the live camera pipeline from the application project:

```bash
cd ../bullet_app
uv sync --locked --all-packages --all-extras --all-groups
uv run --locked --all-packages --all-extras --all-groups python examples/live_los.py \
  --transport-config ../bullet_render/configs/transport.json --debug
```

The Bullet VS Code workspace provides **Bullet: App (debug)**, also launched by
**Bullet: Physics + render + app** and **Bullet: Run all**. `ImageReceiver`
now lives in `bullet_app.receiver`; it is no longer exported from this library.
