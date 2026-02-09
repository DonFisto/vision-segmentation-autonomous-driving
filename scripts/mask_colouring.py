import numpy as np
from PIL import Image
from pathlib import Path

inp = Path("out/untrained_manual")
out = Path("out/untrained_manual_color")
out.mkdir(parents=True, exist_ok=True)

# Simple palette: class 0 -> black, class 1 -> green
palette = np.zeros((256, 3), dtype=np.uint8)
palette[0] = [0, 0, 0]
palette[1] = [0, 255, 0]

for f in sorted(inp.glob("pred_*.png"))[:50]:
    m = np.array(Image.open(f))
    rgb = palette[m]
    Image.fromarray(rgb).save(out / f.name.replace("pred_", "pred_color_"))

print("Saved color masks to", out)