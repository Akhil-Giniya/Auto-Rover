"""
mapping/drum_map.py — production drum position mapper with error handling.
"""

import time
import numpy as np
from pathlib import Path
from rov_logger import get_logger

log = get_logger("mapping")

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    MPL = True
except ImportError:
    MPL = False
    log.warning("matplotlib not available — map saved as text only")


class DrumMap:
    def __init__(self):
        self.red: list[tuple[int, int]]  = []
        self.blue: list[tuple[int, int]] = []
        self.ts = time.strftime("%Y%m%d_%H%M%S")

    def record_red(self, cx: int, cy: int) -> None:
        self.red.append((cx, cy))

    def record_blue(self, cx: int, cy: int) -> None:
        self.blue.append((cx, cy))

    def get_positions(self) -> dict:
        out = {}
        if self.red:
            clusters = _cluster(np.array(self.red), 50)
            for i, c in enumerate(clusters[:3]):
                out[f"red{i+1}"] = (int(c[0]), int(c[1]))
        if self.blue:
            m = np.array(self.blue).mean(axis=0)
            out["blue"] = (int(m[0]), int(m[1]))
        return out

    def finalize(self, output_path: str = "logs/drum_map.png") -> str:
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        pos = self.get_positions()
        lines = ["=== DRUM MAP ===", f"Generated: {self.ts}"]
        for name, (x, y) in pos.items():
            lines.append(f"  {name}: pixel ({x}, {y})")
        summary = "\n".join(lines)
        log.info(summary)

        if MPL and pos:
            try:
                _plot(pos, output_path)
                log.info(f"Drum map saved: {output_path}")
            except Exception as e:
                log.error(f"Drum map plot failed: {e}", exc_info=True)
                _save_text(summary, output_path.replace(".png", ".txt"))
        else:
            _save_text(summary, output_path.replace(".png", ".txt"))
        return summary


def _plot(positions: dict, path: str) -> None:
    fig, ax = plt.subplots(figsize=(6, 5))
    ax.set_xlim(0, 640); ax.set_ylim(0, 480); ax.invert_yaxis()
    ax.set_facecolor("#0a1a2e"); fig.patch.set_facecolor("#0a1a2e")
    ax.set_title("2D Drum Map", color="white")
    ax.tick_params(colors="white")
    for name, (x, y) in positions.items():
        c = "#2266ff" if name == "blue" else "#ff3333"
        ax.scatter(x, y, s=300, c=c, edgecolors="white", linewidths=1.5, zorder=5)
        ax.annotate(name, (x, y), xytext=(8, 8), textcoords="offset points",
                    color="white", fontsize=9)
    ax.set_xlabel("Frame X (px)", color="white")
    ax.set_ylabel("Frame Y (px)", color="white")
    plt.tight_layout()
    plt.savefig(path, dpi=120, bbox_inches="tight")
    plt.close()


def _save_text(text: str, path: str) -> None:
    try:
        with open(path, "w") as f:
            f.write(text)
    except Exception as e:
        log.error(f"Cannot save text map: {e}")


def _cluster(pts: np.ndarray, threshold: int) -> list:
    if len(pts) == 0:
        return []
    used = [False] * len(pts)
    clusters = []
    for i, p in enumerate(pts):
        if used[i]:
            continue
        grp = [p]; used[i] = True
        for j, q in enumerate(pts):
            if not used[j] and np.linalg.norm(p - q) < threshold:
                grp.append(q); used[j] = True
        clusters.append(np.array(grp).mean(axis=0))
    return clusters
