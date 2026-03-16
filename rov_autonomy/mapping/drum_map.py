"""
mapping/drum_map.py
Records drum positions as they are detected and generates a 2D map image.
Coordinates are stored in camera pixel space and annotated with labels.
"""

import time
import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")   # no display needed on RPi
    import matplotlib.pyplot as plt
    MPL_AVAILABLE = True
except ImportError:
    MPL_AVAILABLE = False
    print("[DrumMap] matplotlib not available — map will be saved as text only")


class DrumMap:
    """
    Accumulates drum detections and writes a 2D map.

    Usage:
        dmap = DrumMap()
        dmap.record_red(cx, cy)   # call multiple times as drums are seen
        dmap.record_blue(cx, cy)
        dmap.finalize("drum_map.png")
    """

    def __init__(self):
        self.red_detections: list[tuple[int, int]] = []
        self.blue_detections: list[tuple[int, int]] = []
        self.timestamp = time.strftime("%Y%m%d_%H%M%S")

    def record_red(self, cx: int, cy: int) -> None:
        self.red_detections.append((cx, cy))

    def record_blue(self, cx: int, cy: int) -> None:
        self.blue_detections.append((cx, cy))

    def get_positions(self) -> dict:
        """Return best-estimate positions by averaging detections."""
        positions = {}
        if self.red_detections:
            arr = np.array(self.red_detections)
            # Cluster into up to 3 drums using mean per detected cluster
            # Simple approach: just keep unique centroids (within 50px)
            clusters = self._cluster(arr, threshold=50)
            for i, c in enumerate(clusters[:3]):
                positions[f"red{i+1}"] = tuple(int(v) for v in c)

        if self.blue_detections:
            arr = np.array(self.blue_detections)
            mean = arr.mean(axis=0)
            positions["blue"] = tuple(int(v) for v in mean)

        return positions

    def finalize(self, output_path: str = "drum_map.png") -> str:
        """Generate and save the 2D map. Returns summary string."""
        positions = self.get_positions()
        summary_lines = ["=== DRUM MAP ==="]
        for name, (x, y) in positions.items():
            summary_lines.append(f"  {name}: pixel ({x}, {y})")
        summary = "\n".join(summary_lines)
        print(summary)

        if MPL_AVAILABLE and positions:
            self._plot(positions, output_path)
            print(f"[DrumMap] saved to {output_path}")
        else:
            txt_path = output_path.replace(".png", ".txt")
            with open(txt_path, "w") as f:
                f.write(summary)

        return summary

    def _plot(self, positions: dict, path: str) -> None:
        fig, ax = plt.subplots(figsize=(6, 5))
        ax.set_xlim(0, 640)
        ax.set_ylim(0, 480)
        ax.invert_yaxis()
        ax.set_facecolor("#0a1a2e")
        fig.patch.set_facecolor("#0a1a2e")
        ax.set_title("2D Drum Position Map", color="white")
        ax.tick_params(colors="white")

        for name, (x, y) in positions.items():
            color = "#2266ff" if name == "blue" else "#ff3333"
            ax.scatter(x, y, s=300, c=color, zorder=5,
                       edgecolors="white", linewidths=1.5)
            ax.annotate(name, (x, y), textcoords="offset points",
                        xytext=(8, 8), color="white", fontsize=9)

        ax.set_xlabel("Frame X (px)", color="white")
        ax.set_ylabel("Frame Y (px)", color="white")
        plt.tight_layout()
        plt.savefig(path, dpi=120, bbox_inches="tight")
        plt.close()

    @staticmethod
    def _cluster(points: np.ndarray, threshold: int) -> list:
        """Naive clustering: group points within threshold px."""
        if len(points) == 0:
            return []
        clusters = []
        used = [False] * len(points)
        for i, p in enumerate(points):
            if used[i]:
                continue
            group = [p]
            used[i] = True
            for j, q in enumerate(points):
                if not used[j] and np.linalg.norm(p - q) < threshold:
                    group.append(q)
                    used[j] = True
            clusters.append(np.array(group).mean(axis=0))
        return clusters
