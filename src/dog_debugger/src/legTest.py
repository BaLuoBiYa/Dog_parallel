import math
import importlib
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

import legPy
from legPy import MotorPlacement


@dataclass
class LegGeometry:
    upper_length: float
    lower_length: float
    tip_length: float


def _circle_intersections(
    c0: np.ndarray, r0: float, c1: np.ndarray, r1: float
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    d = float(np.linalg.norm(c1 - c0))
    if d < 1e-8:
        return None
    if d > (r0 + r1) or d < abs(r0 - r1):
        return None

    a = (r0 * r0 - r1 * r1 + d * d) / (2.0 * d)
    h_sq = r0 * r0 - a * a
    if h_sq < -1e-8:
        return None
    h = math.sqrt(max(0.0, h_sq))

    p2 = c0 + a * (c1 - c0) / d
    offset = np.array([-(c1[1] - c0[1]), c1[0] - c0[0]], dtype=float) * (h / d)
    return p2 + offset, p2 - offset


def _pick_elbow(
    candidates: Optional[Tuple[np.ndarray, np.ndarray]],
    side: str,
    x_ref: float,
    target_z_hint: float,
) -> Optional[np.ndarray]:
    if candidates is None:
        return None
    p0, p1 = candidates

    if side == "left":
        left = [p for p in (p0, p1) if float(p[0]) <= x_ref]
        if len(left) == 1:
            return left[0]
        if len(left) == 2:
            return left[0] if abs(float(left[0][1] - target_z_hint)) <= abs(float(left[1][1] - target_z_hint)) else left[1]

    if side == "right":
        right = [p for p in (p0, p1) if float(p[0]) >= x_ref]
        if len(right) == 1:
            return right[0]
        if len(right) == 2:
            return right[0] if abs(float(right[0][1] - target_z_hint)) <= abs(float(right[1][1] - target_z_hint)) else right[1]

    if abs(float(p0[1] - target_z_hint)) <= abs(float(p1[1] - target_z_hint)):
        return p0
    return p1


def run_demo() -> None:
    """Interactive leg visualizer using either joint angles or tip position."""
    try:
        plt = importlib.import_module("matplotlib.pyplot")
        widgets = importlib.import_module("matplotlib.widgets")
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "matplotlib is required for visualization. Install it with: pip install matplotlib"
        ) from exc

    geom = LegGeometry(upper_length=120.0, lower_length=130.0, tip_length=20.0)
    leg = legPy.Leg2Dof(geom.upper_length, geom.lower_length, geom.tip_length)

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_axes([0.08, 0.26, 0.62, 0.68])
    ax_info = fig.add_axes([0.74, 0.26, 0.24, 0.68])
    ax_info.axis("off")

    TextBox = widgets.TextBox
    Button = widgets.Button

    box_front = TextBox(fig.add_axes([0.08, 0.16, 0.13, 0.05]), "Front(deg)", initial="0.0")
    box_rear = TextBox(fig.add_axes([0.23, 0.16, 0.13, 0.05]), "Rear(deg)", initial="0.0")
    box_x = TextBox(fig.add_axes([0.08, 0.08, 0.13, 0.05]), "Tip X(mm)", initial="0.0")
    box_z = TextBox(fig.add_axes([0.23, 0.08, 0.13, 0.05]), "Tip Y(mm)", initial="180.0")

    btn_by_joint = Button(fig.add_axes([0.40, 0.16, 0.15, 0.05]), "Apply Joint")
    btn_by_tip = Button(fig.add_axes([0.57, 0.16, 0.15, 0.05]), "Apply Tip")
    btn_reset = Button(fig.add_axes([0.40, 0.08, 0.32, 0.05]), "Reset")

    state = {
        "front": 0.0,
        "rear": 0.0,
        "status": "Ready.",
        "ok": True,
    }

    def _read_float(text: str, name: str) -> float:
        try:
            return float(text)
        except ValueError as exc:
            raise ValueError(f"{name} is not a valid number: {text}") from exc

    def _draw() -> None:
        tip = leg.getTipCurrent()
        tx = float(tip.pos[0])
        ty = float(tip.pos[2])

        # Coincident motor mounts and coincident chain end points.
        base_front = np.array([0.0, 0.0], dtype=float)
        base_rear = np.array([0.0, 0.0], dtype=float)
        tip_front = np.array([tx, ty], dtype=float)
        tip_rear = np.array([tx, ty], dtype=float)

        elbows_front = _circle_intersections(base_front, geom.upper_length, tip_front, geom.lower_length)
        elbows_rear = _circle_intersections(base_rear, geom.upper_length, tip_rear, geom.lower_length)
        # Front and rear passive joints must stay on opposite sides.
        elbow_front = _pick_elbow(elbows_front, side="left", x_ref=tx, target_z_hint=ty)
        elbow_rear = _pick_elbow(elbows_rear, side="right", x_ref=tx, target_z_hint=ty)

        ax.clear()
        ax.grid(True, alpha=0.25)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm, down)")
        ax.set_title("2-DOF Parallel Leg Visualizer")
        ax.invert_yaxis()

        ax.plot([base_front[0]], [base_front[1]], "ko", ms=8, label="Coincident motors")
        ax.plot([tx], [ty], "o", ms=8, color="#d97706", label="Coincident chain end")

        if elbow_front is not None:
            ax.plot([base_front[0], elbow_front[0]], [base_front[1], elbow_front[1]], color="#2563eb", lw=2.2)
            ax.plot([elbow_front[0], tip_front[0]], [elbow_front[1], tip_front[1]], color="#0ea5e9", lw=2.2)
        if elbow_rear is not None:
            ax.plot([base_rear[0], elbow_rear[0]], [base_rear[1], elbow_rear[1]], color="#2563eb", lw=2.2)
            ax.plot([elbow_rear[0], tip_rear[0]], [elbow_rear[1], tip_rear[1]], color="#0ea5e9", lw=2.2)

        ax.scatter(
            [base_front[0], tx],
            [base_front[1], ty],
            s=[60, 70],
            c=["black", "#dc2626"],
            zorder=5,
        )

        if elbow_front is not None:
            ax.scatter([elbow_front[0]], [elbow_front[1]], s=45, c="#1d4ed8", zorder=5)
        if elbow_rear is not None:
            ax.scatter([elbow_rear[0]], [elbow_rear[1]], s=45, c="#1d4ed8", zorder=5)

        max_reach = geom.upper_length + geom.lower_length + 20.0
        ax.set_xlim(-max_reach, max_reach)
        ax.set_ylim(-max_reach * 0.2, max_reach)

        if elbow_front is None or elbow_rear is None:
            state["status"] = "Geometry warning: no valid elbow intersection for current state."
            state["ok"] = False

        ax_info.clear()
        ax_info.axis("off")
        status_color = "#166534" if state["ok"] else "#b91c1c"
        info = (
            f"Front joint: {math.degrees(state['front']):.3f} deg\\n"
            f"Rear joint : {math.degrees(state['rear']):.3f} deg\\n"
            f"Tip X      : {tx:.3f} mm\\n"
            f"Tip Y      : {ty:.3f} mm\\n\\n"
            f"Status: {state['status']}"
        )
        ax_info.text(0.02, 0.95, info, va="top", ha="left", fontsize=11, color=status_color)

        fig.canvas.draw_idle()

    def _set_joint(front: float, rear: float) -> None:
        ok = leg.updateMotor([front, rear], [0.0, 0.0], [0.0, 0.0])
        state["front"] = front
        state["rear"] = rear
        state["ok"] = bool(ok)
        state["status"] = "Updated from joint angles." if ok else "updateMotor failed: pose unreachable."

    def _set_tip(x: float, z: float) -> None:
        ok = leg.setTip([x, 0.0, z], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        if not ok:
            state["ok"] = False
            state["status"] = "setTip failed: target unreachable."
            return

        front = leg.getMotorTarget(MotorPlacement.FRONT)
        rear = leg.getMotorTarget(MotorPlacement.REAR)
        ok_fk = leg.updateMotor([float(front.pos), float(rear.pos)], [0.0, 0.0], [0.0, 0.0])

        state["front"] = float(front.pos)
        state["rear"] = float(rear.pos)
        state["ok"] = bool(ok_fk)
        state["status"] = "Updated from tip target." if ok_fk else "FK update failed after IK target."

    def _on_apply_joint(_event) -> None:
        try:
            f_deg = _read_float(box_front.text, "front angle")
            r_deg = _read_float(box_rear.text, "rear angle")
            _set_joint(math.radians(f_deg), math.radians(r_deg))
        except ValueError as exc:
            state["ok"] = False
            state["status"] = str(exc)
        _draw()

    def _on_apply_tip(_event) -> None:
        try:
            x = _read_float(box_x.text, "tip x")
            z = _read_float(box_z.text, "tip z")
            _set_tip(x, z)
            box_front.set_val(f"{math.degrees(state['front']):.6f}")
            box_rear.set_val(f"{math.degrees(state['rear']):.6f}")
        except ValueError as exc:
            state["ok"] = False
            state["status"] = str(exc)
        _draw()

    def _on_reset(_event) -> None:
        box_front.set_val("0.0")
        box_rear.set_val("0.0")
        box_x.set_val("0.0")
        box_z.set_val("180.0")
        _set_tip(0.0, 180.0)
        box_front.set_val(f"{math.degrees(state['front']):.6f}")
        box_rear.set_val(f"{math.degrees(state['rear']):.6f}")
        _draw()

    btn_by_joint.on_clicked(_on_apply_joint)
    btn_by_tip.on_clicked(_on_apply_tip)
    btn_reset.on_clicked(_on_reset)

    _set_tip(0.0, 180.0)
    box_front.set_val(f"{math.degrees(state['front']):.6f}")
    box_rear.set_val(f"{math.degrees(state['rear']):.6f}")
    _draw()

    plt.show()


if __name__ == "__main__":
    run_demo()