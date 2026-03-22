import argparse
import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


def main():
    parser = argparse.ArgumentParser(description="Animate trajectory points from CSV")
    parser.add_argument("--csv", required=True, help="Input CSV path")
    parser.add_argument(
        "--out",
        default=None,
        help="Output GIF path (default: test/trajectory/<csv_filename_without_ext>.gif)",
    )
    parser.add_argument("--fps", type=int, default=30, help="GIF FPS")
    parser.add_argument(
        "--step",
        type=int,
        default=1,
        help="Frame stride (e.g. 2 means every 2 points)",
    )
    args = parser.parse_args()

    if not os.path.exists(args.csv):
        raise FileNotFoundError(f"CSV not found: {args.csv}")

    if args.out is None:
        stem = os.path.splitext(os.path.basename(args.csv))[0]
        args.out = os.path.join("test", "trajectory", f"{stem}.gif")

    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    df = pd.read_csv(args.csv)

    required_cols = ["t_s", "pos_x", "pos_y", "pos_z", "vel_x", "vel_z"]
    for c in required_cols:
        if c not in df.columns:
            raise ValueError(f"Missing required column: {c}")

    if args.step > 1:
        df = df.iloc[::args.step].reset_index(drop=True)

    t = df["t_s"].to_numpy()
    x = df["pos_x"].to_numpy()
    z = df["pos_z"].to_numpy()
    vx = df["vel_x"].to_numpy()
    vz = df["vel_z"].to_numpy()

    n = len(df)
    if n < 2:
        raise ValueError("Not enough points to animate")

    fig, (ax_path, ax_time) = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle("Trajectory Animation")

    # 左图：X-Z 平面轨迹
    ax_path.set_title("Path in X-Z Plane")
    ax_path.set_xlabel("pos_x")
    ax_path.set_ylabel("pos_z")
    ax_path.grid(True, alpha=0.3)

    margin_x = (x.max() - x.min()) * 0.1 + 1e-6
    margin_z = (z.max() - z.min()) * 0.1 + 1e-6
    ax_path.set_xlim(x.min() - margin_x, x.max() + margin_x)
    ax_path.set_ylim(z.min() - margin_z, z.max() + margin_z)
    ax_path.invert_yaxis()

    line_path, = ax_path.plot([], [], lw=2, label="trail")
    point_path, = ax_path.plot([], [], "ro", ms=6, label="current")
    ax_path.legend(loc="best")

    # 右图：x/z 方向速度随时间变化
    ax_time.set_title("Velocity vs Time")
    ax_time.set_xlabel("time (s)")
    ax_time.set_ylabel("velocity")
    ax_time.grid(True, alpha=0.3)

    ymin = min(vx.min(), vz.min())
    ymax = max(vx.max(), vz.max())
    margin_v = (ymax - ymin) * 0.1 + 1e-6

    ax_time.set_xlim(t.min(), t.max())
    ax_time.set_ylim(ymin - margin_v, ymax + margin_v)

    line_vx, = ax_time.plot([], [], label="vel_x")
    line_vz, = ax_time.plot([], [], label="vel_z")
    cursor = ax_time.axvline(x=t[0], color="k", linestyle="--", alpha=0.6)
    ax_time.legend(loc="best")

    def init():
        line_path.set_data([], [])
        point_path.set_data([], [])
        line_vx.set_data([], [])
        line_vz.set_data([], [])
        cursor.set_xdata([t[0], t[0]])
        return line_path, point_path, line_vx, line_vz, cursor

    def update(i):
        line_path.set_data(x[: i + 1], z[: i + 1])
        point_path.set_data([x[i]], [z[i]])

        line_vx.set_data(t[: i + 1], vx[: i + 1])
        line_vz.set_data(t[: i + 1], vz[: i + 1])
        cursor.set_xdata([t[i], t[i]])

        ax_path.set_title(f"Path in X-Z Plane (t={t[i]:.3f}s)")
        return line_path, point_path, line_vx, line_vz, cursor

    ani = FuncAnimation(
        fig,
        update,
        frames=n,
        init_func=init,
        interval=1000.0 / max(args.fps, 1),
        blit=True,
    )

    writer = PillowWriter(fps=args.fps)
    ani.save(args.out, writer=writer)
    print(f"Saved animation: {args.out}")


if __name__ == "__main__":
    main()