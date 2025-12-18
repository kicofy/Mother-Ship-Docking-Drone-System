import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path
from typing import List, Tuple, Optional

from pymavlink import mavutil
import matplotlib.pyplot as plt
from serial.tools import list_ports


def deg1e7_to_deg(value_1e7: int) -> float:
    return float(value_1e7) / 1e7


def latlon_to_meters(lat_deg: float, lon_deg: float, lat0_deg: float, lon0_deg: float) -> Tuple[float, float]:
    lat_rad = math.radians(lat_deg)
    lat0_rad = math.radians(lat0_deg)
    dlat = lat_rad - lat0_rad
    dlon = math.radians(lon_deg - lon0_deg)
    x = dlon * math.cos(lat0_rad) * 6378137.0
    y = dlat * 6378137.0
    return x, y


def detect_px4_serial(preferred_bauds: Tuple[int, ...], heartbeat_timeout_s: float) -> Optional[Tuple[str, int]]:
    candidates = list(list_ports.comports())
    for port in candidates:
        for baud in preferred_bauds:
            try:
                conn = mavutil.mavlink_connection(device=port.device, baud=baud, autoreconnect=False)
                hb = conn.wait_heartbeat(timeout=heartbeat_timeout_s)
                conn.close()
                if hb:
                    return port.device, baud
            except Exception:
                continue
    return None


def wait_heartbeat(conn: mavutil.mavfile, timeout_s: float) -> bool:
    try:
        hb = conn.wait_heartbeat(timeout=timeout_s)
        return hb is not None
    except Exception:
        return False


def collect_gps_samples(
    conn: mavutil.mavfile,
    duration_s: float,
) -> List[Tuple[float, float, float, float, Optional[int], Optional[float], Optional[float]]]:
    """
    Collect GPS samples for duration.
    Returns list of (t, lat_deg, lon_deg, alt_m, num_sats, hdop_or_eph, rel_alt_m).
    """
    samples: List[Tuple[float, float, float, float, Optional[int], Optional[float], Optional[float]]] = []
    end_ts = time.time() + duration_s

    while time.time() < end_ts:
        msg = conn.recv_match(type=["GLOBAL_POSITION_INT", "GPS_RAW_INT"], blocking=True, timeout=1.0)
        now = time.time()
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == "GLOBAL_POSITION_INT":
            lat = deg1e7_to_deg(getattr(msg, "lat", 0))
            lon = deg1e7_to_deg(getattr(msg, "lon", 0))
            alt_m = float(getattr(msg, "alt", 0)) / 1000.0  # mm -> m
            rel_alt_m = float(getattr(msg, "relative_alt", 0)) / 1000.0  # mm -> m
            num_sats = None
            hdop = None
        else:  # GPS_RAW_INT
            lat = deg1e7_to_deg(getattr(msg, "lat", 0))
            lon = deg1e7_to_deg(getattr(msg, "lon", 0))
            alt_m = float(getattr(msg, "alt", 0)) / 1000.0  # mm -> m
            rel_alt_m = None
            num_sats = getattr(msg, "satellites_visible", None)
            eph = getattr(msg, "eph", None)
            hdop = None
            if eph is not None:
                hdop = float(eph) / 100.0 if eph > 100 else float(eph)
        samples.append((now, lat, lon, alt_m, num_sats, hdop, rel_alt_m))

    return samples


def save_csv(path: Path, samples: List[Tuple[float, float, float, float, Optional[int], Optional[float], Optional[float]]]):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_s", "lat_deg", "lon_deg", "alt_m", "num_sats", "hdop_or_eph", "ekf_relative_alt_m"])
        for row in samples:
            writer.writerow(row)


def compute_drift_metrics(samples: List[Tuple[float, float, float, float, Optional[int], Optional[float], Optional[float]]]) -> Tuple[float, float, float]:
    if not samples:
        return 0.0, 0.0, 0.0
    _, lat0, lon0, _, _, _, _ = samples[0]
    xs: List[float] = []
    ys: List[float] = []
    for _, lat, lon, *_ in samples:
        x, y = latlon_to_meters(lat, lon, lat0, lon0)
        xs.append(x)
        ys.append(y)
    radii = [math.hypot(x, y) for x, y in zip(xs, ys)]
    max_radius = max(radii) if radii else 0.0
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    rms_radius = math.sqrt(sum((x - mean_x) ** 2 + (y - mean_y) ** 2 for x, y in zip(xs, ys)) / len(xs))
    return max_radius, rms_radius, (sum(radii) / len(radii)) if radii else 0.0


def plot_drift_xy(samples: List[Tuple[float, float, float, float, Optional[int], Optional[float], Optional[float]]], out_path: Path):
    if not samples:
        return
    _, lat0, lon0, *_ = samples[0]
    xs: List[float] = []
    ys: List[float] = []
    for _, lat, lon, *_ in samples:
        x, y = latlon_to_meters(lat, lon, lat0, lon0)
        xs.append(x)
        ys.append(y)

    max_radius, rms_radius, _ = compute_drift_metrics(samples)

    fig, ax = plt.subplots(figsize=(6, 6), dpi=120)
    ax.scatter(xs, ys, s=8, c="#1f77b4", alpha=0.8, label="Trajectory")
    ax.scatter([0.0], [0.0], c="red", s=30, label="Start")

    circle_max = plt.Circle((0, 0), max_radius, color="orange", fill=False, linestyle="--", label=f"Max radius {max_radius:.2f} m")
    circle_rms = plt.Circle((0, 0), rms_radius, color="green", fill=False, linestyle=":", label=f"RMS radius {rms_radius:.2f} m")
    ax.add_artist(circle_max)
    ax.add_artist(circle_rms)

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.set_xlabel("East offset x (m)")
    ax.set_ylabel("North offset y (m)")
    ax.set_title("GPS Drift (relative to start)")
    ax.legend(loc="best")

    extent = max(
        max_radius,
        max((abs(v) for v in xs), default=0.0),
        max((abs(v) for v in ys), default=0.0),
    )
    pad = max(0.5, 0.05 * max(1.0, extent))
    R = extent + pad
    ax.set_xlim(-R, R)
    ax.set_ylim(-R, R)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def plot_time_series(samples: List[Tuple[float, float, float, float, Optional[int], Optional[float], Optional[float]]], out_sat: Path, out_alt: Path, out_rel: Path):
    if not samples:
        return
    t0 = samples[0][0]
    times = [s[0] - t0 for s in samples]
    num_sats = [s[4] if s[4] is not None else float("nan") for s in samples]
    altitudes = [s[3] for s in samples]
    rel_altitudes = [s[6] for s in samples]  # may contain None
    alt0 = altitudes[0]
    alt_drift = [a - alt0 for a in altitudes]

    # Satellites over time
    fig1, ax1 = plt.subplots(figsize=(7, 3), dpi=120)
    ax1.plot(times, num_sats, marker="o", markersize=2, linewidth=1.0)
    ax1.grid(True, linestyle="--", alpha=0.4)
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Satellites (#)")
    ax1.set_title("Satellites over time")
    fig1.tight_layout()
    out_sat.parent.mkdir(parents=True, exist_ok=True)
    fig1.savefig(out_sat)
    plt.close(fig1)

    # Altitude drift over time
    fig2, ax2 = plt.subplots(figsize=(7, 3), dpi=120)
    ax2.plot(times, alt_drift, marker=".", markersize=2, linewidth=1.0, color="#d62728")
    ax2.grid(True, linestyle="--", alpha=0.4)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Altitude drift (m)")
    ax2.set_title("Altitude drift over time (relative to start)")
    fig2.tight_layout()
    fig2.savefig(out_alt)
    plt.close(fig2)

    # EKF relative altitude over time (ignore None)
    rel_pairs = [(t, ra) for t, ra in zip(times, rel_altitudes) if ra is not None]
    if rel_pairs:
        rt, rv = zip(*rel_pairs)
        fig3, ax3 = plt.subplots(figsize=(7, 3), dpi=120)
        ax3.plot(list(rt), list(rv), linewidth=1.2, color="#2ca02c")
        ax3.grid(True, linestyle="--", alpha=0.4)
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Relative altitude (m)")
        ax3.set_title("EKF2 relative altitude over time")
        fig3.tight_layout()
        fig3.savefig(out_rel)
        plt.close(fig3)


def main():
    parser = argparse.ArgumentParser(description="Log GPS for 2 minutes over serial and plot drift (no arming)")
    parser.add_argument("--port", default=None, help="Serial port, e.g. COM17 (auto-detect if omitted)")
    parser.add_argument("--baud", type=int, default=57600, help="Baud rate (used if --port given)")
    parser.add_argument("--bauds", default="921600,57600,115200", help="Baud list for auto-detect")
    parser.add_argument("--duration", type=float, default=120.0, help="Recording duration seconds")
    parser.add_argument("--outdir", default=str(Path(__file__).resolve().parent / "sessions"), help="Output root directory")
    args = parser.parse_args()

    # Prepare output session folder
    session_root = Path(args.outdir)
    session_dir = session_root / datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir.mkdir(parents=True, exist_ok=True)

    # Serial selection
    port = args.port
    baud = args.baud
    if port is None:
        bauds_tuple = tuple(int(x.strip()) for x in args.bauds.split(",") if x.strip())
        print(f"[detect] scanning serials, baud candidates: {bauds_tuple}")
        detected = detect_px4_serial(bauds_tuple, heartbeat_timeout_s=2.0)
        if detected is None:
            print("No serial detected. Provide --port, e.g. --port COM17")
            return
        port, baud = detected

    print(f"[open] {port} @ {baud}")
    conn = mavutil.mavlink_connection(device=port, baud=baud, autoreconnect=True)

    print("[wait] heartbeat...")
    if not wait_heartbeat(conn, timeout_s=10.0):
        print("No heartbeat. Check wiring and baud.")
        return
    print("[ok] connected. collecting GPS...")

    samples = collect_gps_samples(conn, duration_s=args.duration)
    print(f"[done] collected {len(samples)} samples")

    # Save CSV
    csv_path = session_dir / "gps_drift.csv"
    save_csv(csv_path, samples)
    print(f"[save] csv: {csv_path}")

    # Plots
    drift_png = session_dir / "gps_drift_xy.png"
    sats_png = session_dir / "gps_satellites_over_time.png"
    alt_png = session_dir / "gps_altitude_drift_over_time.png"
    plot_drift_xy(samples, drift_png)
    plot_time_series(samples, sats_png, alt_png, session_dir / "ekf_relative_alt_over_time.png")

    # Metrics
    max_radius, rms_radius, avg_radius = compute_drift_metrics(samples)
    print(f"[metrics] max_radius={max_radius:.2f} m, rms_radius={rms_radius:.2f} m, avg_radius={avg_radius:.2f} m")
    print(f"[output] session folder: {session_dir}")


if __name__ == "__main__":
    main()

