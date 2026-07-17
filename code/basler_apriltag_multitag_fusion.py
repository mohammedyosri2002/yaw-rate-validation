# ============================================================
# WARNINGS / ASSUMPTIONS  (read once before running)
# ------------------------------------------------------------
# 1. Calibrated intrinsics below are HARD-CODED. Recalibrate and
#    update them if you change lens, focus, aperture, or resolution.
# 2. Yaw sign is flipped (yaw = -yaw) to match the motor/encoder
#    direction. Verify this matches YOUR rig's positive direction.
# 3. Have ALL tags in view BEFORE motion starts. A tag first seen
#    after the body has rotated gets a wrong zero reference and may
#    be outlier-rejected or bias the fused yaw.
# 4. The on-line 'fused_yaw_rate_deg_s' is a raw finite difference
#    (noisy). For final RMS, compute the velocity OFFLINE with a
#    Savitzky-Golay derivative on fused_yaw_deg vs wall_time_ms.
# 5. Timestamps are taken at frame GRAB (good), but on the PC clock.
#    The encoder uses its own micros() clock. Align the two using the
#    LED flash: find the 'frame_brightness' spike here and match it to
#    the encoder's "# SYNC time_us=..." line.
# 6. Video FPS is now the REAL measured capture rate (auto), or a fixed
#    value if you set OUTPUT_FPS. It is NOT the camera's max frame rate.
#    This makes saved-video playback duration match the recording.
# 7. mp4v codec may be missing on some machines -> auto-falls back to
#    MJPG/.avi. If neither opens, it runs without video (with a warning).
# 8. Overlays + video add CPU and lower the real capture fps. For
#    precision runs, set SAVE_VIDEO=False and/or DRAW_OVERLAYS=False.
# 9. 'frame_brightness' is a full-frame mean. For a faint LED, restrict
#    it to a small ROI where the LED appears for better sensitivity.
# ============================================================

import time
import math
import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pypylon import pylon
from pupil_apriltags import Detector

# ============================================================
# BASLER + APRILTAG MULTI-TAG YAW FUSION  (improved + fps fix)
# ============================================================


# ============================================================
# USER INPUT
# ============================================================
duration_sec = float(input("Enter recording duration in seconds: ").strip())
filter_mode = input("Type 'filter' to enable filtering, or press Enter for raw fusion only: ").strip().lower()
USE_FILTER = (filter_mode == "filter")

print(f"\nRecording duration: {duration_sec} s")
print(f"Filter enabled: {USE_FILTER}")


# ============================================================
# FEATURE TOGGLES
# ============================================================
SAVE_VIDEO      = True      # write annotated processed video to disk
DRAW_OVERLAYS   = True      # green tag outlines + HUD text
DRAW_TAG_IDS    = False     # external "ID: n" label next to each tag (off by default)
LOG_BRIGHTNESS  = True      # log frame mean brightness (for LED sync flash detection)

# Video frame rate:
#   OUTPUT_FPS = None  -> auto-measure the REAL capture rate over a short warm-up
#   OUTPUT_FPS = 30.0  -> force a fixed output fps
OUTPUT_FPS         = None
WARMUP_FRAMES      = 20      # frames used to measure real fps before opening the writer
VIDEO_FPS_FALLBACK = 30.0    # used only if the real rate can't be measured


# ============================================================
# APRILTAG SETTINGS  (black tag size = 70 mm)
# ============================================================
TAG_FAMILY = "tag36h11"
TAG_SIZE_M = 0.07


# ============================================================
# CALIBRATED CAMERA PARAMETERS
# ============================================================
CAMERA_MATRIX = np.array([
    [1443.2496917062335, 0.0, 963.2151010468531],
    [0.0, 1443.737705496836, 580.3332937596235],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

DIST_COEFFS = np.array([
    -0.1575447, 0.16190211, -0.00044674, -0.00022088, -0.11840788
], dtype=np.float64)


# ============================================================
# CAMERA SETTINGS
# ============================================================
EXPOSURE_US = 5000.0
GAIN_DB = 0.0


# ============================================================
# FILTER / FUSION SETTINGS
# ============================================================
LPF_ALPHA = 0.20
OUTLIER_THRESHOLD_DEG = 12.0


# ============================================================
# HELPER FUNCTIONS  (unchanged math)
# ============================================================
def rotation_matrix_to_euler_xyz(R):
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def unwrap_angle_deg(current_wrapped, previous_unwrapped):
    if previous_unwrapped is None:
        return current_wrapped
    delta = current_wrapped - previous_unwrapped
    while delta > 180.0:
        current_wrapped -= 360.0
        delta = current_wrapped - previous_unwrapped
    while delta < -180.0:
        current_wrapped += 360.0
        delta = current_wrapped - previous_unwrapped
    return current_wrapped


def low_pass_filter(current_value, previous_filtered, alpha):
    if previous_filtered is None:
        return current_value
    return alpha * current_value + (1.0 - alpha) * previous_filtered


def safe_text(v):
    if v is None or (isinstance(v, float) and (np.isnan(v) or np.isinf(v))):
        return "N/A"
    return f"{v:.2f}"


def fuse_tag_measurements(tag_measurements):
    if len(tag_measurements) == 0:
        return None, 0, []
    yaws = np.array([m[1] for m in tag_measurements], dtype=np.float64)
    weights = np.array([max(m[2], 1.0) for m in tag_measurements], dtype=np.float64)
    ids = [m[0] for m in tag_measurements]
    if len(yaws) == 1:
        return float(yaws[0]), 1, ids
    median_yaw = np.median(yaws)
    keep_mask = np.abs(yaws - median_yaw) <= OUTLIER_THRESHOLD_DEG
    kept_yaws = yaws[keep_mask]
    kept_weights = weights[keep_mask]
    kept_ids = [ids[i] for i in range(len(ids)) if keep_mask[i]]
    if len(kept_yaws) == 0:
        return float(median_yaw), 1, []
    fused_yaw = float(np.sum(kept_yaws * kept_weights) / np.sum(kept_weights))
    return fused_yaw, len(kept_yaws), kept_ids


# ============================================================
# FILE NAMES
# ============================================================
stamp = time.strftime("%Y%m%d_%H%M%S")
CSV_FILE = f"apriltag_multitag_fused_{stamp}.csv"
XLSX_FILE = f"apriltag_multitag_fused_{stamp}.xlsx"
PLOT_YAW_FILE = f"apriltag_multitag_fused_yaw_{stamp}.png"
PLOT_RATE_FILE = f"apriltag_multitag_fused_rate_{stamp}.png"
VIDEO_FILE = f"apriltag_multitag_fused_{stamp}.mp4"


# ============================================================
# BASLER CAMERA SETUP
# ============================================================
camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
camera.Open()

try:
    camera.UserSetSelector.SetValue("Default")
    camera.UserSetLoad.Execute()
except Exception:
    pass

for setter in [
    lambda: camera.PixelFormat.SetValue("Mono8"),
    lambda: camera.TriggerMode.SetValue("Off"),
    lambda: camera.AcquisitionMode.SetValue("Continuous"),
    lambda: camera.ExposureAuto.SetValue("Off"),
    lambda: camera.GainAuto.SetValue("Off"),
]:
    try:
        setter()
    except Exception:
        pass

try:
    camera.ExposureTime.SetValue(EXPOSURE_US)
except Exception:
    pass
try:
    camera.Gain.SetValue(GAIN_DB)
except Exception:
    pass

converter = pylon.ImageFormatConverter()
converter.OutputPixelFormat = pylon.PixelType_Mono8
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

width = camera.Width.Value
height = camera.Height.Value

# Informational only: the camera's MAX achievable rate (NOT used for the
# video writer). This is the value that previously caused the too-fast video.
try:
    cam_max_fps = float(camera.ResultingFrameRate.GetValue())
    print(f"Camera ResultingFrameRate (sensor max, NOT used for video): {cam_max_fps:.1f} fps")
except Exception:
    cam_max_fps = None

# precompute undistort maps once
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
    CAMERA_MATRIX, DIST_COEFFS, (width, height), 1, (width, height)
)
map1, map2 = cv2.initUndistortRectifyMap(
    CAMERA_MATRIX, DIST_COEFFS, None, new_camera_matrix, (width, height), cv2.CV_16SC2
)

fx_u = float(new_camera_matrix[0, 0])
fy_u = float(new_camera_matrix[1, 1])
cx_u = float(new_camera_matrix[0, 2])
cy_u = float(new_camera_matrix[1, 2])


# ============================================================
# VIDEO WRITER HELPER
# Opens an annotated, full-resolution, color writer at the given fps,
# with an automatic codec/container fallback.
# ============================================================
def open_video_writer(fps):
    global VIDEO_FILE
    fps = float(max(1.0, min(120.0, fps)))
    vw = cv2.VideoWriter(VIDEO_FILE, cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height), True)
    if not vw.isOpened():
        VIDEO_FILE = f"apriltag_multitag_fused_{stamp}.avi"
        vw = cv2.VideoWriter(VIDEO_FILE, cv2.VideoWriter_fourcc(*"MJPG"), fps, (width, height), True)
    if not vw.isOpened():
        return None, None
    return vw, fps


# ============================================================
# APRILTAG DETECTOR
# ============================================================
detector = Detector(
    families=TAG_FAMILY, nthreads=4, quad_decimate=1.0, quad_sigma=0.0,
    refine_edges=1, decode_sharpening=0.25, debug=0
)


# ============================================================
# MAIN STATE
# ============================================================
rows = []
start_time = time.time()
start_wall_ms = int(round(start_time * 1000.0))

prev_elapsed = None
prev_fused_yaw = None
prev_filtered_yaw = None

yaw_zero_ref = {}
yaw_prev_unwrapped = {}

HUD_FONT = cv2.FONT_HERSHEY_SIMPLEX

# video writer state
video_writer = None
writer_fps = None
warmup_buffer = []   # list of (annotated_frame, capture_time_s) until writer opens

if SAVE_VIDEO and OUTPUT_FPS is not None:
    video_writer, writer_fps = open_video_writer(OUTPUT_FPS)
    if video_writer is None:
        print("WARNING: could not open a video writer; continuing without video.")
        SAVE_VIDEO = False
    else:
        print(f"Video writer ready (fixed): {VIDEO_FILE} @ {writer_fps:.2f} fps, {width}x{height}")
elif SAVE_VIDEO:
    print(f"Video writer will open after measuring the real capture rate (~{WARMUP_FRAMES} frames).")

print("\nBasler multi-tag fusion capture started...")
print("Press ESC to stop early.")

try:
    while camera.IsGrabbing():
        now = time.time()
        if (now - start_time) >= duration_sec:
            break

        grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

        if grab_result.GrabSucceeded():
            # --- timestamp AT capture, before any heavy processing ---
            cap_now = time.time()
            wall_time_ms = int(round(cap_now * 1000.0))
            elapsed_s = cap_now - start_time
            elapsed_time_ms = wall_time_ms - start_wall_ms

            image = converter.Convert(grab_result)
            frame = image.GetArray()                 # Mono8 (grayscale)

            frame_brightness = float(np.mean(frame)) if LOG_BRIGHTNESS else None

            undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

            tags = detector.detect(
                undistorted, estimate_tag_pose=True,
                camera_params=[fx_u, fy_u, cx_u, cy_u], tag_size=TAG_SIZE_M
            )

            disp = cv2.cvtColor(undistorted, cv2.COLOR_GRAY2BGR) if (DRAW_OVERLAYS or SAVE_VIDEO) else None

            visible_ids = []
            fused_yaw_deg = None
            fused_yaw_rate_deg_s = None
            used_ids = []
            used_count = 0
            tag_measurements = []

            for tag in tags:
                tag_id = int(tag.tag_id)

                # green outline around every detected tag (ID label optional)
                if disp is not None and DRAW_OVERLAYS:
                    corners = tag.corners.astype(np.int32)
                    cv2.polylines(disp, [corners.reshape(-1, 1, 2)], True, (0, 255, 0), 2, cv2.LINE_AA)
                    if DRAW_TAG_IDS:
                        top_idx = int(np.argmin(corners[:, 1]))
                        tx, ty = corners[top_idx]
                        cv2.putText(disp, f"ID: {tag_id}", (int(tx) - 10, int(ty) - 12),
                                    HUD_FONT, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

                if not hasattr(tag, "pose_R"):
                    continue

                decision_margin = float(tag.decision_margin)
                _, _, yaw_wrapped = rotation_matrix_to_euler_xyz(tag.pose_R)
                yaw_wrapped = -yaw_wrapped

                prev_unwrapped = yaw_prev_unwrapped.get(tag_id, None)
                yaw_unwrapped = unwrap_angle_deg(yaw_wrapped, prev_unwrapped)
                yaw_prev_unwrapped[tag_id] = yaw_unwrapped

                if tag_id not in yaw_zero_ref:
                    yaw_zero_ref[tag_id] = yaw_unwrapped

                yaw_relative = yaw_unwrapped - yaw_zero_ref[tag_id]
                visible_ids.append(tag_id)
                tag_measurements.append((tag_id, yaw_relative, decision_margin))

            visible_count = len(tag_measurements)

            if visible_count > 0:
                fused_yaw_deg, used_count, used_ids = fuse_tag_measurements(tag_measurements)
                if USE_FILTER:
                    fused_yaw_deg = low_pass_filter(fused_yaw_deg, prev_filtered_yaw, LPF_ALPHA)
                    prev_filtered_yaw = fused_yaw_deg
                if prev_elapsed is not None and prev_fused_yaw is not None:
                    dt = elapsed_s - prev_elapsed
                    if dt > 0:
                        fused_yaw_rate_deg_s = (fused_yaw_deg - prev_fused_yaw) / dt
                prev_fused_yaw = fused_yaw_deg
                prev_elapsed = elapsed_s
            else:
                prev_fused_yaw = None
                prev_elapsed = elapsed_s

            visible_ids_text = ",".join(map(str, visible_ids)) if visible_ids else "N/A"
            used_ids_text = ",".join(map(str, used_ids)) if used_ids else "N/A"

            if disp is not None and DRAW_OVERLAYS:
                hud = [
                    f"Time (s): {elapsed_s:.3f}",
                    f"Visible tags: {visible_ids_text}",
                    f"Used tags: {used_ids_text}",
                    f"Fused yaw (deg): {safe_text(fused_yaw_deg)}",
                    f"Fused yaw rate (deg/s): {safe_text(fused_yaw_rate_deg_s)}",
                    f"Filter: {'ON' if USE_FILTER else 'OFF'}",
                ]
                for i, txt in enumerate(hud):
                    y = 40 + i * 36
                    cv2.putText(disp, txt, (20, y), HUD_FONT, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
                    cv2.putText(disp, txt, (20, y), HUD_FONT, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

            # ---- video: write directly, or buffer during warm-up to measure real fps ----
            if disp is not None:
                if SAVE_VIDEO:
                    if video_writer is not None:
                        video_writer.write(disp)
                    else:
                        # auto mode: collect warm-up frames, then open writer at measured fps
                        warmup_buffer.append((disp, cap_now))
                        if len(warmup_buffer) >= WARMUP_FRAMES:
                            t0 = warmup_buffer[0][1]
                            t1 = warmup_buffer[-1][1]
                            meas_fps = (len(warmup_buffer) - 1) / (t1 - t0) if t1 > t0 else VIDEO_FPS_FALLBACK
                            video_writer, writer_fps = open_video_writer(meas_fps)
                            if video_writer is not None:
                                print(f"Video writer ready (measured): {VIDEO_FILE} @ {writer_fps:.2f} fps")
                                for f_img, _ in warmup_buffer:
                                    video_writer.write(f_img)
                            else:
                                print("WARNING: could not open a video writer; continuing without video.")
                                SAVE_VIDEO = False
                            warmup_buffer = []
                view = cv2.resize(disp, (1280, 720))
                cv2.imshow("Basler AprilTag Multi-Tag Fusion", view)

            row = {
                "wall_time_ms": wall_time_ms,
                "elapsed_time_ms": elapsed_time_ms,
                "elapsed_time_s": elapsed_s,
                "visible_count": visible_count,
                "visible_ids": visible_ids_text,
                "used_count_after_outlier_rejection": used_count,
                "used_ids": used_ids_text,
                "fused_yaw_deg": fused_yaw_deg,
                "fused_yaw_rate_deg_s": fused_yaw_rate_deg_s,
                "filter_enabled": int(USE_FILTER),
            }
            if LOG_BRIGHTNESS:
                row["frame_brightness"] = frame_brightness
            for (tag_id, yaw_relative, decision_margin) in tag_measurements:
                row[f"tag_{tag_id}_yaw_deg"] = yaw_relative
                row[f"tag_{tag_id}_decision_margin"] = decision_margin
            rows.append(row)

            if (cv2.waitKey(1) & 0xFF) == 27:
                break

        grab_result.Release()

finally:
    if camera.IsGrabbing():
        camera.StopGrabbing()
    camera.Close()
    # if the run ended before the warm-up completed, open the writer now and flush
    if SAVE_VIDEO and video_writer is None and len(warmup_buffer) >= 2:
        t0 = warmup_buffer[0][1]
        t1 = warmup_buffer[-1][1]
        meas_fps = (len(warmup_buffer) - 1) / (t1 - t0) if t1 > t0 else VIDEO_FPS_FALLBACK
        video_writer, writer_fps = open_video_writer(meas_fps)
        if video_writer is not None:
            for f_img, _ in warmup_buffer:
                video_writer.write(f_img)
    if video_writer is not None:
        video_writer.release()
    cv2.destroyAllWindows()


# ============================================================
# SAVE DATA
# ============================================================
df = pd.DataFrame(rows)
df.to_csv(CSV_FILE, index=False)
df.to_excel(XLSX_FILE, index=False)

print("\nSaved files:")
print(CSV_FILE)
print(XLSX_FILE)
if SAVE_VIDEO and writer_fps is not None:
    print(VIDEO_FILE)

# report the true average capture rate and confirm the writer matches it
if len(df) > 1:
    dur = df["elapsed_time_s"].iloc[-1] - df["elapsed_time_s"].iloc[0]
    if dur > 0:
        real_fps = (len(df) - 1) / dur
        print(f"Measured average capture rate: {real_fps:.2f} fps")
        if writer_fps is not None:
            print(f"Video written at: {writer_fps:.2f} fps "
                  f"(playback duration ~ {len(df) / writer_fps:.1f} s vs recording {dur:.1f} s)")


# ============================================================
# PLOTS
# ============================================================
valid_yaw = df.dropna(subset=["fused_yaw_deg"]).copy()
if len(valid_yaw) > 0:
    plt.figure(figsize=(14, 6))
    plt.plot(valid_yaw["elapsed_time_s"], valid_yaw["fused_yaw_deg"], label="Fused Yaw")
    plt.xlabel("Time (s)"); plt.ylabel("Yaw angle (deg)")
    plt.title("Fused Yaw Angle vs Time"); plt.grid(True); plt.legend(); plt.tight_layout()
    plt.savefig(PLOT_YAW_FILE, dpi=220); plt.close()
    print(PLOT_YAW_FILE)

valid_rate = df.dropna(subset=["fused_yaw_rate_deg_s"]).copy()
if len(valid_rate) > 0:
    plt.figure(figsize=(14, 6))
    plt.plot(valid_rate["elapsed_time_s"], valid_rate["fused_yaw_rate_deg_s"], label="Fused Yaw Rate")
    plt.xlabel("Time (s)"); plt.ylabel("Yaw angular velocity (deg/s)")
    plt.title("Fused Yaw Angular Velocity vs Time"); plt.grid(True); plt.legend(); plt.tight_layout()
    plt.savefig(PLOT_RATE_FILE, dpi=220); plt.close()
    print(PLOT_RATE_FILE)

print("\nDone.")
