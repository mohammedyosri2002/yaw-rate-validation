import cv2
import numpy as np
import glob
import os

# ============================================================
# CHECKERBOARD SETTINGS
# IMPORTANT:
# Printed board = 10 x 7 squares
# So inner corners = 9 x 6
# Square size = 20 mm
# ============================================================

CHECKERBOARD = (9, 6)     # inner corners per row and column
SQUARE_SIZE_MM = 20.0     # real square size in millimeters

# Folder containing captured calibration images
IMAGE_FOLDER = "calib_images"
IMAGE_PATTERN = os.path.join(IMAGE_FOLDER, "*.png")

# ============================================================
# TERMINATION CRITERIA FOR SUBPIXEL CORNER REFINEMENT
# ============================================================
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# ============================================================
# PREPARE 3D OBJECT POINTS
# Example:
# (0,0,0), (20,0,0), (40,0,0), ...
# ============================================================
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE_MM

# Arrays to store object points and image points
objpoints = []   # 3D points in real world
imgpoints = []   # 2D points in image plane

images = glob.glob(IMAGE_PATTERN)

if len(images) == 0:
    raise FileNotFoundError(f"No images found in folder: {IMAGE_FOLDER}")

print(f"Found {len(images)} images")

valid_images = 0
image_size = None

for fname in images:
    img = cv2.imread(fname, cv2.IMREAD_GRAYSCALE)

    if img is None:
        print(f"Could not read: {fname}")
        continue

    image_size = img.shape[::-1]

    # Try to find checkerboard corners
    ret, corners = cv2.findChessboardCorners(
        img,
        CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    if ret:
        # Refine corner locations for better accuracy
        corners2 = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), criteria)

        objpoints.append(objp)
        imgpoints.append(corners2)
        valid_images += 1

        # Optional preview
        vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.drawChessboardCorners(vis, CHECKERBOARD, corners2, ret)
        cv2.imshow("Detected Corners", cv2.resize(vis, (1280, 720)))
        cv2.waitKey(200)

        print(f"[OK] {fname}")
    else:
        print(f"[FAIL] Checkerboard not found in: {fname}")

cv2.destroyAllWindows()

if valid_images < 8:
    raise RuntimeError("Too few valid calibration images. Capture more clear images from different angles.")

print(f"\nValid calibration images: {valid_images}")

# ============================================================
# RUN CAMERA CALIBRATION
# ============================================================
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints,
    imgpoints,
    image_size,
    None,
    None
)

# ============================================================
# REPROJECTION ERROR
# ============================================================
total_error = 0
for i in range(len(objpoints)):
    projected_points, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
    error = cv2.norm(imgpoints[i], projected_points, cv2.NORM_L2) / len(projected_points)
    total_error += error

mean_error = total_error / len(objpoints)

# ============================================================
# PRINT RESULTS
# ============================================================
print("\n================ CALIBRATION RESULTS ================")
print(f"RMS reprojection error from OpenCV: {ret}")
print(f"Mean reprojection error: {mean_error}")

print("\nCamera Matrix:")
print(camera_matrix)

print("\nDistortion Coefficients:")
print(dist_coeffs)

fx = camera_matrix[0, 0]
fy = camera_matrix[1, 1]
cx = camera_matrix[0, 2]
cy = camera_matrix[1, 2]

print("\nUseful values:")
print(f"FX = {fx}")
print(f"FY = {fy}")
print(f"CX = {cx}")
print(f"CY = {cy}")

# ============================================================
# SAVE RESULTS TO FILE
# ============================================================
np.savez(
    "camera_calibration_results.npz",
    camera_matrix=camera_matrix,
    dist_coeffs=dist_coeffs,
    fx=fx,
    fy=fy,
    cx=cx,
    cy=cy,
    rms=ret,
    mean_error=mean_error
)

print("\nSaved calibration results to: camera_calibration_results.npz")

# ============================================================
# OPTIONAL: TEST UNDISTORTION ON FIRST IMAGE
# ============================================================
test_img = cv2.imread(images[0], cv2.IMREAD_GRAYSCALE)
h, w = test_img.shape[:2]

new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
undistorted = cv2.undistort(test_img, camera_matrix, dist_coeffs, None, new_camera_matrix)

cv2.imwrite("undistorted_test.png", undistorted)
print("Saved undistorted test image as: undistorted_test.png")
