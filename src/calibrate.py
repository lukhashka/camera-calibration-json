import cv2
import numpy as np
import glob
import json
import os

def calibrate_camera(images_path, checkerboard_size=(11, 8), square_size=0.025):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
    objp *= square_size

    objpoints = [] 
    imgpoints = [] 

    images = []
    for ext in ['*.jpg', '*.png', '*.jpeg', '*.JPG']:
        images.extend(glob.glob(os.path.join(images_path, ext)))
    
    if not images:
        print(f"pictures not found in {images_path}")
        return None, None, None

    img_shape = None

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_shape = gray.shape[::-1]

        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK

        ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, flags)

        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            print(f"successfully processed: {os.path.basename(fname)}")
        else:
            print(f"can't fing chessboard at: {os.path.basename(fname)}")

    if len(objpoints) == 0:
        print("\ncan't fing chessboard at each picture")
        return None, None, None

    print("\ncalculating")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_shape, None, None)

    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    total_error = mean_error / len(objpoints)
    
    print(f"calibrate successfully | Total Reprojection Error: {total_error:.4f} pixels")
    
    return mtx, dist, total_error

def save_calibration(mtx, dist, error, output_path):
    config = {
        "reprojection_error": float(error),
        "camera_matrix": mtx.tolist(),
        "distortion_coefficients": dist.tolist()[0],
        "note": "Intrinsics formatted for NeRF/3DGS pipeline preprocessing"
    }
    with open(output_path, 'w') as f:
        json.dump(config, f, indent=4)
    print(f"config saved in {output_path}")

def undistort_image(img_path, mtx, dist, out_path):
    img = cv2.imread(img_path)
    if img is None:
        print(f"can't read image: {img_path}")
        return
        
    h, w = img.shape[:2]
    
    dst = cv2.undistort(img, mtx, dist, None, mtx)
    
    cv2.imwrite(out_path, dst)
    print(f"picture saved in {out_path} (size: {dst.shape[1]}x{dst.shape[0]})")

if __name__ == "__main__":
    CHECKERBOARD_DIR = "data/checkerboard" 
    
    TEST_IMAGE = "data/raw_drone_images/test.png"  
    
    CONFIG_OUT = "calibration_config.json"
    IMAGE_OUT = "output/undistorted_test.jpg"
    
    os.makedirs("output", exist_ok=True)

    mtx, dist, error = calibrate_camera(CHECKERBOARD_DIR, checkerboard_size=(9, 6))
    
    if mtx is not None:
        save_calibration(mtx, dist, error, CONFIG_OUT)
        
        if os.path.exists(TEST_IMAGE):
            undistort_image(TEST_IMAGE, mtx, dist, IMAGE_OUT)
        else:
            print(f"file not found by path: {TEST_IMAGE}")