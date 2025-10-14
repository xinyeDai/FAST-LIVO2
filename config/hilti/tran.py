# import numpy as np

# def LiDAR_to_camera(R_iL: np.ndarray, T_iL: np.ndarray,
#                   R_ic: np.ndarray, T_ic: np.ndarray) -> (np.ndarray, np.ndarray):  # type: ignore
    
#     # R_iL 是正交矩阵，其逆等于转置
#     R_Li = R_iL.T  # 或者 np.linalg.inv(R_il)

#     R_Lc = R_Li @ R_ic
#     T_Lc = R_Li @ (T_ic - T_iL)

#     return R_Lc, T_Lc


# if __name__ == "__main__":
#     # 已知imu2LiDAR，imu2camera，求LiDAR2camera
#     R_iL = np.array([[    0.99983,   0.0170232, -0.00707929],
#                      [  0.0171417,   -0.999708,   0.0170198],
#                      [-0.00678749,  -0.0171382,    -0.99983]])                             # IMU->LiDAR rotation
#     T_iL = np.array([-0.003050707070885951, -0.021993853931529066,  0.15076415229379997])  # IMU->LiDAR translation

#     R_ic = np.array([[-0.00273126,  0.00124672,    0.999995],
#                      [   0.999991,  0.00339164,  0.00272701],
#                      [-0.00338823,    0.999993, -0.00125597]])                             # IMU->Camera rotation
#     T_ic = np.array([0.0507054642910155, -0.060959522169800155, -0.005930631162279414])    # IMU->Camera translation

#     R_Lc, T_Lc = LiDAR_to_camera(R_iL, T_iL, R_ic, T_ic)

#     print("Rotation LiDAR->Camera (R_ic):\n", R_Lc)
#     print("Translation LiDAR->Camera (T_ic):\n", T_Lc)

import numpy as np

# ----------------- ①  LiDAR→Camera：通过 IMU 枢纽求解 -----------------
def LiDAR_to_camera(R_iL: np.ndarray, T_iL: np.ndarray,
                    R_ic: np.ndarray, T_ic: np.ndarray) -> (np.ndarray, np.ndarray):
    """
    已知 IMU→LiDAR (R_iL,T_iL) 与 IMU→Camera (R_ic,T_ic)，
    求 LiDAR→Camera (R_Lc,T_Lc)
    """
    R_Li = R_iL.T                      # LiDAR→IMU
    R_Lc = R_Li @ R_ic
    T_Lc = R_Li @ (T_ic - T_iL)
    return R_Lc, T_Lc


# ----------------- ②  Camera→LiDAR：对 LiDAR→Camera 取逆 -----------------
def camera_to_LiDAR(R_Lc: np.ndarray, T_Lc: np.ndarray) -> (np.ndarray, np.ndarray):
    """
    已知 LiDAR→Camera (R_Lc,T_Lc)，求 Camera→LiDAR (R_cL,T_cL)。
      R_cL = R_Lc^T
      T_cL = - R_Lc^T · T_Lc
    """
    R_cL = R_Lc.T                      # 旋转矩阵是正交的，逆=转置
    T_cL = -R_cL @ T_Lc
    return R_cL, T_cL


if __name__ == "__main__":
    # ------- 输入：IMU→LiDAR、IMU→Camera ------
    R_iL = np.array([[ 0.99983,   0.0170232, -0.00707929],
                     [ 0.0171417, -0.999708,  0.0170198 ],
                     [-0.00678749,-0.0171382, -0.99983 ]])
    T_iL = np.array([-0.003050707070885951, -0.021993853931529066, 0.15076415229379997])

    R_ic = np.array([[-0.00273126,  0.00124672,  0.999995 ],
                     [ 0.999991,    0.00339164,  0.00272701],
                     [-0.00338823,  0.999993,   -0.00125597]])
    T_ic = np.array([ 0.0507054642910155, -0.060959522169800155, -0.005930631162279414])

    # ------- 先求 LiDAR→Camera ------
    R_Lc, T_Lc = LiDAR_to_camera(R_iL, T_iL, R_ic, T_ic)
    print("LiDAR  → Camera  R_Lc:\n", R_Lc)
    print("LiDAR  → Camera  T_Lc:\n", T_Lc)

    # ------- 再取逆，求 Camera→LiDAR ------
    R_cL, T_cL = camera_to_LiDAR(R_Lc, T_Lc)
    print("\nCamera → LiDAR  R_cL:\n", R_cL)
    print("Camera → LiDAR  T_cL:\n", T_cL)

    # ------- 验证：R_Lc·R_cL ≈ I,   R_Lc·T_cL + T_Lc ≈ 0 ------
    print("\nCheck  R_Lc·R_cL:\n", R_Lc @ R_cL)
    print("Check  R_Lc·T_cL + T_Lc:\n", R_Lc @ T_cL + T_Lc)

