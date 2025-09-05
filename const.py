import numpy as np

missle_1 = np.array([20000, 0, 2000])
missle_2 = np.array([19000, 600, 2100])
missle_3 = np.array([18000, -600, 1900])
missle_v_scalar = 300

plane_1 = np.array([17800, 0, 1800])
plane_2 = np.array([12000, 1400, 1400])
plane_3 = np.array([6000, -3000, 700])
plane_4 = np.array([11000, 2000, 1800])
plane_5 = np.array([13000, -2000, 1300])
plane_v_range = np.array([70, 140])

smoke_v_fall = 3
smoke_radius = 10
smoke_duration = 20

target_radius = 7
target_height = 10
# 目标圆柱的几何中心
target_center = np.array([0, 200, 0]) + np.array([0, 0, target_height / 2.0])