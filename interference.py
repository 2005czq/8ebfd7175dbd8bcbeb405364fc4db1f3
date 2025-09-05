from const import *
import numpy as np

G = 9.8


def is_interfered(missile_pos, smoke_center):
    """
    判断在当前时刻，烟幕云团是否完全遮挡了导弹对目标的视线。
    
    Args:
        missile_pos (np.array): 导弹当前位置 (3D)
        smoke_center (np.array): 烟幕云团球心当前位置 (3D)

    Returns:
        bool: 如果完全遮挡，返回 True，否则返回 False
    """

    vec_mt = target_center - missile_pos
    vec_ms = smoke_center - missile_pos
    dist_mt_sq = np.dot(vec_mt, vec_mt)
    
    # 1. 前提条件检查：烟幕必须在导弹和目标之间
    # 投影长度必须为正且小于总长度
    proj_len = np.dot(vec_ms, vec_mt)
    if proj_len <= 0 or proj_len >= dist_mt_sq:
        return False

    # 2. 计算视锥在烟幕深度的参数
    dist_mt = np.sqrt(dist_mt_sq)
    dist_ms = np.linalg.norm(vec_ms)
    ratio = dist_ms / dist_mt
    
    # 视锥横截面的半径和半高
    r_cone_at_s = target_radius * ratio
    h_cone_half_at_s = (target_height / 2.0) * ratio
    
    # 3. 计算烟幕球心与视锥中心线的偏移距离
    # 视锥中心线上与烟幕深度相同的点 C
    centerline_point_c = missile_pos + (vec_mt / dist_mt) * dist_ms
    offset_distance = np.linalg.norm(smoke_center - centerline_point_c)
    
    # 4. 判断是否完全覆盖
    # 视锥横截面最远的点（角点）到中心线的距离
    max_cone_radius = np.sqrt(r_cone_at_s**2 + h_cone_half_at_s**2)
    
    if offset_distance + max_cone_radius <= smoke_radius:
        return True
        
    return False


def calc_interference(plane_v, drop_t, explode_t):
    """
    计算在给定参数下，导弹被烟幕干扰的总时长。

    Args:
        plane_v (np.array): 飞机的速度矢量 (3D, m/s)
        drop_t (float): 飞机飞行后投放烟幕弹的时间 (s)
        explode_t (float): 烟幕弹被投放后到起爆的间隔时间 (s)

    Returns:
        float: 导弹被干扰的总时长 (s)
    """
    # --- 1. 计算各物体运动轨迹参数 ---
    
    # 导弹
    missile_dir = -missle_1
    missile_dir_unit = missile_dir / np.linalg.norm(missile_dir)
    missile_v_vec = missle_v_scalar * missile_dir_unit

    # 烟幕弹 & 烟幕云团
    t_drop = drop_t
    t_explode = t_drop + explode_t
    t_dissipate = t_explode + smoke_duration

    # 计算烟幕弹投放位置
    plane_drop_pos = plane_1 + plane_v * t_drop
    
    # 计算烟幕弹起爆位置 (抛体运动)
    fall_duration = explode_t
    # 水平位移
    dx = plane_v[0] * fall_duration
    dy = plane_v[1] * fall_duration
    # 竖直位移
    dz = plane_v[2] * fall_duration - 0.5 * G * fall_duration**2
    
    smoke_explode_pos = plane_drop_pos + np.array([dx, dy, dz])

    # --- 2. 仿真循环 ---
    
    total_interference_time = 0.0
    dt = 0.00001  # 时间步长，越小越精确，计算越慢
    
    current_t = t_explode
    while current_t < t_dissipate:
        # 计算当前时刻各物体位置
        missile_pos = missle_1 + missile_v_vec * current_t
        
        time_since_explode = current_t - t_explode
        smoke_center = smoke_explode_pos - np.array([0, 0, smoke_v_fall * time_since_explode])
        
        # 检查是否干扰
        if is_interfered(missile_pos, smoke_center):
            total_interference_time += dt
            
        current_t += dt
        
    return total_interference_time

if __name__ == "__main__":
    # 1. 计算飞机速度矢量
    # "以 120m/s 的速度朝 (0,0,0) 飞行"，且"保持高度不变"
    plane_speed = 120.0
    # 只考虑XY平面上的方向
    plane_dir_xy = -plane_1[:2] 
    plane_dir_unit_xy = plane_dir_xy / np.linalg.norm(plane_dir_xy)
    plane_v_xy = plane_speed * plane_dir_unit_xy
    # 组合成3D速度矢量
    plane_velocity_vector = np.array([plane_v_xy[0], plane_v_xy[1], 0.0])

    # 2. 设定投放和起爆参数
    drop_time = 1.5
    explode_interval = 3.6

    # 3. 调用函数计算
    interference_duration = calc_interference(plane_velocity_vector, drop_time, explode_interval)

    # 4. 打印结果
    print(f"飞机初始位置: {plane_1}")
    print(f"飞机速度矢量: {np.round(plane_velocity_vector, 2)} m/s")
    print(f"投放时间: {drop_time} s")
    print(f"起爆间隔: {explode_interval} s")
    print("-" * 30)
    print(f"导弹被干扰的总时长为: {interference_duration:.4f} 秒")