import open3d as o3d
import numpy as np
from PIL import Image
import yaml
import os

def pcd_to_grid(pcd_path, output_dir, resolution=0.05, 
                height_min=0.0, height_max=2.0):  # 过滤地面
    """3D点云转2D栅格，过滤地面点避免全图障碍"""
    os.makedirs(output_dir, exist_ok=True)
    
    pcd = o3d.io.read_point_cloud(pcd_path)
    pts = np.asarray(pcd.points)
    print(f"Total points: {len(pts)}")
    
    mask = (pts[:, 2] > height_min) & (pts[:, 2] < height_max)
    pts = pts[mask]
    print(f"Points in [{height_min}, {height_max}]m: {len(pts)}")
    
    if len(pts) == 0:
        raise ValueError("No valid points after filtering")
    
    # 计算边界
    min_x, min_y = np.min(pts[:, :2], axis=0)
    max_x, max_y = np.max(pts[:, :2], axis=0)
    
    # 包含原点
    min_x, min_y = min(0.0, min_x), min(0.0, min_y)
    max_x, max_y = max(0.0, max_x), max(0.0, max_y)
    
    width = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))
    
    print(f"Grid: {width}x{height}")
    
    # 初始化为自由空间 (255 = 白)
    grid = np.ones((height, width), dtype=np.uint8) * 255
    
    # 投影
    cols = ((pts[:, 0] - min_x) / resolution).astype(int)
    rows = ((pts[:, 1] - min_y) / resolution).astype(int)
    rows = height - 1 - rows  # Y轴翻转
    
    # 边界检查
    valid = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    cols, rows = cols[valid], rows[valid]
    
    # 标记障碍物 (0 = 黑)
    grid[rows, cols] = 0
    
    # 保存
    Image.fromarray(grid).save(os.path.join(output_dir, "map.pgm"))
    
    with open(os.path.join(output_dir, "map.yaml"), 'w') as f:
        yaml.dump({
            'image': 'map.pgm',
            'resolution': float(resolution),
            'origin': [float(min_x), float(min_y), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }, f)
    
    print(f"Saved. Obstacle pixels: {np.sum(grid == 0)}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('pcd', type=str)
    parser.add_argument('-o', default='./grid_map')
    parser.add_argument('-r', type=float, default=0.05)
    parser.add_argument('--zmin', type=float, default=-0.2, help='Min height (filter ground)')
    parser.add_argument('--zmax', type=float, default=2.0, help='Max height')
    args = parser.parse_args()
    
    pcd_to_grid(args.pcd, args.o, args.r, args.zmin, args.zmax)