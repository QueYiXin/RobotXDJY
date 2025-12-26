import cv2
import numpy as np
import sys

def remove_small_clusters(input_pgm, output_pgm, max_noise_size=5):
    """
    使用连通域分析去除小的噪点团簇。
    :param input_pgm: 输入地图路径
    :param output_pgm: 输出地图路径
    :param max_noise_size: 阈值。如果一个黑点团簇的像素数量 <= 这个值，就被视为噪点移除。
                           建议设为 3 到 5，可以同时去除单点和两三个点连在一起的噪点。
    """
    # 1. 读取原图
    src = cv2.imread(input_pgm, cv2.IMREAD_GRAYSCALE)
    if src is None:
        print(f"Error: 无法读取图像 {input_pgm}")
        sys.exit(1)
    
    # 2. 准备数据：提取障碍物
    # 在 SLAM 地图中，0 是黑（障碍），我们要把它们提取出来变成“前景”
    # 创建一个二值图：障碍物为 255 (白)，其他为 0 (黑)，方便 OpenCV 处理
    # 注意：这里我们只提取完全黑的点 (像素值 < 10)
    binary_obstacles = (src < 10).astype(np.uint8) * 255

    # 3. 核心算法：连通域分析
    # connectivity=8 表示检查周围 8 个方向（对角线相连也算连在一起）
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_obstacles, connectivity=8)

    # 4. 创建输出副本
    dst = src.copy()
    
    print(f"检测到 {num_labels - 1} 个障碍物块...")
    removed_count = 0

    # 5. 遍历所有连通域
    # 注意：label=0 是背景，我们从 1 开始遍历
    for i in range(1, num_labels):
        # 获取该连通域的面积 (像素个数)
        area = stats[i, cv2.CC_STAT_AREA]
        
        # 你的需求：去除 2 个点连在一起的，或者 1 个点的
        # 所以只要面积很小，就认为是噪点
        if area <= max_noise_size:
            # 将该连通域对应的像素在原图中涂白 (254 = Free Space)
            # labels == i 会生成一个掩码，选中属于该连通域的所有像素
            dst[labels == i] = 254
            removed_count += 1

    print(f"处理完成。移除了 {removed_count} 个小于等于 {max_noise_size} 像素的小团簇。")
    print(f"已保存至: {output_pgm}")
    
    # 保存结果
    cv2.imwrite(output_pgm, dst)

# --- 运行示例 ---
if __name__ == "__main__":
    input_map = "map_r2r_origin.pgm"
    output_map = "map_r2r.pgm"
    
    # 阈值设为 4：
    # - 单个噪点 (面积1) -> 删除
    # - 你描述的横向噪点 [0, 0] (面积2) -> 删除
    # - L型或斜向的小噪点 (面积3或4) -> 删除
    # - 真正的墙壁 (面积通常 > 50) -> 完美保留
    remove_small_clusters(input_map, output_map, max_noise_size=5)