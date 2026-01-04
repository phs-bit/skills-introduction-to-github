# 最小化代码修改指南

## 修改目标
将原有的存储完整路径的堆改为存储单个节点，每个节点记录路径代价(g)和启发代价(h)。

## 需要修改的部分

### 1. 修改 GeometricPath 类（约第25行）

**原代码：**
```python
class GeometricPath:
    def __init__(self, traj_points, cost, col_check_list, parent_path_index):
        # traj_points: [[x1, y1], [x2, y2], ...] 包含起点终点，以及所有拐点
        self.traj_points = traj_points
        self.collision_check_list = col_check_list # 0 不清楚 1 碰撞 2 未碰撞 
        self.cost = cost
        self.parent_path_index = parent_path_index # 记录父路径的索引，方便回溯

    def __lt__(self, other):
        # 用于 heapq 的比较，代价越小越优先
        return self.cost < other.cost
```

**修改为：**
```python
class GeometricPath:
    def __init__(self, position, g_cost, h_cost, parent_node):
        # position: [x, y] 当前节点位置
        self.position = position
        self.g_cost = g_cost  # 从起点到当前节点的实际代价
        self.h_cost = h_cost  # 从当前节点到目标的启发式代价
        self.f_cost = g_cost + h_cost  # 总代价
        self.parent_node = parent_node  # 父节点引用（用于回溯路径）

    def __lt__(self, other):
        # 用于 heapq 的比较，f_cost 越小越优先
        return self.f_cost < other.f_cost
```

### 2. 添加辅助函数（在 run_geometric_search 函数之前）

```python
def calculate_heuristic(pos1, pos2):
    """计算两点间的启发式代价（欧几里得距离）"""
    return math.hypot(pos2[0] - pos1[0], pos2[1] - pos1[1])

def reconstruct_path(goal_node):
    """从目标节点回溯到起点，重建完整路径"""
    path = []
    current = goal_node
    while current is not None:
        path.append(current.position)
        current = current.parent_node
    path.reverse()
    return path
```

### 3. 修改 run_geometric_search 函数的主循环（约第285行开始）

**关键修改点：**

#### 3.1 初始化部分（约第292-305行）

**原代码：**
```python
    # 2. 连接起点和终点得到一个直线轨迹（初始轨迹）
    initial_path = GeometricPath([start_point, goal_point], 
                                 calculate_path_cost([start_point, goal_point]), 
                                 [0],
                                 -1)
    
    # 3. 将初始轨迹加入搜索集合 (使用优先队列，代价最低优先)
    search_set = [] # 优先队列 (min-heap)
    heapq.heappush(search_set, initial_path)
    
    # 用于记录所有被探索过的路径，以便回溯和避免重复，这里用列表索引模拟路径ID
    explored_paths = [initial_path]
    path_counter = 0
```

**修改为：**
```python
    # 2. 创建起点节点
    initial_g_cost = 0.0
    initial_h_cost = calculate_heuristic(start_point, goal_point)
    initial_node = GeometricPath(
        position=start_point,
        g_cost=initial_g_cost,
        h_cost=initial_h_cost,
        parent_node=None
    )
    
    # 3. 将初始节点加入搜索集合 (使用优先队列，f_cost最低优先)
    search_set = []  # 优先队列 (min-heap)
    heapq.heappush(search_set, initial_node)
    
    # 用于记录已访问的节点，避免重复访问
    visited = set()  # 存储已访问的位置 (x, y)
```

#### 3.2 主循环部分（约第312-350行）

**原代码：**
```python
        current_path = heapq.heappop(search_set)
        print("当前路径:")
        print(current_path.traj_points)
        # 5. 检测是否发生碰撞 (对于路径 P 的所有直线段)
        collided = False
        collision_info = None
        
        # P 是一个折线路径，由多个线段组成
        path_points = current_path.traj_points
        segments_to_check = []
        if visualize_level > 2:
            # 解包所有点的x和y坐标
            x_coords, y_coords = zip(*path_points)
            # plt.cla()
            plt.plot(x_coords, y_coords, linewidth=2.5, color='r', marker='o', markersize=4)
            plt.pause(1)
            # plt.cla()
        # 找出当前路径中发生碰撞的线段
        for i in range(len(path_points) - 1):
            p_start = path_points[i]
            p_end = path_points[i+1]
            print("A-B",p_start,p_end)
            print("collision_check_list",current_path.collision_check_list)
            if current_path.collision_check_list[i] != 2:# 有碰撞或未知，一般应该是未知
                
                is_collided, col_pt, max_overlap = line_collision_check_fast(p_start, p_end, mapParameters)
                if col_pt is not None and visualize_level > 2:
                    plt.plot(col_pt[0], col_pt[1], 'bo', markersize=6)
                    print("col_pt",col_pt)

                
                if is_collided:
                    current_path.collision_check_list[i] = 1
                    print("collision added")
                    collided = True
                    # 流程图要求：找到直线段和障碍物重合距离最大的 OB
                    # 我们选择第一个碰撞段作为当前迭代的修正目标
                    collision_info = {
                        'p_start': p_start,
                        'p_end': p_end,
                        'collision_point': col_pt,
                        'A': p_start, # 对应流程图的 A
                        'B': p_end, # 对应流程图的 B
                        'segment_index': i # 碰撞发生的线段索引
                    }
                    print("collision_check_list modified",current_path.collision_check_list)
                    break # 流程图隐含只修正第一个碰撞点
                else:
                    current_path.collision_check_list[i] = 2
                    print("no collision added")
            else:
                print("line no collision")
            print("collision_check_list modified",current_path.collision_check_list)
        
        if not collided:
            # 搜索成功，返回轨迹
            print(f"搜索成功！迭代次数: {iteration}")
            
            # 6. 平滑处理（可选，在流程图的平滑步骤）
            final_traj_x, final_traj_y = [], []
            for point in path_points:
                final_traj_x.append(point[0])
                final_traj_y.append(point[1])
            
            # 流程图的平滑操作（删除多余拐点等）在这里实现
            # 由于当前路径已经是折线，平滑通常是使用曲线拟合（如 B-Spline）
            # 为了简化，我们只返回折线，但标注此处为平滑步骤
            
            return final_traj_x, final_traj_y
```

**修改为：**
```python
        current_node = heapq.heappop(search_set)
        current_pos = current_node.position
        
        print(f"当前节点: {current_pos}, g={current_node.g_cost:.2f}, h={current_node.h_cost:.2f}, f={current_node.f_cost:.2f}")
        
        # 检查是否已访问过
        pos_key = (round(current_pos[0]), round(current_pos[1]))
        if pos_key in visited:
            continue
        visited.add(pos_key)
        
        # 5. 检测从当前节点到目标位置是否有碰撞
        is_collided, col_pt, _ = line_collision_check_fast(current_pos, goal_point, mapParameters)
        
        if visualize_level > 2:
            plt.plot([current_pos[0], goal_point[0]], [current_pos[1], goal_point[1]], 
                    linewidth=2, color='orange', alpha=0.5)
            if col_pt is not None:
                plt.plot(col_pt[0], col_pt[1], 'bo', markersize=6)
            plt.pause(0.1)
        
        if not is_collided:
            # 没有碰撞，成功到达目标
            print(f"搜索成功！迭代次数: {iteration}")
            
            # 创建目标节点
            goal_node = GeometricPath(
                position=goal_point,
                g_cost=current_node.g_cost + calculate_heuristic(current_pos, goal_point),
                h_cost=0.0,
                parent_node=current_node
            )
            
            # 重建路径
            path_points = reconstruct_path(goal_node)
            final_traj_x = [p[0] for p in path_points]
            final_traj_y = [p[1] for p in path_points]
            
            return final_traj_x, final_traj_y
```

#### 3.3 碰撞处理部分（约第362-400行）

**原代码：**
```python
        # 碰撞了，执行修正
        print('Collision detected')
        print('collision_point', collision_info['collision_point'])
        # 7. 找到 OB 距直线最远的点 L 和 R (左右各一个)
        A = collision_info['A']
        B = collision_info['B']
        # 寻找最远点时，使用碰撞点来限定搜索区域
        furthest_L, furthest_R = find_furthest_obstacle_point(collision_info['collision_point'], mapParameters, A, B)
        if visualize_level > 3:
            if furthest_L is not None:
                plt.plot(furthest_L[0], furthest_L[1], 'bs', markersize=4)
                print('furthest_L:', furthest_L)
            if furthest_R is not None:
                print('furthest_R:', furthest_R)
                plt.plot(furthest_R[0], furthest_R[1], 'rs', markersize=4)
        
        
        # 8. 利用端点 A, B，分别连接 L, R，得到两条折线 ALB 和 ARB
        
        if furthest_L is not None:
            # L 侧修正路径 (Path L): A -> L -> B

            # furthest_L = furthest_L + add_L
            # 创建新的路径点列表：[..., P[i], L, P[i+1], ...]
            new_points_L = path_points[:collision_info['segment_index'] + 1] # P[0]...P[i]=A  前半段
            new_points_L.append(furthest_L) # 插入 L
            new_points_L.extend(path_points[collision_info['segment_index'] + 1:]) # P[i+1]=B...P[end]  后半段
            print("new_points_L:", new_points_L)
            # 处理碰撞列表
            new_collision_check_list = current_path.collision_check_list[:collision_info['segment_index']]
            new_collision_check_list.append(0)#当前点
            new_collision_check_list.append(0)#新增点
            new_collision_check_list.extend(current_path.collision_check_list[collision_info['segment_index'] + 1:])

            cost_L = calculate_path_cost(new_points_L) + math.hypot(furthest_L[0]-goal[0], furthest_L[1]-goal[1])*0.5
            path_L = GeometricPath(new_points_L, cost_L, new_collision_check_list, current_path.parent_path_index)
            
            # 9. 更新新轨迹，计算代价，加入搜索集合
            heapq.heappush(search_set, path_L)
            explored_paths.append(path_L)
            path_counter += 1
            
        if furthest_R is not None:
            # R 侧修正路径 (Path R): A -> R -> B
            # furthest_R = furthest_R + add_R
            # 创建新的路径点列表：[..., P[i], R, P[i+1], ...]
            new_points_R = path_points[:collision_info['segment_index'] + 1] # P[0]...P[i]=A
            new_points_R.append(furthest_R) # 插入 R
            new_points_R.extend(path_points[collision_info['segment_index'] + 1:]) # P[i+1]=B...P[end]
            print("new_points_R:", new_points_R)
            new_collision_check_list = current_path.collision_check_list[:collision_info['segment_index']]
            new_collision_check_list.append(0)#当前点
            new_collision_check_list.append(0)#新增点
            new_collision_check_list.extend(current_path.collision_check_list[collision_info['segment_index'] + 1:])
            
            cost_R = calculate_path_cost(new_points_R) + math.hypot(furthest_R[0]-goal[0], furthest_R[1]-goal[1])*0.5
            path_R = GeometricPath(new_points_R, cost_R, new_collision_check_list, current_path.parent_path_index)
            
            # 9. 更新新轨迹，计算代价，加入搜索集合
            heapq.heappush(search_set, path_R)
            explored_paths.append(path_R)
            path_counter += 1
```

**修改为：**
```python
        # 碰撞了，执行修正
        print('Collision detected')
        print('collision_point', col_pt)
        
        # 7. 找到 OB 距直线最远的点 L 和 R (左右各一个)
        furthest_L, furthest_R = find_furthest_obstacle_point(col_pt, mapParameters, current_pos, goal_point)
        
        if visualize_level > 3:
            if furthest_L is not None:
                plt.plot(furthest_L[0], furthest_L[1], 'bs', markersize=8)
                print('furthest_L:', furthest_L)
            if furthest_R is not None:
                plt.plot(furthest_R[0], furthest_R[1], 'rs', markersize=8)
                print('furthest_R:', furthest_R)
            plt.pause(0.5)
        
        # 8. 创建左右两个节点并加入堆
        if furthest_L is not None:
            pos_L_key = (round(furthest_L[0]), round(furthest_L[1]))
            if pos_L_key not in visited:
                # 计算左节点的代价
                g_cost_L = current_node.g_cost + calculate_heuristic(current_pos, furthest_L)
                h_cost_L = calculate_heuristic(furthest_L, goal_point)
                
                left_node = GeometricPath(
                    position=furthest_L,
                    g_cost=g_cost_L,
                    h_cost=h_cost_L,
                    parent_node=current_node
                )
                
                heapq.heappush(search_set, left_node)
                print(f"添加左节点: {furthest_L}, g={g_cost_L:.2f}, h={h_cost_L:.2f}, f={left_node.f_cost:.2f}")
        
        if furthest_R is not None:
            pos_R_key = (round(furthest_R[0]), round(furthest_R[1]))
            if pos_R_key not in visited:
                # 计算右节点的代价
                g_cost_R = current_node.g_cost + calculate_heuristic(current_pos, furthest_R)
                h_cost_R = calculate_heuristic(furthest_R, goal_point)
                
                right_node = GeometricPath(
                    position=furthest_R,
                    g_cost=g_cost_R,
                    h_cost=h_cost_R,
                    parent_node=current_node
                )
                
                heapq.heappush(search_set, right_node)
                print(f"添加右节点: {furthest_R}, g={g_cost_R:.2f}, h={h_cost_R:.2f}, f={right_node.f_cost:.2f}")
```

## 总结

以上是**最小化修改**方案，主要修改集中在：

1. **GeometricPath类定义**：从存储完整路径改为存储单个节点
2. **添加两个辅助函数**：`calculate_heuristic` 和 `reconstruct_path`
3. **主循环的三个部分**：
   - 初始化：创建单个起点节点
   - 主循环：处理单个节点而非完整路径
   - 碰撞处理：只添加左右两个节点而非完整路径

核心思想是将"路径搜索"改为"节点搜索"，通过父节点指针在最后重建完整路径。
