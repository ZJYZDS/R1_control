"""
executor 的 Docstring当前配置 zero_point.yaw = 0.0，所以实际上旋转矩阵是单位矩阵，(gx, gy) = (zp.x
  + local_x, zp.y + local_y)。如果日后场地坐标系与地图坐标系有旋转，修改 yaw
  即可。

  坐标轴约定

  - x 轴向右为正，y 轴向上为正 (r1_graph_config.yaml:82)
  - 朝向 0° = +y 轴（即 atan2(dx, dy) 以 dy 为第二参数）
  - dir_correct=1 表示逆时针为正

  ---
  2. 航点生成的两个阶段

  阶段一：route_to_waypoints() (pipeline/graph_planner.py:204-266)

  将图规划得到的 route（节点序列）转换为相对航点列表 [(type, dx, dy, dyaw,
  name), ...]：

  ┌───────────┬──────────────────────┐
  │   type    │         含义         │
  ├───────────┼──────────────────────┤
  │ rotate    │ 原地旋转 dtheta 弧度 │
  ├───────────┼──────────────────────┤
  │ translate │ 平移 (dx, dy) 米     │
  └───────────┴──────────────────────┘

  关键逻辑：
  - 起点 K 节点：如果朝向与路径方向不同，先插入 rotate 航点。dtheta = (heading -
   start_yaw) * dir_correct，其中 heading = atan2(next_y - cur_y, next_x -      
  cur_x)                                                                   
  - 中间 K 节点：先到达（translate），如果需要转向则插入 rotate（dtheta =
  前后两段 heading 之差 * dir_correct）                                  
  - K→TGT 段：如果 TGT 在 tgt_facing                                            
  中有定义，使用预定义的梅林朝向而非路径几何方向
  - 末尾 K 节点：最终朝向由 target_end_yaw 决定                                 
                                                                  
  阶段二：route_to_absolute_waypoints() (pipeline/orchestrator.py:27-78)        
                                                                                
  将 route 转为 zero_point 坐标系下的绝对位姿 [{"x", "y", "theta", "name",      
  "height?"}, ...]：                                                            
                                                                                
  - x, y 直接使用 route 节点在 zero_point 系下的坐标（即 id_to_coord /          
  known_coords 的值）
  - theta = (heading_rad - ref_yaw) * dir_correct，归一化到 [-π, π]             
  - heading_rad 的算法：                                                        
    - TGT 节点 → 用 tgt_facing[id]（预定义的面向场内方向）                      
    - K→TGT → 如果 TGT 有 tgt_facing，用 TGT 的朝向                             
    - K→K → atan2(dx, dy)（+y=0 的约定）                                        
    - 末尾 K → 直接用 target_end_yaw                                            
                                                                                
  ---                                                                           
  3. 航点执行（两阶段控制）                                                     
                                                                                
  WaypointExecutor (executor/waypoint_executor.py) 收到绝对航点后：
                                                                                
  坐标修正 (line 386-394)                                                       
                                                                                
  从 LiDAR odom 变换到底盘中心：                                                
  cx = position.x - lidar_xoffset*cos(yaw) + lidar_yoffset*sin(yaw)
                 - offset_imu[0]*cos(yaw) + offset_imu[1]*sin(yaw)              
  cy = position.y - lidar_xoffset*sin(yaw) - lidar_yoffset*cos(yaw)             
                 - offset_imu[0]*sin(yaw) - offset_imu[1]*cos(yaw)              
                                                                                
  两阶段运动                                                                    
                                                                                
  ┌───────────┬──────────────────────┬──────────────────────────────────────┐   
  │   阶段    │       发送内容       │               完成条件               │   
  ├───────────┼──────────────────────┼──────────────────────────────────────┤   
  │ ROTATE    │ dtheta（其他=0）     │ angle_err < angle_thre (0.06 rad)    │
  ├───────────┼──────────────────────┼──────────────────────────────────────┤   
  │ TRANSLATE │ dx_cm,               │ dist_err < dist_thre (0.03 m) 且持续 │   
  │           │ dy_cm（dtheta=0）    │  hold_duration (1.0s)                │   
  └───────────┴──────────────────────┴──────────────────────────────────────┘   
                                                                                
  到达后：                                                                      
  - 普通航点 → 直接 _advance() 到下一个
  - TGT 航点 → 先发 TGT 串口握手帧 [0x0A, height_byte, 0x0B]，等待回应 [0xA1,   
  0x01, 0xB1] 后再推进                                                        
                                                                                
  超时保护                                                        
                                                                                
  单航点超时 waypoint_timeout = 10s，超时后发布 fault 并强制推进。              
                                                                                
  ---                                                                           
  4. 旋转角度计算的关键参数                                       
                                                                                
  ┌──────────────┬────────────────┬────────────────┬───────────────────────┐
  │     参数     │      红方      │      蓝方      │         含义          │    
  ├──────────────┼────────────────┼────────────────┼───────────────────────┤    
  │ ref_yaw      │ 0.0            │ 0.0            │ 参考零朝向（+y 轴）   │    
  ├──────────────┼────────────────┼────────────────┼───────────────────────┤    
  │ dir_correct  │ 1              │ 1              │ 旋转方向（1=逆时针为  │    
  │              │                │                │ 正）                  │    
  ├──────────────┼────────────────┼────────────────┼───────────────────────┤    
  │ start_yaw    │ (0-ref_yaw)*di │ (0-ref_yaw)*di │ 起始朝向              │    
  │              │ r=0            │ r=0            │                       │
  ├──────────────┼────────────────┼────────────────┼───────────────────────┤    
  │ target_end_y │ 0              │ 0              │ 终点朝向              │
  │ aw           │                │                │                       │    
  └──────────────┴────────────────┴────────────────┴───────────────────────┘
                                                                                
  tgt_facing 中各个 TGT ID 的朝向是用 ref_yaw ± n*pi/2*dir_correct              
  表达式定义的，表示到达该 TGT 后面向场内（梅林方向）。
                                                                                
  ---                                        

"""

"""


"""
