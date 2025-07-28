int32 island_id

# uint8 STATUS_NEW      = 0   # 新发现的岛屿
# uint8 STATUS_UPDATED  = 1   # 形状/大小更新的岛屿
# uint8 STATUS_MERGED   = 2   # 由其他岛屿融合而成的岛屿

# uint8 status

geometry_msgs/Point bmin
geometry_msgs/Point bmax

int32[] merged_ids          # 记录被融合进这个岛屿的旧ID列表