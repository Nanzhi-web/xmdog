# --- 通用对齐参数 (Default / Stage 3) ---
bool line_found
float32 line_center_offset

# --- Stage 2 专用 (寻球) ---
bool line_is_horizontal
bool ball_found
string ball_color
float32 ball_x_offset
float32 ball_size_ratio

# --- Stage 3 专用 (综合避障与目标) ---
bool height_limit_danger
bool obstacle_danger
float32 obstacle_distance
bool target_found
string target_type
float32 target_distance
