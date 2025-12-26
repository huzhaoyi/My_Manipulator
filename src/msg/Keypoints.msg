std_msgs/Header header

bool has_target  # 是否检测到目标,如果为false,则后面的数据都不可用

geometry_msgs/Point[2] corner_points # 目标检测框左上角和右下角点

geometry_msgs/Point[] keypoints #数组类型，存放检测到的关键点信息[x,y,z]
float32[] angles  #数组类型,存放每个关键点对应的角度信息theta(度)

float32 score  # 目标框置信度

float32 score_threshold # 当前的目标框置信度阈值
bool use_score_threshold  # 是否启用阈值过滤
bool is_available  # 是否可用
