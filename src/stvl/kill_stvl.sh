#!/bin/bash
# 1. 先降级到 inactive
ros2 lifecycle set /costmap/costmap deactivate

# 2. 再清理资源
ros2 lifecycle set /costmap/costmap cleanup

# 3. 最后关机
ros2 lifecycle set /costmap/costmap shutdown
