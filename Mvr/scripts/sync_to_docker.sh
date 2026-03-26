#!/bin/bash

# ================= 配置区 =================
# 请将下面这两个路径替换为你实际的绝对路径
LOCAL_PATH="/home/jf/lab/3_my_projects/Project04_xiaomibei/Mvr"
CONTAINER_PATH="/home/Mvr"
CONTAINER_NAME="cyberdog_sim_run"
# ==========================================

echo "🚀 准备将代码同步至容器: $CONTAINER_NAME"
echo "📂 本机路径: $LOCAL_PATH"
echo "🎯 容器路径: $CONTAINER_PATH"
echo "----------------------------------------"

# 检查本机目录是否存在
if [ ! -d "$LOCAL_PATH" ]; then
  echo "❌ 错误: 找不到本机目录 $LOCAL_PATH"
  exit 1
fi

# 核心同步命令：打包 -> 过滤 __pycache__ -> 传输到容器 -> 解压
# 这个操作比 docker cp 更智能，且不会产生任何临时文件
tar -cf - --exclude='__pycache__' --exclude='*.pyc' -C "$LOCAL_PATH" . | docker exec -i "$CONTAINER_NAME" tar -xf - -C "$CONTAINER_PATH"

# 检查上一条命令是否执行成功
if [ $? -eq 0 ]; then
  echo "✅ 同步成功！(__pycache__ 已被自动拦截)"
else
  echo "❌ 同步失败，请检查容器是否正在运行，或者容器内的目标路径是否存在。"
fi
