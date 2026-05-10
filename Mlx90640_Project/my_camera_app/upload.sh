#!/bin/bash

# 1. 检查是否输入了说明（也就是脚本的第一个参数 $1）
if [ -z "$1" ]; then
    # 如果没输入说明，给一个默认提示
    echo "❌ 错误：请提供上传说明！"
    echo "用法示例: ./upload.sh \"更新了某某驱动\""
    exit 1
fi

# 把输入的说明存到变量里
COMMIT_MSG="$1"

echo "================================="
echo "🚀 开始自动化推送到 GitHub..."
echo "================================="

echo "▶ 1/4 查看当前状态..."
git status -s

echo "▶ 2/4 添加所有修改到暂存区..."
git add .

echo "▶ 3/4 提交说明: [ $COMMIT_MSG ]..."
git commit -m "$COMMIT_MSG"

# 这里的 main 如果你之前用的是 master，记得改成 master
echo "▶ 4/4 推送到云端..."
git push origin main

echo "================================="
echo "✅ 上传完成！"
echo "================================="