#!/usr/bin/env bash
# ============================================================
# go_home.sh — 一条命令把机械臂送回 home（尽量自动处理异常）
#
# 用法:
#   ./go_home.sh            # 默认 IP 172.16.0.2, 速度 0.12 rad/s
#   ./go_home.sh 172.16.0.2 0.10
#
# 行为:
#   1. 检查 FCI 端口 1337
#   2. 调 safe_goto_retry：从"当前实际位置"出发做余弦关节轨迹，
#      遇 reflex 自动 automaticErrorRecovery() 后重规划剩余行程（默认 8 次）
#   3. 外层再重试 2 轮，直到末端距 home < 1 cm
#   4. 失败时按日志里的错误类型，直接告诉你该去 Desk 点哪个按钮
#
# 它无法自动处理的（必须人工）:
#   - "User stopped"            → Desk 点 Resume，或 FCI OFF/ON
#   - reflex 清不掉              → Desk 点 Reset
#   - 急停按钮按下               → 先松开急停
#   - 关节 Locked（brakes）      → Desk 里解锁
# ============================================================
set -uo pipefail

IP="${1:-172.16.0.2}"
SPEED="${2:-0.12}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="${HERE}/../franka-safe/build"
TOL_RAD="0.01"    # safe_goto_retry 的关节容差（≈0.6°）
TOL_M="0.01"      # 我们认为"到家"的末端距离
OUTER=2

fail_hint() {
  local log="$1"
  echo
  echo "================ 没能自动回到 home ================"
  if grep -qi "User stopped" "$log"; then
    echo "原因: 机械臂处于 User stopped（Desk 侧停止）"
    echo "处理: (1) Desk 页面点 Resume / ▶"
    echo "      (2) FCI 开关 OFF → 等 3 秒 → ON"
    echo "      (3) Joints 拨 Locked 等 2 秒, 再拨回 Unlocked"
  elif grep -qi "brakes" "$log"; then
    echo "原因: 关节被锁（brakes locked）"
    echo "处理: Desk → Joints → Unlocked"
  elif grep -qi "reflex" "$log"; then
    echo "原因: 反复触发 reflex（撞到东西 / 负载过高 / 主机 CPU 被占满）"
    echo "处理: 清障; Desk 点 Reset; 关掉 Firefox 等占 CPU 的程序再试"
  elif grep -qiE "connection|network|refused|timed out" "$log"; then
    echo "原因: 网络 / FCI 连接问题"
    echo "处理: 查网线; ip a | grep 172.16; nc -zv $IP 1337"
  else
    echo "原因: 请看上面的日志尾部"
  fi
  echo "日志: $log"
  echo "==================================================="
  exit 1
}

echo "=== 1. 连接检查 ==="
if ! timeout 3 bash -c "echo > /dev/tcp/${IP}/1337" 2>/dev/null; then
  echo "  ❌ FCI 端口 1337 不通 ($IP)"
  fail_hint /dev/null
fi
echo "  ✅ FCI 端口通"

LOG="/tmp/go_home_$(date +%H%M%S).log"
echo "=== 2. 回 home（日志: $LOG） ==="

for ((round=1; round<=OUTER; round++)); do
  echo "  --- 第 $round 轮 ---"
  chrt -f 80 taskset -c 14 "$BIN/safe_goto_retry" \
      "$IP" home "$SPEED" 8 "$TOL_RAD" \
      < /dev/null >>"$LOG" 2>&1
  grep -E "尝试|走完|中断|恢复|异常|容差" "$LOG" | tail -5

  DIST=$(timeout 20 "$BIN/safe_fk" "$IP" home 2>/dev/null \
         | awk '/末端直线距离/{print $2; exit}')
  if [[ -n "${DIST:-}" ]] && awk "BEGIN{exit !($DIST < $TOL_M)}"; then
    echo
    echo "✅ 已回到 home（末端误差 ${DIST} m）"
    exit 0
  fi
  echo "  仍在 ${DIST:-?} m，重试..."
  sleep 1
done

fail_hint "$LOG"
