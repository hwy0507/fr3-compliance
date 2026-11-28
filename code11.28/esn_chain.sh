#!/usr/bin/env bash
# ============================================================
# esn_chain.sh — 用 ESN 依次走过任意多个姿态（A→B→C→…）
#
# 用法:
#   ./esn_chain.sh <IP> <姿态1> <姿态2> ... [峰值速度rad/s] [段间停顿s]
# 例:
#   ./esn_chain.sh 172.16.0.2 home pos2 pos3 home 0.15
#   ./esn_chain.sh 172.16.0.2 home pos2 pos3 home 0.15 1.0
#
# 说明:
#   每一段单独调用一次 esn_yield：
#     - 该程序会先（位置控制）走到本段起点，再用 ESN 走到终点
#     - 段与段首尾相接，所以"先走到起点"基本是空操作
#     - 每段重新做外力零偏校准
#   ESN_OFF=1 可关闭模型（用于对照）
#   ESN_BIN=esn_velocity 可切换底层（默认 esn_yield：关节阻抗）
# ============================================================
set -uo pipefail

if [[ $# -lt 3 ]]; then
  echo "用法: $0 <IP> <姿态1> <姿态2> ... [速度rad/s] [停顿s]"
  echo "例:   $0 172.16.0.2 home pos2 pos3 home 0.15"
  exit 1
fi

IP="$1"; shift
POSES=(); SPEED=""; DWELL="0"
for a in "$@"; do
  if [[ "$a" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    if [[ -z "$SPEED" ]]; then SPEED="$a"; else DWELL="$a"; fi
  else
    POSES+=("$a")
  fi
done
SPEED="${SPEED:-0.15}"

if [[ ${#POSES[@]} -lt 2 ]]; then
  echo "至少需要两个姿态"; exit 1
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="${ESN_BIN:-${HERE}/../franka-safe/build/esn_yield}"
[[ "$BIN" != /* ]] && BIN="${HERE}/../franka-safe/build/${BIN}"
[[ -x "$BIN" ]] || { echo "找不到 $BIN，请先编译"; exit 1; }

echo "链路: ${POSES[*]}   速度 ${SPEED} rad/s   段间停顿 ${DWELL}s"
echo "模型: $([[ -n "${ESN_OFF:-}" ]] && echo 关闭 || echo 开启)"

n=$(( ${#POSES[@]} - 1 ))
for ((i=0; i<n; i++)); do
  from="${POSES[$i]}"; to="${POSES[$((i+1))]}"
  echo
  echo "=========================================="
  echo " 第 $((i+1))/$n 段: ${from} → ${to}"
  echo "=========================================="
  if [[ -n "${ESN_OFF:-}" ]]; then
    ESN_OFF="$ESN_OFF" chrt -f 80 taskset -c 14 "$BIN" "$IP" "$from" "$to" "$SPEED" high
  else
    chrt -f 80 taskset -c 14 "$BIN" "$IP" "$from" "$to" "$SPEED" high
  fi
  rc=$?
  if [[ $rc -ne 0 ]]; then
    echo "!! 第 $((i+1)) 段失败（退出码 $rc），中止"
    exit $rc
  fi
  if (( i < n-1 )); then
    sleep "$DWELL"
  fi
done

echo
echo "链路完成: ${POSES[*]}"
