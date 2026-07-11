#!/usr/bin/env bash
# E2E 重复统计:抓取/任务成功率与耗时分布(对标赛题"复杂物体抓取成功率>=90%")
# 用法: source install/setup.bash && bash benchmarks/e2e_repeat_stats.sh [-n 次数]
# 每轮前清理 gzserver 残留(已知坑);输出逐轮结果与汇总到 benchmarks/results/
set -u
N=5
while getopts "n:" opt; do case $opt in n) N=$OPTARG;; *) exit 1;; esac; done

WS="$(cd "$(dirname "$0")/.." && pwd)"
STAMP=$(date +%Y%m%d-%H%M%S)
OUT_DIR="$WS/benchmarks/results"
mkdir -p "$OUT_DIR"
REPORT="$OUT_DIR/e2e_repeat_${STAMP}.md"
PASS=0

{
  echo "# 诚实 E2E 重复统计 $STAMP"
  echo "- 轮数: $N | 单轮预算: 测试内 420s + 启动"
  echo ""
  echo "| 轮 | 结果 | 耗时(s) | 备注 |"
  echo "|---|---|---|---|"
} > "$REPORT"

for i in $(seq 1 "$N"); do
  pkill -9 -f 'gzserver|gzclient' 2>/dev/null; sleep 3
  T0=$(date +%s)
  LOG="$OUT_DIR/e2e_run_${STAMP}_$i.log"
  if python3 -m pytest "$WS/src/lab_cobot_bringup/test/test_honest_e2e_launch.py" \
       -p no:anyio -q > "$LOG" 2>&1; then
    RES="PASS"; PASS=$((PASS+1)); NOTE=""
  else
    RES="FAIL"
    # 摘要失败原因(statuses 或断言首行)
    NOTE=$(grep -oE "statuses=\[[^]]*\]|AssertionError[^\"]{0,80}" "$LOG" | head -1 | tr '|' '/')
  fi
  DT=$(( $(date +%s) - T0 ))
  echo "| $i | $RES | $DT | $NOTE |" >> "$REPORT"
  echo "[$i/$N] $RES (${DT}s) $NOTE"
done
pkill -9 -f 'gzserver|gzclient' 2>/dev/null

{
  echo ""
  echo "## 汇总"
  echo "- **成功率: $PASS/$N = $(python3 -c "print(f'{$PASS/$N*100:.1f}')")%**(赛题指标: 抓取成功率>=90%)"
  echo "- 口径: 任务级成功(NAV->DETECT->PICK->NAV->PLACE->HOME->DONE 全链,含抓取与放置落点断言)"
  echo "- 单轮日志: e2e_run_${STAMP}_*.log"
} >> "$REPORT"

echo ""
echo "成功率 $PASS/$N  报告: $REPORT"
[ "$PASS" -eq "$N" ]
