#!/usr/bin/env bash
# 사용법: tools/new_week.sh <주차번호> [멤버폴더]
#   예) tools/new_week.sh 3                 -> members/park-jongjin/week03/log.md
#       tools/new_week.sh 3 ham-taehoon     -> members/ham-taehoon/week03/log.md
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "사용법: $0 <주차번호> [멤버폴더(기본: park-jongjin)]" >&2
  exit 1
fi

root="$(cd "$(dirname "$0")/.." && pwd)"
nn=$(printf "%02d" "$((10#$1))")
member="${2:-park-jongjin}"
dir="$root/members/$member/week$nn"

case "$((10#$nn))" in
  2) gate="SRR (×2)";; 6) gate="PDR (×2)";; 12) gate="CDR (×2)";; 16) gate="모듈검증 (×2)";;
  20) gate="조립완료 (×2)";; 24) gate="통합 (×2)";; 30) gate="TRR (×2)";; 32) gate="최종 (×2)";;
  *) gate="해당 없음";;
esac

case "$member" in
  park-jongjin) name="박종진";; ham-taehoon) name="함태훈";; *) name="$member";;
esac

if [ -e "$dir/log.md" ]; then
  echo "이미 존재: $dir/log.md" >&2
  exit 1
fi

mkdir -p "$dir/img"
sed -e "s/weekNN/week$nn/g" -e "s/제 NN 주차/제 $nn 주차/" -e "s/<이름>/$name/" \
    -e "s/해당 없음 \/ <게이트명> (×2)/$gate/" \
    "$root/templates/contribution_log.md" > "$dir/log.md"
touch "$dir/img/.gitkeep"

echo "생성: $dir/log.md  (게이트: $gate)"
echo "제출 시:  git tag -a week$nn -m \"week$nn: <요약>\" && git push origin week$nn"
