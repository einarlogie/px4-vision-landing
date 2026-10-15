#!/usr/bin/env bash
# Run repeated precision landings, each on a freshly restarted simulator,
# appending one row per run to results/landing_runs.csv.
#
# usage (from the repo root in WSL):
#   tools/batch_landings.sh <first_seed> <last_seed> [extra precision_land.py args]
# e.g.
#   tools/batch_landings.sh 1 10 --random-offset 2.0
#   tools/batch_landings.sh 11 13 --random-offset 2.0 --occlusion 5,3
set -uo pipefail
first=$1; last=$2; shift 2
export GUI=${GUI:-0}

wait_ready() {
  for _ in $(seq 1 120); do
    docker compose logs sim 2>/dev/null | grep -q "home set" && return 0
    sleep 1
  done
  return 1
}

for seed in $(seq "$first" "$last"); do
  echo "=== seed $seed: restarting simulator"
  docker compose down -t 2 >/dev/null 2>&1
  docker compose up -d >/dev/null 2>&1
  if ! wait_ready; then echo "seed $seed: simulator did not start"; continue; fi
  timeout 600 docker compose exec -T app python scripts/precision_land.py \
      --seed "$seed" --run-id "seed$seed" "$@" 2>&1 | grep -E "RESULT|ERROR|Traceback|Error"
  if docker compose ps -a --format '{{.Service}} {{.Status}}' | grep -q "sim Exited"; then
    echo "seed $seed: simulator crashed"; docker compose logs sim --tail 5
  fi
done
docker compose down -t 2 >/dev/null 2>&1
