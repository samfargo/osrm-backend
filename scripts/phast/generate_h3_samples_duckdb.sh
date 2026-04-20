#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  generate_h3_samples_duckdb.sh --output <samples.csv> [--bbox min_lon,min_lat,max_lon,max_lat] [--resolution 9]

Defaults:
  --bbox       7.4092,43.7233,7.4399,43.7519   (Monaco bounding box)
  --resolution 9

Output CSV columns:
  h3_id,res,sample_index,lon,lat

Sample set per cell:
  0 = center
  1 = vertex 0
  2 = vertex 3
EOF
}

OUTPUT_PATH=""
BBOX="7.4092,43.7233,7.4399,43.7519"
RESOLUTION="9"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output)
      OUTPUT_PATH="${2:-}"
      shift 2
      ;;
    --bbox)
      BBOX="${2:-}"
      shift 2
      ;;
    --resolution)
      RESOLUTION="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ -z "${OUTPUT_PATH}" ]]; then
  echo "--output is required" >&2
  usage
  exit 1
fi

IFS=',' read -r MIN_LON MIN_LAT MAX_LON MAX_LAT <<< "${BBOX}"
if [[ -z "${MIN_LON}" || -z "${MIN_LAT}" || -z "${MAX_LON}" || -z "${MAX_LAT}" ]]; then
  echo "Invalid --bbox. Expected: min_lon,min_lat,max_lon,max_lat" >&2
  exit 1
fi

if [[ ! "${RESOLUTION}" =~ ^[0-9]+$ ]]; then
  echo "--resolution must be an integer." >&2
  exit 1
fi

mkdir -p "$(dirname "${OUTPUT_PATH}")"
OUTPUT_PATH_SQL="${OUTPUT_PATH//\'/\'\'}"

duckdb -no-stdin -c "
INSTALL h3 FROM community;
LOAD h3;
COPY (
  WITH cells AS (
    SELECT
      unnest(
        h3_polygon_wkt_to_cells(
          'POLYGON((${MIN_LON} ${MIN_LAT},${MAX_LON} ${MIN_LAT},${MAX_LON} ${MAX_LAT},${MIN_LON} ${MAX_LAT},${MIN_LON} ${MIN_LAT}))',
          ${RESOLUTION}
        )
      ) AS h3
  )
  SELECT
    h3_h3_to_string(h3) AS h3_id,
    ${RESOLUTION}::INTEGER AS res,
    0 AS sample_index,
    h3_cell_to_lng(h3) AS lon,
    h3_cell_to_lat(h3) AS lat
  FROM cells
  UNION ALL
  SELECT
    h3_h3_to_string(h3) AS h3_id,
    ${RESOLUTION}::INTEGER AS res,
    1 AS sample_index,
    h3_vertex_to_lng(h3_cell_to_vertex(h3, 0)) AS lon,
    h3_vertex_to_lat(h3_cell_to_vertex(h3, 0)) AS lat
  FROM cells
  UNION ALL
  SELECT
    h3_h3_to_string(h3) AS h3_id,
    ${RESOLUTION}::INTEGER AS res,
    2 AS sample_index,
    h3_vertex_to_lng(h3_cell_to_vertex(h3, 3)) AS lon,
    h3_vertex_to_lat(h3_cell_to_vertex(h3, 3)) AS lat
  FROM cells
  ORDER BY h3_id, sample_index
) TO '${OUTPUT_PATH_SQL}' (HEADER, DELIMITER ',');
"

ROW_COUNT=$(($(wc -l < "${OUTPUT_PATH}") - 1))
CELL_COUNT=$((ROW_COUNT / 3))
echo "Wrote ${OUTPUT_PATH} (${CELL_COUNT} cells, ${ROW_COUNT} sample rows, r${RESOLUTION})."
