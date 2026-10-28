#!/bin/bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
astra_learn/consolidate.sh --from v001 --episode astra_rov_g3_20261026_001
astra_learn/consolidate.sh --from v002 --episode astra_rov_backup_g3_20261026_001
astra_learn/consolidate.sh --from v003 --episode astra_rov_backup_g3_20261026_002
astra_learn/consolidate.sh --from v004 --episode astra_rov_backup_g3_20261026_003
astra_learn/consolidate.sh --from v005 --episode astra_rov_backup2_g3_20261026_001
