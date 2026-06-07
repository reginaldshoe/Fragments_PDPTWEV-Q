"""Non-Gurobi validation for integrated helper modules."""
from __future__ import annotations
import json
from evrp_fragments.callback.route_tools import extract_routes_from_solution, stitch_sid_sequence

def main() -> int:
    arc_by_id = {0:{'Start':'D0','End':'C1','seq':('D0','C1'),'start_onboard':frozenset(),'end_onboard':frozenset({'C1'})},1:{'Start':'C1','End':'D1','seq':('C1','D1'),'start_onboard':frozenset({'C1'}),'end_onboard':frozenset()},2:{'Start':'D1','End':'D0','seq':('D1','D0'),'start_onboard':frozenset(),'end_onboard':frozenset()}}
    routes = extract_routes_from_solution([0,1,2], arc_by_id); stitched = [stitch_sid_sequence(r, arc_by_id) for r in routes]
    print(json.dumps({'routes': routes, 'stitched': stitched}, indent=2)); assert stitched == [['D0','C1','D1','D0']]
    return 0
if __name__ == '__main__': raise SystemExit(main())
