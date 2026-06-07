from evrp_fragments.callback.route_tools import extract_routes_from_solution, stitch_sid_sequence

def test_stitching():
    arc_by_id={0:{'Start':'D0','End':'C1','seq':('D0','C1')},1:{'Start':'C1','End':'D0','seq':('C1','D0')}}
    routes=extract_routes_from_solution([0,1], arc_by_id)
    assert routes == [[0,1]]
    assert stitch_sid_sequence(routes[0], arc_by_id) == ['D0','C1','D0']
