from exporterV2.core.mesh_geometry import build_open_tube_topology


def test_open_tube_topology_preserves_ring_winding():
    counts, indices = build_open_tube_topology(2, 3)

    assert counts == [3, 3, 3, 3, 3, 3]
    assert indices == [
        0,
        3,
        4,
        0,
        4,
        1,
        1,
        4,
        5,
        1,
        5,
        2,
        2,
        5,
        3,
        2,
        3,
        0,
    ]


def test_open_tube_topology_has_no_faces_for_one_ring():
    assert build_open_tube_topology(1, 8) == ([], [])
