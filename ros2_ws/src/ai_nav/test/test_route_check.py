import os

from ai_nav.plan_logic import load_places
from ai_nav.route_check import RouteChecker

PKG = os.path.join(os.path.dirname(__file__), '..')


def checker(world):
    places = load_places(os.path.join(PKG, 'config', f'places_{world}.yaml'))
    return RouteChecker(os.path.join(PKG, 'maps', f'{world}.yaml'), places)


def test_all_places_reachable():
    for world in ('warehouse', 'field'):
        rc = checker(world)
        start = next(iter(rc.places))
        for pid in rc.places:
            path, _ = rc.shortest_path(start, pid)
            assert path is not None, (world, pid)


def test_route_lengths_are_ordered():
    rc = checker('warehouse')
    aisles = ['south_aisle', 'middle_aisle', 'north_aisle']
    lengths = [rc.shortest_path('receiving', 'shipping', avoid=[x for x in aisles if x != a])[1]
               for a in aisles]
    assert lengths == sorted(lengths)


def test_direct_path_uses_south_aisle():
    rc = checker('warehouse')
    path, _ = rc.shortest_path('receiving', 'shipping')
    assert rc.aisles_on(path) == {'south_aisle'}


def test_detours():
    rc = checker('warehouse')
    assert rc.suggest_detour('receiving', 'shipping', set()) == ['shipping']
    assert rc.suggest_detour('receiving', 'shipping', {'south_aisle'}) == ['middle_aisle', 'shipping']
    assert rc.suggest_detour((2.95, 1.05), 'shipping',
                             {'south_aisle', 'middle_aisle'}) == ['north_aisle', 'shipping']
    assert rc.suggest_detour('receiving', 'shipping',
                             {'south_aisle', 'middle_aisle', 'north_aisle'}) is None


def test_same_hall_moves_need_no_aisle():
    rc = checker('warehouse')
    problems, _ = rc.check_plan('receiving', ['office'], {'south_aisle', 'middle_aisle', 'north_aisle'})
    assert problems == []
