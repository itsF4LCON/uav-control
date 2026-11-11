import io
import json
import math
import numpy as np

import config as C
from drone.world import World
from drone.flight import (
    set_selected_point,
    clear_selected_point,
    request_launch,
    request_land,
    request_kill,
    clear_land_request,
)
from godot_bridge import GodotBridge
from web_gcs import _GCSRequestHandler, _LATEST_PACKET


def assert_finite(w):
    assert np.all(np.isfinite(w.quad.pos))
    assert np.all(np.isfinite(w.quad.vel))
    assert np.all(np.isfinite(w.quad.att))
    assert np.all(np.isfinite(w.quad.rate))


def test_prelaunch_climb_and_loiter():
    clear_selected_point()
    clear_land_request()
    w = World(auto_launch=False)

    for _ in range(200):
        w.step()
        assert w.phase == "READY — PRESS START FLIGHT"
        assert np.allclose(w.quad.pos, [0.0, 0.0, 1.2])

    request_launch()
    w.step()
    assert not w.waiting_for_launch
    assert w.phase == "HAND-LAUNCH CLIMB"

    radii, alts = [], []
    for _ in range(int(20.0 / C.DT)):
        w.step()
        assert_finite(w)
        assert w.quad.pos[2] > 0.3
        if w.t > 8.0:
            radii.append(math.hypot(w.quad.pos[0], w.quad.pos[1]))
            alts.append(w.quad.pos[2])

    assert abs(np.mean(radii) - C.CIRCLE_RADIUS) < 0.5
    assert abs(np.mean(alts) - C.HOVER_ALT) < 0.2
    assert np.linalg.norm(w.ctrl.rate_des) > 0.0


def test_point_switching_and_wind():
    clear_selected_point()
    clear_land_request()
    old_mean, old_gust = C.WIND_MEAN, C.WIND_GUST_STD
    try:
        C.WIND_MEAN = (1.6, -1.0, 0.0)
        C.WIND_GUST_STD = 1.2
        w = World(auto_launch=True)
        for tx, ty, tz, tr, mode in [
            (15.0, 15.0, 3.0, 5.0, "loiter"),
            (-18.0, 10.0, 2.2, 3.0, "fly_to"),
            (0.0, 0.0, 2.2, 4.5, "loiter"),
        ]:
            set_selected_point(tx, ty, tz, radius=tr, mode=mode)
            for _ in range(int(5.0 / C.DT)):
                w.step()
                assert_finite(w)
                assert w.quad.pos[2] > 0.5
    finally:
        C.WIND_MEAN = old_mean
        C.WIND_GUST_STD = old_gust
        clear_selected_point()


def test_rtl_landing_kill_and_relaunch():
    clear_selected_point()
    clear_land_request()
    w = World(auto_launch=True)

    set_selected_point(20.0, 12.0, 2.2, radius=4.5, mode="fly_to")
    for _ in range(int(6.0 / C.DT)):
        w.step()

    request_land()
    for _ in range(int(35.0 / C.DT)):
        w.step()
        assert_finite(w)
        if w.disarmed:
            break

    assert w.disarmed
    assert math.hypot(w.quad.pos[0], w.quad.pos[1]) < 3.0

    request_launch()
    w.step()
    assert not w.disarmed and not w.waiting_for_launch

    request_kill()
    w.step()
    assert w.disarmed and w.phase == "DISARMED (KILL SWITCH)"
    clear_selected_point()
    clear_land_request()


def test_web_gcs():
    bridge = GodotBridge(web_port=None)
    try:
        w = World(auto_launch=False)
        w.step()
        bridge.send(w, False, -1)
        assert _LATEST_PACKET["phase"] == "READY — PRESS START FLIGHT"

        class DummyHandler(_GCSRequestHandler):
            def __init__(self, path, body=b""):
                self.path = path
                self.headers = {"Content-Length": str(len(body))}
                self.rfile = io.BytesIO(body)
                self.wfile = io.BytesIO()
                self.status_code = None

            def send_response(self, code, message=None):
                self.status_code = code

            def send_header(self, k, v):
                pass

            def end_headers(self):
                pass

        h_get = DummyHandler("/")
        h_get.do_GET()
        html = h_get.wfile.getvalue().decode("utf-8")
        assert h_get.status_code == 200
        assert "START FLIGHT" in html and "KILL" in html

        h_post = DummyHandler("/api/command", json.dumps({"cmd": "start_flight"}).encode("utf-8"))
        h_post.do_POST()
        w.step()
        assert not w.waiting_for_launch
    finally:
        bridge.close()


if __name__ == "__main__":
    test_prelaunch_climb_and_loiter()
    test_point_switching_and_wind()
    test_rtl_landing_kill_and_relaunch()
    test_web_gcs()
    print("ALL TESTS PASSED")
