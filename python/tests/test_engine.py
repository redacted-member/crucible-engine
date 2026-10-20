# SPDX-License-Identifier: MIT
"""Tests for the crucible package. Run from the repository root: PYTHONPATH=python python3 -m unittest discover python/tests"""

import json
import unittest

import numpy as np

from crucible_engine import CrucibleError, Engine


def play(engine: Engine, seconds: float = 0.3) -> np.ndarray:
    engine.note_on(60, 0.8)
    return engine.render(seconds)


class RenderTests(unittest.TestCase):
    def test_render_is_non_silent_and_shaped(self) -> None:
        with Engine() as engine:
            audio = play(engine)
        self.assertEqual(audio.shape, (int(round(0.3 * 48000)), 2))
        self.assertEqual(audio.dtype, np.float32)
        self.assertGreater(float(np.abs(audio).max()), 0.01)
        self.assertTrue(np.isfinite(audio).all())

    def test_render_is_deterministic(self) -> None:
        with Engine(seed=7) as a, Engine(seed=7) as b:
            np.testing.assert_array_equal(play(a), play(b))

    def test_render_needs_exactly_one_length(self) -> None:
        with Engine() as engine:
            with self.assertRaises(ValueError):
                engine.render()
            with self.assertRaises(ValueError):
                engine.render(1.0, samples=10)

    def test_closed_engine_raises(self) -> None:
        engine = Engine()
        engine.close()
        engine.close()
        with self.assertRaises(CrucibleError):
            engine.render(0.01)


class ParameterTests(unittest.TestCase):
    def test_text_setter_round_trips(self) -> None:
        with Engine() as engine:
            engine.set_param_text("filter_cutoff", "1000 Hz")
            text = engine.param_text("filter_cutoff")
            value = engine.get_param("filter_cutoff")
            self.assertAlmostEqual(value, 1000.0, delta=10.0)
            engine.set_param_text("filter_cutoff", text)
            self.assertAlmostEqual(engine.get_param("filter_cutoff"), value, delta=value * 0.01)

    def test_unknown_parameter(self) -> None:
        with Engine() as engine:
            with self.assertRaises(KeyError):
                engine.set_param("no_such_param", 1.0)
            with self.assertRaises(CrucibleError):
                engine.set_param_text("filter_cutoff", "not a number")


class PatchTests(unittest.TestCase):
    def test_patch_round_trip(self) -> None:
        with Engine() as source:
            source.set_param("filter_cutoff", 777.0)
            patch = source.save_patch("test")
            reference = play(source)
        self.assertEqual(json.loads(patch)["meta"]["name"], "test")
        with Engine() as target:
            target.load_patch(patch)
            self.assertAlmostEqual(target.get_param("filter_cutoff"), 777.0, delta=1.0)
            np.testing.assert_array_equal(play(target), reference)

    def test_bad_patch_raises(self) -> None:
        with Engine() as engine:
            with self.assertRaises(CrucibleError):
                engine.load_patch("{not json")

    def test_describe_lists_parameters(self) -> None:
        with Engine() as engine:
            description = engine.describe()
            self.assertEqual(len(description["params"]), engine.param_count)


class ProductTests(unittest.TestCase):
    def test_bad_product_reports_a_path(self) -> None:
        bad = {"format": "crucible-product", "version": 1, "id": "t", "name": "T", "voice": "none",
               "fx": {"instances": [{"id": "x", "type": "fx.does_not_exist"}], "lanes": []}}
        with self.assertRaises(CrucibleError) as caught:
            Engine.from_product(bad)
        message = str(caught.exception)
        self.assertIn("fx", message)
        self.assertIn("fx.does_not_exist", message)

    def test_malformed_product_json(self) -> None:
        with self.assertRaises(CrucibleError):
            Engine.from_product("{not json")


class ResetTests(unittest.TestCase):
    def test_reset_silences_a_reverb_tail(self) -> None:
        with Engine() as engine:
            engine.set_param("fx_ra_on", 1.0)
            engine.set_param("fx_ra_mix", 1.0)
            engine.set_param("fx_ra_decay", engine.describe()["params"][engine.param_index("fx_ra_decay")]["max"])
            engine.note_on(60, 0.9)
            engine.render(0.3)
            self.assertGreater(engine.tail_seconds, 0.5)
            engine.all_notes_off()
            tail = engine.render(0.3)
            self.assertGreater(float(np.abs(tail).max()), 1e-4)
            engine.reset()
            after = engine.render(0.3)
            # The algorithmic reverb keeps an anti-denormal offset of about 1e-19 (-370 dB).
            self.assertLess(float(np.abs(after).max()), 1e-12)

    def test_latency_is_a_non_negative_int(self) -> None:
        with Engine() as engine:
            self.assertGreaterEqual(engine.latency_samples, 0)

    def test_routes_read_back_in_position_order(self) -> None:
        with Engine() as engine:
            self.assertEqual(engine.routes(), [])
            self.assertTrue(engine.add_route(1074, "filter_cutoff", 0.5))
            self.assertTrue(engine.add_route(1001, "filter_res", 0.25))
            self.assertTrue(engine.add_route(1074, "filter_cutoff", -0.5))
            routes = engine.routes()
            self.assertEqual([(r["source"], r["target"]) for r in routes], [(1074, "filter_cutoff"), (1001, "filter_res")])
            self.assertEqual(routes[0]["amount"], -0.5)
            self.assertTrue(engine.remove_route(1074, "filter_cutoff"))
            self.assertEqual([r["target"] for r in engine.routes()], ["filter_res"])


if __name__ == "__main__":
    unittest.main()
