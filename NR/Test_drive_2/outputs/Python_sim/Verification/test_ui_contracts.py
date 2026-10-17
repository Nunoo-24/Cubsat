"""Source-level checks for HTML controls referenced by inline JavaScript.

These checks catch missing/duplicated DOM IDs; they do not execute or render a
browser UI.
"""

from html.parser import HTMLParser
from pathlib import Path
import re
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class _IdCollector(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = []

    def handle_starttag(self, _tag, attrs):
        self.ids.extend(value for key, value in attrs if key == "id")


class BrowserUiContractTests(unittest.TestCase):
    def assert_ui_contract(self, relative_path, handler_relative_path):
        source = (PROJECT_ROOT / relative_path).read_text(encoding="utf-8")
        handler_source = (PROJECT_ROOT / handler_relative_path).read_text(
            encoding="utf-8")
        parser = _IdCollector()
        parser.feed(source)
        ids = set(parser.ids)
        duplicate_ids = sorted({item for item in parser.ids
                                if parser.ids.count(item) > 1})
        references = set(re.findall(
            r"getElementById\(['\"]([^'\"]+)['\"]\)", source
        ))
        self.assertEqual(duplicate_ids, [], "duplicate HTML element IDs")
        self.assertEqual(sorted(references - ids), [],
                         "JavaScript refers to missing HTML element IDs")
        ui_routes = set(re.findall(r"['\"](/api/[^'\"?`]+)", source))
        missing_routes = sorted(route for route in ui_routes
                                if route not in handler_source)
        self.assertEqual(missing_routes, [],
                         "UI refers to an endpoint absent from its handler")

    def test_simulation_environment_ui_dom_ids(self):
        self.assert_ui_contract("Sim_Environment/ui.html",
                                "Sim_Environment/main.py")

    def test_ground_station_ui_dom_ids(self):
        self.assert_ui_contract("Ground_Station/ui.html",
                                "Ground_Station/main.py")


if __name__ == "__main__":
    unittest.main()
