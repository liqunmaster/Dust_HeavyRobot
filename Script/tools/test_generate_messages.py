import importlib.util
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).with_name("generate_messages.py")
SPEC = importlib.util.spec_from_file_location("generate_messages", SCRIPT)
GENERATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GENERATOR)


class MessageGeneratorTests(unittest.TestCase):
    def test_parses_fields_and_fixed_arrays(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "sample.msg"
            source.write_text("# @struct Sample\nuint8[8] data\nfloat32 value\n", encoding="utf-8")
            name, fields = GENERATOR.parse_message(source)
            header = GENERATOR.render_header(source, name, fields)
            self.assertIn("struct Sample {", header)
            self.assertIn("uint8_t data[8];", header)
            self.assertIn("float value;", header)

    def test_rejects_invalid_or_duplicate_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "sample.msg"
            for content in (
                "# @struct Sample\nuint8 value\nuint8 value\n",
                "# @struct Sample\nuint8[0] value\n",
                "# @struct Sample\ncustom value\n",
                "# @struct Sample\n# @unknown value\nuint8 value\n",
            ):
                source.write_text(content, encoding="utf-8")
                with self.subTest(content=content), self.assertRaises(ValueError):
                    GENERATOR.parse_message(source)


if __name__ == "__main__":
    unittest.main()
