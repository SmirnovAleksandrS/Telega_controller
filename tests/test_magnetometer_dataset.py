from __future__ import annotations

import os
import tempfile
import unittest

from app.magnetometer_dataset import Dataset, SampleRecord


class MagnetometerDatasetTests(unittest.TestCase):
    def test_trim_keeps_inclusive_selected_range(self) -> None:
        dataset = Dataset("trim_case")
        dataset.extend([
            SampleRecord("s", "raw", "Raw", "builtin", 10, 20, 21, 1.0, 0.0, 0.0, 90.0, ""),
            SampleRecord("s", "raw", "Raw", "builtin", 11, 21, 22, 2.0, 0.0, 0.0, 90.0, ""),
            SampleRecord("s", "raw", "Raw", "builtin", 12, 22, 23, 3.0, 0.0, 0.0, 90.0, ""),
            SampleRecord("s", "raw", "Raw", "builtin", 13, 23, 24, 4.0, 0.0, 0.0, 90.0, ""),
        ])

        dataset.trim(1, 2)

        self.assertEqual([record.timestamp_mcu for record in dataset.records], [11, 12])

    def test_delete_rows_removes_selected_indices(self) -> None:
        dataset = Dataset("delete_case")
        dataset.extend([
            SampleRecord("a", "raw", "Raw", "builtin", 10, 20, 21, 1.0, 0.0, 0.0, 90.0, ""),
            SampleRecord("b", "raw", "Raw", "builtin", 11, 21, 22, 2.0, 0.0, 0.0, 90.0, ""),
            SampleRecord("c", "raw", "Raw", "builtin", 12, 22, 23, 3.0, 0.0, 0.0, 90.0, ""),
        ])

        dataset.delete_rows([0, 2])

        self.assertEqual([record.stream_id for record in dataset.records], ["b"])

    def test_concatenate_returns_merged_dataset(self) -> None:
        dataset_a = Dataset("a")
        dataset_a.append(SampleRecord("a", "raw", "Raw", "builtin", 10, 20, 21, 1.0, 0.0, 0.0, 90.0, ""))
        dataset_b = Dataset("b")
        dataset_b.append(SampleRecord("b", "raw", "Raw", "builtin", 20, 30, 31, 0.0, 1.0, 0.0, 0.0, ""))

        merged = dataset_a.concatenate(dataset_b, name="merged")

        self.assertEqual(merged.name, "merged")
        self.assertEqual(len(merged.records), 2)
        self.assertEqual([record.stream_id for record in merged.records], ["a", "b"])

    def test_csv_round_trip_keeps_required_schema(self) -> None:
        dataset = Dataset("round_trip", metadata={"filters": [{"name": "Hard Iron", "params": {"offset_x": 1.0}}]})
        dataset.append(
            SampleRecord(
                "raw_magnetometer",
                "raw",
                "Raw Magnetometer",
                "builtin",
                2000,
                2010,
                2005,
                0.25,
                -0.5,
                0.75,
                153.4,
                "",
            )
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "dataset.csv")
            dataset.to_csv(path)
            reopened = Dataset.from_csv(path)

            self.assertEqual(len(reopened.records), 1)
            self.assertEqual(reopened.records[0].timestamp_pc_est, 2005)
            self.assertAlmostEqual(reopened.records[0].mag_y, -0.5)
            self.assertEqual(reopened.metadata["filters"][0]["name"], "Hard Iron")


if __name__ == "__main__":
    unittest.main()
