from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def test_drive_control_state_machine(tmp_path: Path) -> None:
    executable = tmp_path / "drive_control_host_test"
    subprocess.run(
        [
            "cc",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-ICore/Inc",
            "Core/Src/service/drive_control.c",
            "Core/Src/platform/ps2_frame.c",
            "test/drive_control_host_test.c",
            "-lm",
            "-o",
            str(executable),
        ],
        cwd=ROOT,
        check=True,
    )
    subprocess.run([str(executable)], check=True)
