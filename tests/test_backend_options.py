from pathlib import Path

import pytest
import yaml

from navigation.backend import materialize


def test_external_parameters_are_unchanged(tmp_path):
    path = tmp_path / "external.yaml"
    path.write_text("node:\n  ros__parameters:\n    topic: /original\n    rate: 10\n")
    cfg = {"backend_params": str(path), "session_dir": str(tmp_path / "session")}
    assert materialize(cfg) is cfg
    prepared = materialize(
        {**cfg, "backend_overrides": {"node": {"ros__parameters": {"topic": "/custom"}}}}
    )
    result = yaml.safe_load(Path(prepared["backend_params"]).read_text())
    assert result["node"]["ros__parameters"] == {"topic": "/custom", "rate": 10}
    assert yaml.safe_load(path.read_text())["node"]["ros__parameters"]["topic"] == "/original"


def test_override_type_conflict_fails_before_launch(tmp_path):
    path = tmp_path / "params.yaml"
    path.write_text("node: 1\n")
    with pytest.raises(ValueError):
        materialize(
            {
                "backend_params": str(path),
                "session_dir": str(tmp_path),
                "backend_overrides": {"node": {"child": 1}},
            }
        )


def test_builtin_launch_is_optional_and_generic(tmp_path):
    from navigation.config import load_config

    root = Path(__file__).resolve().parents[1]
    source = root / "config/policy.example.yaml"
    cfg = yaml.safe_load(source.read_text())
    cfg.pop("launch_file", None)
    config = tmp_path / "node.yaml"
    config.write_text(yaml.safe_dump(cfg))
    result = load_config(config)
    assert Path(result["launch_file"]).is_file()
    assert "/h1/" not in Path(result["launch_file"]).read_text()
