from pathlib import Path

import pytest
import yaml

from navigation.config import load_config

EXAMPLE = Path(__file__).resolve().parents[1] / "config/policy.example.yaml"


def test_example_and_relative_paths(tmp_path):
    config = yaml.safe_load(EXAMPLE.read_text())
    config["session_dir"] = "runs"
    file = tmp_path / "config.yaml"
    file.write_text(yaml.safe_dump(config))
    assert load_config(file)["session_dir"] == str(tmp_path / "runs")


@pytest.mark.parametrize("domain", [True, -1, 233, "91"])
def test_reject_invalid_domain(tmp_path, domain):
    config = yaml.safe_load(EXAMPLE.read_text())
    config["ros_domain_id"] = domain
    file = tmp_path / "config.yaml"
    file.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match="ros_domain_id"):
        load_config(file)


def test_fusion_cannot_publish_over_its_localization_prior(tmp_path):
    config = yaml.safe_load(EXAMPLE.read_text())
    config["map_fusion"] = {
        "enabled": True,
        "prior_topic": "/map",
        "output_topic": "/map",
        "scan_topic": "/scan",
    }
    file = tmp_path / "config.yaml"
    file.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match="localization prior"):
        load_config(file)
