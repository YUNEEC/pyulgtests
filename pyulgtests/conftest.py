"""Configureation for tests."""

import pytest
import os


def pytest_addoption(parser):
    """Check for correct argument."""
    parser.addoption(
        "--filepath",
        action="store",
        default="Please pass absolute log file path as argument",
        help="absolute path to log file",
    )


@pytest.fixture(scope="session")
def filepath(request):
    """For each session, get filepath."""
    return request.config.getoption("--filepath")


@pytest.fixture(scope="session", autouse=True)
def filecheck(filepath):
    """Check if file is ulg file."""
    base, ext = os.path.splitext(filepath)
    if ext.lower() not in (".ulg") or not filepath:
        pytest.exit("passed file is not a .ulg file.")
