import sys

import pytest

# Add badge software to pythonpath
sys.path.append("../../../") 

pytest.importorskip("sim.run")
pytest.importorskip("system.hexpansion.config")


def test_import_xystage_app_and_app_export():
    import sim.apps.XYStage.app as XYStage
    from sim.apps.XYStage import XYStageApp
    assert XYStage.__app_export__ == XYStageApp

def test_xystage_app_init():
    from sim.apps.XYStage import XYStageApp
    XYStageApp()

@pytest.fixture
def port():
    return 1
