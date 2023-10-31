import numpy as np
import time
import sys, os
from easydict import EasyDict as edict
from PyQt5 import QtWidgets, QtCore
from pymodaq.daq_utils.daq_utils import (
    ThreadCommand,
    getLineInfo,
    DataFromPlugins,
    Axis,
)
from pymodaq.daq_viewer.utility_classes import DAQ_Viewer_base, comon_parameters
from pymodaq.daq_utils.parameter.utils import iter_children

# Import GreatEyes SDK: In the hardware folder must be placed greateyesSDK.py, greateyes.dll, geCommLib.dll
HARDWARE_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "hardware")
)
sys.path.append(HARDWARE_DIR)
os.add_dll_directory(HARDWARE_DIR)
import greateyesSDK as ge
from pymodaq.utils.parameter import utils as putils

from daq_2Dviewer_GreateyesCCD import DAQ_2DViewer_GreateyesCCD
from pymodaq_plugins_uniblitz.daq_move_plugins.daq_move_VLM1 import DAQ_Move_VLM1


class DAQ_2DViewer_GreateyesATAS(DAQ_2DViewer_GreateyesCCD, DAQ_Move_VLM1):
    """ """

    param_camera = DAQ_2DViewer_GreateyesCCD.params
    params_shutter = DAQ_Move_VLM1.params

    params = [{'title': 'Shutter:', 'name': 'shutter_bool', 'type': 'led', 'value': False},
              {'title': 'ATAS Mode:', 'name': 'atas_mode', 'type': 'led', 'value': False}]

    def __init__(self, parent=None, params_state=None):
        DAQ_Move_VLM1.__init__(self, parent, params_state)
        DAQ_2DViewer_GreateyesCCD.__init__(self, parent, params_state)

    def ini_detector(self, controller=None):
        """Detector communication initialization
        """
        shutter_initialized = DAQ_Move_VLM1.ini_stage(self, controller)
        QtWidgets.QApplication.processEvents()
        if shutter_initialized.initialized:
            self.move_Home()    # close shutter

        camera_initialized = DAQ_2DViewer_GreateyesCCD.ini_detector(self, controller)
        QtWidgets.QApplication.processEvents()

        initialized = shutter_initialized and camera_initialized
        return initialized

    def commit_settings(self, param):
        """ """
        if 'camera_settings' in putils.get_param_path(param):
            DAQ_2DViewer_GreateyesCCD.commit_settings(self, param)
        elif 'spectro_settings' in putils.get_param_path(param):
            DAQ_Move_VLM1.commit_settings(self, param)
        elif param.name() == 'shutter_bool':
            if param.value():
                self.move_Abs(1)
            else:
                self.move_Abs(0)
        QtWidgets.QApplication.processEvents()


    def close(self):
        DAQ_2DViewer_GreateyesCCD.close(self)
        DAQ_Move_VLM1.close(self)

    def grab_data(self, Naverage=1, **kwargs):
        DAQ_2DViewer_GreateyesCCD.grab_data(self, Naverage, **kwargs)

    def stop(self):
        DAQ_2DViewer_GreateyesCCD.stop(self)


def main():
    """
    this method start a DAQ_Viewer object with this defined plugin as detector
    Returns
    -------

    """
    import sys
    from PyQt5 import QtWidgets
    from pymodaq.daq_utils.gui_utils import DockArea
    from pymodaq.daq_viewer.daq_viewer_main import DAQ_Viewer
    from pathlib import Path

    app = QtWidgets.QApplication(sys.argv)
    win = QtWidgets.QMainWindow()
    area = DockArea()
    win.setCentralWidget(area)
    win.resize(1000, 500)
    win.setWindowTitle("PyMoDAQ Viewer")
    detector = Path(__file__).stem[13:]
    det_type = f"DAQ{Path(__file__).stem[4:6].upper()}"
    prog = DAQ_Viewer(area, title="Testing", DAQ_type=det_type)
    win.show()
    prog.detector = detector
    prog.init_det()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
