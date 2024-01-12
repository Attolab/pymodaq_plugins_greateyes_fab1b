import numpy as np
import time
import sys, os
from PyQt5 import QtWidgets, QtCore
from pymodaq.utils.data import Axis, DataFromPlugins, DataToExport

# Import GreatEyes SDK: In the hardware folder must be placed greateyesSDK.py, greateyes.dll, geCommLib.dll
HARDWARE_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "hardware")
)
sys.path.append(HARDWARE_DIR)
os.add_dll_directory(HARDWARE_DIR)

from pymodaq.utils.parameter import utils as putils

from .daq_2Dviewer_GreateyesCCD import DAQ_2DViewer_GreateyesCCD
from ...daq_move_plugins.daq_move_VLM1 import DAQ_Move_VLM1


class DAQ_2DViewer_GreateyesATAS(DAQ_2DViewer_GreateyesCCD, DAQ_Move_VLM1):
    """ """

    params_camera = DAQ_2DViewer_GreateyesCCD.params
    params_shutter = DAQ_Move_VLM1.params
    d = putils.get_param_dict_from_name(params_shutter, 'multiaxes')
    if d is not None:
        d['visible'] = False

    shutter_status = False
    pump_off = None
    pump_on = None

    params = [{'title': 'Shutter:', 'name': 'shutter_bool', 'type': 'led_push', 'value': False},
              {'title': 'ATAS Mode:', 'name': 'atas_mode', 'type': 'led_push', 'value': False},
              {'title': 'Displayed quantity', 'name': 'quantity', 'type': 'list', 'limits': ['dR/R', 'DeltaOD'], 'value': 'dR/R'},
              {'title': 'Set background', 'name': 'do_bkg', 'type': 'bool_push', 'value': False},
              {'title': 'Clear background', 'name': 'clear_bkg', 'type': 'bool_push', 'value': False}] \
             + params_camera + params_shutter

    def __init__(self, parent=None, params_state=None):
        DAQ_Move_VLM1.__init__(self, parent, params_state)
        DAQ_2DViewer_GreateyesCCD.__init__(self, parent, params_state)
        self.camera_controller = None
        self.shutter_controller = None
        self.background_poff = None
        self.background_pon = None

    def ini_detector(self, controller=None):
        """Detector communication initialization
        """
        shutter_initialized = DAQ_Move_VLM1.ini_stage(self, controller)
        QtWidgets.QApplication.processEvents()
        if shutter_initialized.initialized:
            self.move_Home()  # close shutter

        camera_initialized = DAQ_2DViewer_GreateyesCCD.ini_detector(self, controller)
        QtWidgets.QApplication.processEvents()

        initialized = shutter_initialized and camera_initialized['initialized']
        return camera_initialized['info'], initialized

    def commit_settings(self, param):
        """ """

        if param.name() == 'COM_port':
            DAQ_Move_VLM1.commit_settings(self, param)
        elif param.name() == 'shutter_bool':
            if param.value():
                self.move_Abs(1)
            else:
                self.move_Abs(0)
        elif param.name() == 'atas_mode':
            pass
        elif param.name() == 'quantity':
            pass
        elif param.name() == 'do_bkg':
            self.take_background()
            param.setValue(False)
        elif param.name() == 'clear_bkg':
            self.background_poff = None
            self.background_pon = None
            param.setValue(False)
        else:
            DAQ_2DViewer_GreateyesCCD.commit_settings(self, param)
            
        QtWidgets.QApplication.processEvents()

    def close(self):
        DAQ_2DViewer_GreateyesCCD.close(self)

    def move_Abs(self, position):
        DAQ_Move_VLM1.move_Abs(self, position)
        if position == 0:
            self.settings.child('shutter_bool').setValue(False)
        else:
            self.settings.child('shutter_bool').setValue(True)

    def grab_data(self, Naverage=1, **kwargs):

        if not (self.settings['atas_mode'] and self.data_shape == 'Data1D'):
            # normal mode
            DAQ_2DViewer_GreateyesCCD.grab_data(self, Naverage, **kwargs)

        else:   #atas mode
            size_y = self.settings.child("acquisition_settings", "N_y").value()
            size_x = self.settings.child("acquisition_settings", "N_x").value()

            self.move_Abs(0)
            QtCore.QThread.msleep(16)   #opening time of shutter

            self.camera_controller.StartMeasurement_DynBitDepth(
                correctBias=self.settings.child(
                    "acquisition_settings", "do_correct_bias"
                ).value()
            )
            t_meas = 0
            while self.camera_controller.DllIsBusy():  # DLL is busy
                time.sleep(0.005)
                t_meas += 5
                if t_meas >= self.settings["acquisition_settings", "timing_settings", "timeout"]:  # if measurement takes took long
                    raise Warning('Measurement timeout')
            pump_off = self.camera_controller.GetMeasurementData_DynBitDepth()
            pump_off = np.squeeze(pump_off.reshape(size_y, size_x)).astype(float)

            #now do pump on
            self.move_Abs(1)
            QtCore.QThread.msleep(16)  # opening time of shutter

            self.camera_controller.StartMeasurement_DynBitDepth(
                correctBias=self.settings.child(
                    "acquisition_settings", "do_correct_bias"
                ).value()
            )
            t_meas = 0
            while self.camera_controller.DllIsBusy():  # DLL is busy
                time.sleep(0.005)
                t_meas += 5
                if t_meas >= self.settings["acquisition_settings", "timing_settings", "timeout"]:  # if measurement takes took long
                    raise Warning('Measurement timeout')
            pump_on = self.camera_controller.GetMeasurementData_DynBitDepth()
            pump_on = np.squeeze(pump_on.reshape(size_y, size_x)).astype(float)

            if self.settings.child(
                    "acquisition_settings", "timing_settings", "check_meas_time"
            ):
                self.settings.child(
                    "acquisition_settings", "timing_settings", "last_meas_time"
                ).setValue(
                    "{:.1f}".format(self.camera_controller.GetLastMeasTimeNeeded() * 1000)
                )

            if self.background_poff is not None:
                pump_off -= self.background_poff
            if self.background_pon is not None:
                pump_on -= self.background_pon

            data_to_emit = [pump_off, pump_on]

            if self.settings['quantity'] == 'dR/R':
                od = (pump_on - pump_off) / pump_off
                #od = pump_on/pump_off
            elif self.settings['quantity'] == 'DeltaOD':
                od = -np.real(np.log10(pump_on / pump_off))
            od[np.isnan(od)] = 0
            od[np.isinf(od)] = 0

            data_list = [DataFromPlugins(
                name="Camera",
                data=data_to_emit,
                dim=self.data_shape,
                labels=['Pump off', 'Pump on'],
            ),
                DataFromPlugins(
                    name="Processed signal",
                    data=[od],
                    dim=self.data_shape,
                    labels=[self.settings['quantity']],
                )
            ]

            self.dte_signal.emit(DataToExport('Greateyes',data=data_list))
            QtWidgets.QApplication.processEvents()
            self.move_Abs(0)

    def take_background(self):
        datas = self.parent.datas[0]['data']
        self.background_poff = datas[0]
        self.background_pon = datas[1]

    def stop(self):
        self.move_Abs(0)
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
