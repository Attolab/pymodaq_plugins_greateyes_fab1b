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


class DAQ_2DViewer_GreateyesCCD(DAQ_Viewer_base):
    """ """

    callback_signal = QtCore.pyqtSignal()
    hardware_averaging = False
    params = comon_parameters + [
        {
            "title": "Camera Settings:",
            "name": "camera_settings",
            "type": "group",
            "expanded": True,
            "children": [
                {
                    "title": "Connection type",
                    "name": "connection_type",
                    "type": "list",
                    "values": ["Ethernet", "USB"],
                    "readonly": False,
                },
                {
                    "title": "Camera status",
                    "name": "camera_status",
                    "type": "str",
                    "value": "Not initialized",
                    "readonly": True,
                },
                {
                    "title": "Camera IP (only needed for Ethernet)",
                    "name": "camera_IP",
                    "type": "str",
                    "value": "192.168.1.234",
                    "readonly": False,
                },
                {
                    "title": "DLL Version",
                    "name": "dll_version",
                    "type": "str",
                    "value": "",
                    "readonly": True,
                },
                {
                    "title": "Firmware Version",
                    "name": "firmware_version",
                    "type": "str",
                    "value": "",
                    "readonly": True,
                },
                {
                    "title": "Camera Model ID",
                    "name": "camera_model_id",
                    "type": "int",
                    "value": 0,
                    "readonly": True,
                },
                {
                    "title": "Camera Model",
                    "name": "camera_model_str",
                    "type": "str",
                    "value": "",
                    "readonly": True,
                },
                {
                    "title": "LED On/Off",
                    "name": "led_on",
                    "type": "bool",
                    "value": True,
                },
            ],
        },
        {
            "title": "Acquisition Settings:",
            "name": "acquisition_settings",
            "type": "group",
            "expanded": True,
            "children": [
                {
                    "title": "Image size along x (readonly)",
                    "name": "N_x",
                    "type": "int",
                    "value": 2048,
                    "default": 2048,
                    "min": 1,
                    "readonly": True,
                },
                {
                    "title": "Image size along y (readonly)",
                    "name": "N_y",
                    "type": "int",
                    "value": 512,
                    "default": 512,
                    "min": 1,
                    "readonly": True,
                },
                {
                    "title": "Activate Bias Correction",
                    "name": "do_correct_bias",
                    "type": "bool",
                    "value": False,
                    "default": False,
                },
                {
                    "title": "Capacity Mode",
                    "name": "capacity_mode",
                    "type": "list",
                    "limits": ["Low Noise", "High Signal"],
                },
                {
                    "title": "Gain Mode",
                    "name": "gain_mode",
                    "type": "list",
                    "limits": ["Max Dynamic Range", "Highest Sensitivity"],
                },
                {
                    "title": "Timing settings",
                    "name": "timing_settings",
                    "type": "group",
                    "expanded": True,
                    "children": [
                        {
                            "title": "Exposure time (ms)",
                            "name": "exposure_time",
                            "type": "int",
                            "value": 100,
                            "readonly": False,
                        },
                        {
                            "title": "Readout Speed",
                            "name": "readout_speed",
                            "type": "list",
                            "limits": [],
                            "readonly": False,
                        },
                        {
                            "title": "Check measurement time?",
                            "name": "check_meas_time",
                            "type": "bool",
                            "value": True,
                            "readonly": False,
                        },
                        {
                            "title": "Last measurement time needed (ms)",
                            "name": "last_meas_time",
                            "type": "float",
                            "value": 0.0,
                            "readonly": True,
                        },
                        {
                            "title": "Timeout (ms)",
                            "name": "timeout",
                            "type": "int",
                            "value": 10000,
                            "readonly": False,
                        },
                    ],
                },
                {
                    "title": "Image settings",
                    "name": "image_settings",
                    "type": "group",
                    "expanded": True,
                    "children": [
                        {
                            "title": "Horizontal binning",
                            "name": "bin_x",
                            "type": "int",
                            "value": 1,
                            "default": 1,
                            "min": 1,
                        },
                        {
                            "title": "Vertical binning",
                            "name": "bin_y",
                            "type": "int",
                            "value": 1,
                            "default": 1,
                            "min": 1,
                        },
                        {
                            "title": "Bit Depth (bytes per pixel)",
                            "name": "bit_depth",
                            "type": "list",
                            "limits": [2, 3, 4],
                            "default": 4,
                            "value": 4,
                        },
                        {
                            "title": "Activate Crop Mode",
                            "name": "do_crop",
                            "type": "bool",
                            "value": False,
                            "default": False,
                        },
                        {
                            "title": "Horizontal cropping",
                            "name": "crop_x",
                            "type": "int",
                            "value": 2048,
                            "default": 2048,
                            "min": 1,
                        },
                        {
                            "title": "Vertical cropping",
                            "name": "crop_y",
                            "type": "int",
                            "value": 515,
                            "default": 515,
                            "min": 1,
                        },
                    ],
                },
            ],
        },
        {
            "title": "Temperature Settings:",
            "name": "temperature_settings",
            "type": "group",
            "children": [
                {
                    "title": "Activate temperature control",
                    "name": "do_temperature",
                    "type": "bool",
                    "value": False,
                    "default": False,
                },
                {
                    "title": "Check temperature",
                    "name": "check_temperature",
                    "type": "bool",
                    "value": True,
                    "default": True,
                    "readonly": False,
                },
                {
                    "title": "Set Point",
                    "name": "set_point",
                    "type": "int",
                    "value": 20,
                    "default": 20,
                },
                {
                    "title": "Sensor Temperature",
                    "name": "sensor_temp",
                    "type": "str",
                    "value": "",
                    "readonly": True,
                },
                {
                    "title": "TEC Backside Temperature",
                    "name": "tec_temp",
                    "type": "str",
                    "value": "",
                    "readonly": True,
                },
            ],
        },
    ]

    def __init__(self, parent=None, params_state=None):
        super().__init__(parent, params_state)

        self.x_axis = None
        self.y_axis = None
        self.data_shape = None
        self.controller = None
        self.callback_thread = None

    def ini_detector(self, controller=None):
        """Detector communication initialization

        Parameters
        ----------
        controller: (object) custom object of a PyMoDAQ plugin (Slave case). None if only one detector by controller (Master case)

        Returns
        -------
        self.status (edict): with initialization status: three fields:
            * info (str)
            * controller (object) initialized controller
            *initialized: (bool): False if initialization failed otherwise True
        """
        self.status.update(
            edict(
                initialized=False,
                info="",
                x_axis=None,
                y_axis=None,
                controller=None,
            )
        )

        try:
            # Start initializing
            self.emit_status(
                ThreadCommand("show_splash", ["Initializing Greateyes CCD Camera"])
            )

            if self.settings.child(("controller_status")).value() == "Slave":
                if controller is None:
                    raise Exception(
                        "no controller has been defined externally while this detector is a slave one"
                    )
                else:
                    self.controller = controller
            else:
                self.controller = ge

            self.update_status()
            self.ini_greateyes_camera()

            self.x_axis = self.get_xaxis()
            self.y_axis = self.get_yaxis()
            self.status.x_axis = self.x_axis
            self.status.y_axis = self.y_axis

            # initialize viewers pannel with the future type of data
            # we perform a blocking measurement for simplicity here.

            self.emit_status(ThreadCommand("show_splash", ["Taking one image"]))

            self.data_grabed_signal_temp.emit(
                [
                    DataFromPlugins(
                        name="CCD Image",
                        data=[
                            self.controller.PerformMeasurement_Blocking_DynBitDepth(
                                correctBias=self.settings.child(
                                    "acquisition_settings", "do_correct_bias"
                                ).value()
                            ).astype(np.float)
                        ],
                        dim="Data2D",
                        labels=["dat0"],
                        x_axis=self.y_axis,
                        y_axis=self.x_axis,
                    ),
                ]
            )
            self.settings.child(
                "acquisition_settings", "timing_settings", "last_meas_time"
            ).setValue("{:.1f}".format(self.controller.GetLastMeasTimeNeeded() * 1000))

            self.status.info = "Camera initialized correctly"
            self.status.initialized = True
            self.status.controller = self.controller
            self.emit_status(ThreadCommand("close_splash"))
            return self.status

        except Exception as e:
            self.emit_status(
                ThreadCommand("Update_Status", [getLineInfo() + str(e), "log"])
            )
            self.status.info = getLineInfo() + str(e)
            self.status.initialized = False
            self.emit_status(ThreadCommand("close_splash"))
            return self.status

    def ini_greateyes_camera(self):
        # Disconnect in case it was not done properly
        self.settings.child("camera_settings", "dll_version").setValue(
            self.controller.GetDLLVersion()
        )
        self.controller.DisconnectCamera()
        self.controller.DisconnectCameraServer()

        # Connection setup
        # ===========================
        # USB
        if self.settings.child("camera_settings", "connection_type").value() == "USB":
            connectionSetupWorked = ge.SetupCameraInterface(
                self.controller.connectionType_USB
            )
        # or Ethernet (needs connection to camera server)
        elif (
            self.settings.child("camera_settings", "connection_type").value()
            == "Ethernet"
        ):
            connectionSetupWorked = ge.SetupCameraInterface(
                self.controller.connectionType_Ethernet,
                ipAddress=self.settings.child("camera_settings", "camera_IP").value(),
            )
            if connectionSetupWorked:
                connectionSetupWorked = self.controller.ConnectToSingleCameraServer()
                if connectionSetupWorked:
                    self.emit_status(
                        ThreadCommand("show_splash", ["Connected to Camera Server"])
                    )
                else:
                    raise Exception("Could not connect to camera")
        else:
            connectionSetupWorked = False
            raise ValueError("Unsupported connection type")

        if not connectionSetupWorked:
            raise Exception("Could not connect to camera")

        self.update_status()

        # Connection to camera
        # ====================
        N_Cams = self.controller.GetNumberOfConnectedCams()
        CameraModel = []

        if N_Cams == 1:
            addr = 0
        elif N_Cams > 1:
            compatible_cameras = []
            for cam_addr in range(N_Cams):
                if self.controller.ConnectCamera(CameraModel, addr=cam_addr):
                    compatible_cameras.append(cam_addr)
            if len(compatible_cameras) == 1:
                addr = compatible_cameras[0]
            else:
                raise Exception(
                    "More than one compatible camera was found - not supported by the plugin at the moment."
                )
        elif N_Cams == 0:
            raise Exception("No Camera was found.")

        CameraConnected = self.controller.ConnectCamera(model=CameraModel, addr=addr)
        if CameraConnected:
            self.settings.child("camera_settings", "camera_model_id").setValue(
                CameraModel[0]
            )
            self.settings.child("camera_settings", "camera_model_str").setValue(
                CameraModel[1]
            )
            self.emit_status(
                ThreadCommand("show_splash", ["Connected to Camera " + CameraModel[1]])
            )

            if self.controller.InitCamera(addr=addr):
                self.emit_status(ThreadCommand("show_splash", ["Camera Initialized"]))
            else:
                self.controller.DisconnectCamera()
                raise Exception(
                    "Could not connect to camera; " + self.controller.StatusMSG
                )

        else:
            self.controller.DisconnectCamera()
            if (
                self.settings.child("camera_settings", "connection_type").value()
                == "Ethernet"
            ):
                self.controller.DisconnectCameraServer()
            raise Exception("Could not connect to camera; " + self.controller.StatusMSG)

        self.update_status()

        # Get Functions
        # =================================================
        self.emit_status(
            ThreadCommand("show_splash", ["Obtaining Camera parameters..."])
        )
        self.settings.child("camera_settings", "firmware_version").setValue(
            self.controller.GetFirmwareVersion()
        )
        self.settings.child(
            "acquisition_settings", "timing_settings", "exposure_time"
        ).setLimits((1, self.controller.GetMaxExposureTime()))

        if not self.controller.SupportedSensorFeature(0):  # Supports Capacity Mode?
            self.settings.child("acquisition_settings", "capacity_mode").hide()

        if self.controller.SupportedSensorFeature(
            1
        ):  # Checks if Horizontal Binning is supported
            self.settings.child(
                "acquisition_settings", "image_settings", "bin_x"
            ).setLimits((1, self.controller.GetMaxBinningX()))
        else:
            self.settings.child(
                "acquisition_settings", "image_settings", "bin_x"
            ).hide()

        self.settings.child(
            "acquisition_settings", "image_settings", "bin_y"
        ).setLimits((1, self.controller.GetMaxBinningY()))

        if not self.controller.SupportedSensorFeature(
            2
        ):  # Checks if Horizontal Cropping is supported
            self.settings.child(
                "acquisition_settings", "image_settings", "crop_x"
            ).hide()

        # GetNumberOfSensorOutputModes()
        # GetSensorOutputModeStrings() features not implemented yet.

        # Readout speeds - populate available readout speeds
        speeds = [50, 100, 250, 500, 1, 3]
        speedUnits = ["kHz", "kHz", "kHz", "kHz", "MHz", "MHz"]

        availableSpeeds = []
        for index, speed in enumerate(speeds):
            # try to set camera to the readout speed
            if self.controller.SetReadOutSpeed(
                eval(
                    "self.controller.readoutSpeed_"
                    + str(speed)
                    + "_"
                    + speedUnits[index]
                )
            ):
                availableSpeeds.append(str(speed) + " " + speedUnits[index])
        self.settings.child(
            "acquisition_settings", "timing_settings", "readout_speed"
        ).setLimits(availableSpeeds)
        # by default set camera to the fastest readout speed
        self.settings.child(
            "acquisition_settings", "timing_settings", "readout_speed"
        ).setValue(availableSpeeds[-1])

        # Set Functions
        # =================================================
        # Set bit depth
        self.controller.SetBitDepth(
            self.settings.child(
                "acquisition_settings", "image_settings", "bit_depth"
            ).value()
        )

        # Image size
        imageSize = self.controller.GetImageSize()[0:2]
        self.settings.child("acquisition_settings", "N_x").setValue(imageSize[0])
        self.settings.child("acquisition_settings", "N_y").setValue(imageSize[1])

        # Exposure time
        self.controller.SetExposure(
            self.settings.child(
                "acquisition_settings", "timing_settings", "exposure_time"
            ).value()
        )

        # Temperature control
        # =================================================
        self.settings.child("temperature_settings", "set_point").setLimits(
            self.controller.TemperatureControl_Init()
        )  # Start temperature control and set available temperature range

        if self.settings.child("temperature_settings", "do_temperature").value():
            self.controller.TemperatureControl_SetTemperature(
                self.settings.child("temperature_settings", "set_point").value()
            )
        else:
            self.controller.TemperatureControl_SwitchOff()

        self.timerTemp = self.startTimer(3000)  # Timer event fired every 3 seconds
        if not self.settings.child("temperature_settings", "check_temperature").value():
            self.killTimer(self.timerTemp)

        self.update_image()
        
        # Set up callback
        callback = GreateyesCallback(self.controller.DllIsBusy)
        callback.exposure = self.settings.child(
            "acquisition_settings", "timing_settings", "exposure_time"
        ).value()
        callback.timeout = self.settings.child(
            "acquisition_settings", "timing_settings", "timeout"
        ).value()

        self.callback_thread = QtCore.QThread()
        callback.moveToThread(self.callback_thread)
        callback.data_sig.connect(
            self.emit_data
        )  # when the wait for acquisition returns (with data taken), emit_data will be fired

        self.callback_signal.connect(callback.wait_for_acquisition)
        self.callback_thread.callback = callback
        self.callback_thread.start()

    def update_status(self):
        self.settings.child("camera_settings", "camera_status").setValue(
            self.controller.StatusMSG
        )

    def commit_settings(self, param):
        """ """
        if param.name() == "connection_type":
            if param.value() == "USB":
                self.settings.child("camera_settings", "camera_IP").hide()
            elif param.value() == "Ethernet":
                self.settings.child("camera_settings", "camera_IP").show()

        elif param.name() == "led_on":
            self.controller.SetLEDStatus(param.value())

        elif param.name() == "check_meas_time":
            if param.value():
                self.settings.child(
                    "acquisition_settings", "timing_settings", "last_meas_time"
                ).show()
            else:
                self.settings.child(
                    "acquisition_settings", "timing_settings", "last_meas_time"
                ).hide()

        elif param.name() == "exposure_time":
            self.controller.SetExposure(param.value())
            # Update callback
            self.callback_thread.callback.exposure = param.value()

        elif param.name() == "timeout":
            # Update callback
            self.callback_thread.callback.timeout = param.value()

        elif param.name() == "readout_speed":
            speed, unit = param.value().split()
            if not self.controller.SetReadOutSpeed(
                eval("self.controller.readoutSpeed_" + speed + "_" + unit)
            ):
                self.emit_status(
                    ThreadCommand(
                        "Update_Status", ["Could not update readout speed", "log"]
                    )
                )

        elif param.name() in iter_children(
            self.settings.child("acquisition_settings", "image_settings")
        ):
            self.update_image()

        elif param.name() == "set_point":
            if self.settings.child("temperature_settings", "do_temperature").value():
                self.controller.TemperatureControl_SetTemperature(param.value())

        elif param.name() == "do_temperature":
            if param.value():
                self.controller.TemperatureControl_Init()
                self.controller.TemperatureControl_SetTemperature(param.value())
            else:
                self.controller.TemperatureControl_SwitchOff()

        elif param.name() == "check_temperature":
            if param.value():
                self.timerTemp = self.startTimer(3000)
            else:
                self.killTimer(self.timerTemp)

        elif param.name() == "capacity_mode":
            if param.value() == "Low Noise":
                self.controller.SetupCapacityMode(False)
            elif param.value() == "High Signal":
                self.controller.SetupCapacityMode(True)

        elif param.name() == "gain_mode":
            if param.value() == "Max Dynamic Range":
                self.controller.SetupGain(0)
            elif param.value() == "Highest Sensitivity":
                self.controller.SetupGain(1)

        self.update_status()

    def get_xaxis(self):
        """
        Obtain the horizontal axis of the image.

        Returns
        -------
        1D numpy array
            Contains a vector of integer corresponding to the horizontal camera pixels.
        """
        if self.controller is not None:
            Nx = self.settings.child("acquisition_settings", "N_x").value()
            self.x_axis = Axis(
                data=np.linspace(0, Nx - 1, Nx, dtype=int), label="Pixels"
            )

            self.emit_x_axis()
        else:
            raise (Exception("Controller not defined"))
        return self.x_axis

    def get_yaxis(self):
        """
        Obtain the vertical axis of the image.

        Returns
        -------
        1D numpy array
            Contains a vector of integer corresponding to the vertical camera pixels.
        """
        if self.controller is not None:

            Ny = self.settings.child("acquisition_settings", "N_y").value()
            self.y_axis = Axis(
                data=np.linspace(0, Ny - 1, Ny, dtype=int), label="Pixels"
            )
            self.emit_y_axis()
        else:
            raise (Exception("Controller not defined"))
        return self.y_axis

    def update_image(self):
        self.controller.SetupCropMode2D(
            self.settings.child(
                "acquisition_settings", "image_settings", "crop_x"
            ).value(),
            self.settings.child(
                "acquisition_settings", "image_settings", "crop_y"
            ).value(),
        )
        self.controller.SetBinningMode(
            self.settings.child(
                "acquisition_settings", "image_settings", "bin_x"
            ).value(),
            self.settings.child(
                "acquisition_settings", "image_settings", "bin_y"
            ).value(),
        )
        self.controller.ActivateCropMode(
            self.settings.child(
                "acquisition_settings", "image_settings", "do_crop"
            ).value()
        )

        imageSize = self.controller.GetImageSize()
        self.settings.child("acquisition_settings", "N_x").setValue(imageSize[0])
        self.settings.child("acquisition_settings", "N_y").setValue(imageSize[1])
        self.update_status()

    def timerEvent(self, event):
        """
        Timer event to periodically check the temperature
        """
        sensor_temp = self.controller.TemperatureControl_GetTemperature(0)
        tec_temp = self.controller.TemperatureControl_GetTemperature(1)
        self.settings.child("temperature_settings", "sensor_temp").setValue(sensor_temp)
        self.settings.child("temperature_settings", "tec_temp").setValue(tec_temp)

    def close(self):
        """
        Terminate the communication protocol
        """
        if self.controller.DisconnectCamera():
            if (
                self.settings.child("camera_settings", "connection_type").value()
                == "Ethernet"
            ):
                if ge.DisconnectCameraServer():
                    msg = "Successfully disconnected Camera Server and Camera"
                else:
                    msg = "Error while disconnecting Camera Server"
            else:
                msg = "Successfully disconnected USB Camera"
        else:
            msg = "Error while disconnecting Camera"
        self.emit_status(ThreadCommand("Update_Status", [msg, "log"]))

    def grab_data(self, Naverage=1, **kwargs):
        """
        Parameters
        ----------
        Naverage: (int) Number of hardware averaging
        kwargs: (dict) of others optionals arguments
        """
        try:
            self.settings.child("camera_settings", "camera_status").setValue(
                "Acquiring..."
            )

            self.prepare_data()

            # Start acquisition
            self.controller.StartMeasurement_DynBitDepth(
                correctBias=self.settings.child(
                    "acquisition_settings", "do_correct_bias"
                ).value()
            )
            self.callback_signal.emit()  # will trigger the wait for acquisition

        except Exception as e:
            self.emit_status(ThreadCommand("Update_Status", [str(e), "log"]))

        ## remember to check measurement time if parameter is ticked

    def prepare_data(self):
        width, height, bytesPerPixel = self.controller.GetImageSize()
        # GetMeasurement function allocates memory by itself, no need to do it

        # Switches viewer type depending on image size
        data_shape = "Data2D" if height != 1 else "Data1D"
        if data_shape != self.data_shape:
            self.data_shape = data_shape
            # init the viewers
            self.data_grabed_signal_temp.emit(
                [
                    DataFromPlugins(
                        name="Camera ",
                        data=[np.squeeze(np.zeros((height, width)).astype(np.float))],
                        dim=self.data_shape,
                        labels="Camera",
                    )
                ]
            )

    def emit_data(self):
        """
        Fonction used to emit data obtained by callback.
        See Also
        --------
        daq_utils.ThreadCommand
        """
        try:
            if self.callback_thread.callback.timedout:
                self.emit_status(
                    ThreadCommand("Update_Status", ["Measurement timed out", "log"])
                )

            else:
                size_y = self.settings.child("acquisition_settings", "N_y").value()
                size_x = self.settings.child("acquisition_settings", "N_x").value()
                data = self.controller.GetMeasurementData_DynBitDepth()
                self.data_grabed_signal.emit(
                    [
                        DataFromPlugins(
                            name="Camera",
                            data=[np.squeeze(data.reshape(size_y, size_x)).astype(np.float)],
                            dim=self.data_shape,
                            labels="Camera",
                        )
                    ]
                )
                self.settings.child("camera_settings", "camera_status").setValue(
                    "Data received"
                )

                QtWidgets.QApplication.processEvents()  # here to be sure the timeevents are executed even if in continuous grab mode

                if self.settings.child(
                    "acquisition_settings", "timing_settings", "check_meas_time"
                ):
                    self.settings.child(
                        "acquisition_settings", "timing_settings", "last_meas_time"
                    ).setValue(
                        "{:.1f}".format(self.controller.GetLastMeasTimeNeeded() * 1000)
                    )
        except Exception as e:
            self.emit_status(ThreadCommand("Update_Status", [str(e), "log"]))

    def stop(self):
        self.controller.StopMeasurement()
        return ""


class GreateyesCallback(QtCore.QObject):
    """
    Callback to wait for acquisitions to finish
    """

    data_sig = QtCore.pyqtSignal()

    def __init__(
        self,
        wait_fn,
    ):
        super(GreateyesCallback, self).__init__()
        self.wait_fn = wait_fn
        self.timedout = False
        self.exposure = 0
        self.timeout = 0

    def wait_for_acquisition(self):
        t_meas = 0
        dt = 5  # refresh rate in ms
        t_crit = (
            self.exposure + self.timeout
        )  # time after which we cancel the measurement

        while self.wait_fn():  # DLL is busy
            time.sleep(dt / 1000)
            t_meas = t_meas + dt
            if t_meas >= t_crit:  # if measurement takes took long
                if ge.StopMeasurement():
                    self.timedout = True

        self.data_sig.emit()  # Fired when DLL is not busy: measurement finished, or we stopped it


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
