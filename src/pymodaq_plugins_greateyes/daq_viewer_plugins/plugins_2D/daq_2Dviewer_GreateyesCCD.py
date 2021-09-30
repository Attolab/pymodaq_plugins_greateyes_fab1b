import numpy as np
from easydict import EasyDict as edict
from PyQt5 import QtWidgets
from pymodaq.daq_utils.daq_utils import (
    ThreadCommand,
    getLineInfo,
    DataFromPlugins,
    Axis,
)
from pymodaq.daq_viewer.utility_classes import DAQ_Viewer_base, comon_parameters

from ...hardware import greateyesSDK as ge


class DAQ_2DViewer_GreateyesCCD(DAQ_Viewer_base):
    """ """

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
                    "values": ["USB", "Ethernet"],
                    "readonly": False,
                },
                {
                    "title": "Camera status",
                    "name": "camera_status",
                    "type": "str",
                    "values": "Not initialized",
                    "readonly": True,
                },
                {
                    "title": "Camera IP (only needed for Ethernet)",
                    "name": "camera_IP",
                    "type": "str",
                    "value": "",
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
                            "values": [],
                            "readonly": False,
                        },
                        {
                            "title": "Check measurement time?",
                            "name": "check_meas_time",
                            "type": "bool",
                            "value": False,
                            "readonly": False,
                        },
                        {
                            "title": "Last measurement time needed",
                            "name": "last_meas_time",
                            "type": "float",
                            "value": 0.0,
                            "readonly": True,
                        },

                    ],
                },
                {
                    "title": "Image settings",
                    "name": "image_settings",
                    "type": "group",
                    "expanded": False,
                    "children": [
                        {
                            "title": "Binning along x:",
                            "name": "bin_x",
                            "type": "int",
                            "value": 1,
                            "default": 1,
                            "min": 1,
                        },
                        {
                            "title": "Binning along y:",
                            "name": "bin_y",
                            "type": "int",
                            "value": 1,
                            "default": 1,
                            "min": 1,
                        },
                        {
                            "title": "Activate Crop Mode",
                            "name": "do_crop",
                            "type": "bool",
                            "value": False,
                            "default": False,
                        },
                        {
                            "title": "Cropping along x:",
                            "name": "crop_x",
                            "type": "int",
                            "value": 2048,
                            "default": 2048,
                            "min": 1,
                        },
                        {
                            "title": "Cropping along y:",
                            "name": "crop_y",
                            "type": "int",
                            "value": 512,
                            "default": 512,
                            "min": 1,
                        },
                        {
                            "title": "Image size along x",
                            "name": "N_x",
                            "type": "int",
                            "value": 2048,
                            "default": 2048,
                            "min": 1,
                            "readonly": True,
                        },
                        {
                            "title": "Image size along y",
                            "name": "N_y",
                            "type": "int",
                            "value": 512,
                            "default": 512,
                            "min": 1,
                            "readonly": True,
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
                    "title": "Set Point:",
                    "name": "set_point",
                    "type": "float",
                    "value": 20,
                    "default": 20,
                },
                {
                    "title": "Current value:",
                    "name": "current_value",
                    "type": "float",
                    "value": 0,
                    "default": 0,
                    "readonly": True,
                },
                {
                    "title": "Check temperature",
                    "name": "check_temperature",
                    "type": "bool",
                    "value": True,
                    "default": True,
                    "readonly": False,
                },
            ],
        },
    ]

    def __init__(self, parent=None, params_state=None):
        super().__init__(parent, params_state)

        self.x_axis = None
        self.y_axis = None
        self.cooling_limits = None

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
                ThreadCommand("show_splash", ["Initialising Greateyes CCD Camera "])
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

            self.ini_camera()

            data_x_axis = self.get_xaxis()  # if possible
            self.x_axis = Axis(data=data_x_axis, label="", units="")
            self.emit_x_axis()
            data_y_axis = self.get_yaxis()  # if possible
            self.y_axis = Axis(data=data_y_axis, label="", units="")
            self.emit_y_axis()

            # initialize viewers pannel with the future type of data
            # we perform a blocking measurement for simplicity here.
            self.data_grabed_signal_temp.emit(
                [
                    DataFromPlugins(
                        name="CCD Image",
                        data=[self.controller.PerformMeasurement_Blocking_DynBitDepth()],
                        dim="Data2D",
                        labels=["dat0"],
                        x_axis=self.x_axis,
                        y_axis=self.y_axis,
                    ),
                ]
            )
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

    def ini_camera(self):
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
                ipAddress=self.settings.child("camera_settings", "camera_IP"),
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

        # Connection to camera
        # ====================
        N_Cams = self.controller.GetNumberOfConnectedCams()
        if N_Cams == 1:
            CameraModel = []
            CameraConnected = self.controller.ConnectCamera(model=CameraModel, addr=0)
            if CameraConnected:
                self.params.children("camera_settings", "camera_model_id").setValue(
                    CameraModel[0]
                )
                self.params.children("camera_settings", "camera_model_str").setValue(
                    CameraModel[1]
                )
                self.emit_status(
                    ThreadCommand(
                        "show_splash", ["Connected to Camera " + CameraModel[1]]
                    )
                )

                if self.controller.InitCamera(addr=0):
                    self.emit_status(
                        ThreadCommand("show_splash", ["Camera Initialized"])
                    )
                else:
                    self.controller.DisconnectCamera()
                    raise Exception(
                        "Could not connect to camera; " + self.controller.StatusMSG
                    )

            else:
                self.controller.DisconnectCamera()
                if (
                        self.params.children("camera_settings", "connection_type").value()
                        == "Ethernet"
                ):
                    self.controller.DisconnectCameraServer()
                raise Exception(
                    "Could not connect to camera; " + self.controller.StatusMSG
                )
        elif N_Cams == 0:
            raise Exception(
                "No Camera was found."
            )
        else:
            raise Exception(
                "More than one camera was found - not supported by the plugin at the moment."
            )

        # Get Functions
        # =================================================
        self.emit_status(
            ThreadCommand("show_splash", ["Obtaining Camera parameters..."])
        )
        self.params.children("camera_settings", "dll_version").setValue(
            self.controller.GetDLLVersion()
        )
        self.params.children("camera_settings", "firmware_version").setValue(
            self.controller.GetFirmwareVersion()
        )
        self.params.children("acquisition_settings", "timing_settings", "exposure_time").setLimits(
            (1, self.controller.GetMaxExposureTime())
        )

        self.controller.SupportedSensorFeature(0)  # Supports Capacity Mode? # TODO

        if self.controller.SupportedSensorFeature(
                1
        ):  # Checks if Horizontal Binning is supported
            self.params.children("acquisition_settings", "image_settings", "bin_x").setLimits(
                (1, self.controller.GetMaxBinningX())
            )
        else:
            self.params.children("acquisition_settings", "image_settings", "bin_x").hide()

        self.params.children("acquisition_settings", "image_settings", "bin_y").setLimits(
            (1, self.controller.GetMaxBinningY())
        )

        if self.controller.SupportedSensorFeature(
                2
        ):  # Checks if Horizontal Cropping is supported
            # TODO
            #  self.params.children("camera_settings", "bin_x").setLimits(
            #  (1, self.controller.GetMaxBinningX()))
            pass
        else:
            self.params.children("acquisition_settings", "image_settings", "crop_x").hide()

        # GetNumberOfSensorOutputModes()
        # GetSensorOutputModeStrings() features not implemented yet.

        self.params.children("temperature_settings", "set_point").setLimits(
            self.controller.TemperatureControl_Init()
        )  # Temperature range

        # Readout speeds - populate available readout speeds
        speeds = [50, 100, 250, 500, 1, 3]
        speedUnits = ["kHz", "kHz", "kHz", "kHz", "MHz", "MHz"]

        availableSpeeds = []
        for speed, index in enumerate(speeds):
            # try to set camera to the readout speed
            if self.controller.SetReadOutSpeed(eval("readoutSpeed_" + str(speed) + "_" + speedUnits[index])):
                availableSpeeds.append(str(speed) + " " + speedUnits[index])
        self.settings.child("acquisition_settings", "timing settings", "readout_speed").setOpts(values=availableSpeeds)

    def update_status(self):
        self.settings.child("camera_settings", "camera_status").setValue(self.controller.StatusMSG)

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
                self.settings.child("acquisition_settings", "timing_settings", "last_meas_time").show()
            else:
                self.settings.child("acquisition_settings", "timing_settings", "last_meas_time").hide()

        elif param.name() == "exposure_time":
            self.controller.SetExposure(param.value())

    def get_xaxis(self):
        """
            Obtain the horizontal axis of the image.

            Returns
            -------
            1D numpy array
                Contains a vector of integer corresponding to the horizontal camera pixels.
        """
        if self.camera_controller is not None:
            # if self.control_type == "camera":
            Nx = self.settings.child('camera_settings', 'image_size', 'Nx').value()
            self.x_axis = Axis(data=np.linspace(0, Nx - 1, Nx, dtype=np.int), label='Pixels')

            self.emit_x_axis()
        else:
            raise (Exception('controller not defined'))
        return self.x_axis

    def get_yaxis(self):
        """
            Obtain the vertical axis of the image.

            Returns
            -------
            1D numpy array
                Contains a vector of integer corresponding to the vertical camera pixels.
        """
        if self.camera_controller is not None:

            Ny = self.settings.child('camera_settings', 'image_size', 'Ny').value()
            self.y_axis = Axis(data=np.linspace(0, Ny - 1, Ny, dtype=np.int), label='Pixels')
            self.emit_y_axis()
        else:
            raise (Exception('Camera not defined'))
        return self.y_axis

    def update_image(self):
        self.controller.SetupCropMode2D(
            self.settings.child('acquisition_settings', 'image_settings', 'crop_x').value(),
            self.settings.child('acquisition_settings', 'image_settings', 'crop_y').value(),
        )
        self.controller.SetBinningMode(
            self.settings.child('acquisition_settings', 'image_settings', 'bin_x').value(),
            self.settings.child('acquisition_settings', 'image_settings', 'bin_y').value(),
        )
        self.controller.ActivateCropMode(self.settings.child('acquisition_settings', 'image_settings', 'do_crop').value())

        imageSize = self.controller.GetImageSize()
        self.settings.child('acquisition_settings', 'image_settings', 'N_x').setValue(imageSize[0])
        self.settings.child('acquisition_settings', 'image_settings', 'N_y').setValue(imageSize[1])
        self.update_status()

    def close(self):
        """
        Terminate the communication protocol
        """
        ## TODO for your custom plugin
        self.controller.your_method_to_terminate_the_communication()
        ##

    def prepare_data(self):
        sizex = self.settings.child('acquisition_settings', 'image_size', 'N_x').value()
        sizey = self.settings.child('acquisition_settings', 'image_size', 'N_y').value()

        # Initialize data: self.data for the memory to store new data and self.data_average to store the average data
        image_size = sizex * sizey
        self.data = np.zeros((image_size,), dtype=np.long)
        # self.data_pointer = self.data.ctypes.data_as(ctypes.c_void_p)

        data_shape = 'Data2D' if sizey != 1 else 'Data1D'
        if data_shape != self.data_shape:
            self.data_shape = data_shape
            # init the viewers
            self.data_grabed_signal_temp.emit([DataFromPlugins(name='Camera ',
                                                               data=[np.squeeze(
                                                                   self.data.reshape((sizey, sizex)).astype(np.float))],
                                                               dim=self.data_shape)])

    def grab_data(self, Naverage=1, **kwargs):
        """

        Parameters
        ----------
        Naverage: (int) Number of hardware averaging
        kwargs: (dict) of others optionals arguments
        """
        ## TODO for your custom plugin

        ##synchrone version (blocking function)
        data_tot = self.controller.your_method_to_start_a_grab_snap()
        self.data_grabed_signal.emit(
            [
                DataFromPlugins(
                    name="Mock1", data=data_tot, dim="Data2D", labels=["dat0"]
                )
            ]
        )
        #########################################################

        ##asynchrone version (non-blocking function with callback)
        self.controller.your_method_to_start_a_grab_snap(self.callback)
        #########################################################

    def callback(self):
        """optional asynchrone method called when the detector has finished its acquisition of data"""
        data_tot = self.controller.your_method_to_get_data_from_buffer()
        self.data_grabed_signal.emit(
            [
                DataFromPlugins(
                    name="Mock1", data=data_tot, dim="Data2D", labels=["dat0"]
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
            self.ind_grabbed += 1
            sizey = self.settings.child('camera_settings', 'image_size', 'Ny').value()
            sizex = self.settings.child('camera_settings', 'image_size', 'Nx').value()
            self.camera_controller.GetAcquiredDataNumpy(self.data_pointer, sizex * sizey)
            self.data_grabed_signal.emit([DataFromPlugins(name='Camera',
                                                          data=[np.squeeze(
                                                              self.data.reshape((sizey, sizex)).astype(np.float))],
                                                          dim=self.data_shape)])
            QtWidgets.QApplication.processEvents()  # here to be sure the timeevents are executed even if in continuous grab mode

        except Exception as e:
            self.emit_status(ThreadCommand('Update_Status', [str(e), 'log']))

    def stop(self):

        ## TODO for your custom plugin
        self.controller.your_method_to_stop_acquisition()
        self.emit_status(ThreadCommand("Update_Status", ["Some info you want to log"]))
        ##############################

        return ""


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
