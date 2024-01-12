from pymodaq.control_modules.move_utility_classes import DAQ_Move_base, comon_parameters_fun, main  # common set of parameters for all actuators
from pymodaq.utils.daq_utils import ThreadCommand # object used to send info back to the main thread
from pymodaq.utils.parameter import Parameter


class DAQ_Move_GEExposureController(DAQ_Move_base):
    """Plugin for the Template Instrument

    This object inherits all functionality to communicate with PyMoDAQ Module through inheritance via DAQ_Move_base
    It then implements the particular communication with the instrument

    Attributes:
    -----------
    controller: object
        The particular object that allow the communication with the hardware, in general a python wrapper around the
         hardware library
    """
    _controller_units = 's'
    is_multiaxes = True
    axes_names = ['Exposure time']

    params = comon_parameters_fun(is_multiaxes, axes_names)

    def ini_attributes(self):
        # This class should only be used as slave
        pass

    def get_actuator_value(self):
        """Get the current value from the hardware with scaling conversion.

        Returns
        -------
        float: The position obtained after scaling conversion.
        """
        pos = self.controller.current_exposure_time
        pos = self.get_position_with_scaling(pos)
        return pos

    def close(self):
        """Terminate the communication protocol"""
        pass

    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        pass
        # if param.name() == "a_parameter_you've_added_in_self.params":
        #    self.controller.your_method_to_apply_this_param_change()
        # else:
        #     pass

    def ini_stage(self, controller=None):
        """Actuator communication initialization

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator by controller (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True
        """
        if controller is None:
            raise Exception('This class should only be used as a slave to a Greateyes CCD')

        self.ini_stage_init(old_controller=controller,
                            new_controller=None)

        info = "Initialized exposure controller as a slave to the camera"
        return info

    def move_abs(self, value):
        """ Move the actuator to the absolute target defined by value

        Parameters
        ----------
        value: (float) value of the absolute target positioning
        """

        value = self.check_bound(value)  #if user checked bounds, the defined bounds are applied here
        self.target_value = value
        value = self.set_position_with_scaling(value)  # apply scaling if the user specified one

        self.controller.current_exposure_time = value
        self.emit_status(ThreadCommand('Update_Status', [f'Exposure time set to {value}']))


    def move_rel(self, value):
        """ Move the actuator to the relative target actuator value defined by value

        Parameters
        ----------
        value: (float) value of the relative target positioning
        """
        value = self.check_bound(self.current_position + value) - self.current_position
        self.target_value = value + self.current_position
        value = self.set_position_relative_with_scaling(value)

        self.controller.current_exposure_time = value
        self.emit_status(ThreadCommand('Update_Status', [f'Exposure time set to {value}']))


    def move_home(self):
        """Call the reference method of the controller"""

        self.controller.current_exposure_time = 1
        self.emit_status(ThreadCommand('Update_Status', [f'Exposure time set to {value}']))


    def stop_motion(self):
        pass

if __name__ == '__main__':
    main(__file__)