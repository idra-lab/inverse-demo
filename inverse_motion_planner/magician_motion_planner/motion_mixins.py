from typing import cast


class ControlMixin:

    def with_impedance_control(self):
        from magician_msgs.msg import ControlMode
        self.control_msg.control_mode = ControlMode.CONTROL_MODE_IMPEDANCE
        return self

    def with_admittance_control(self, reference_force: float):
        from magician_msgs.msg import ControlMode
        self.control_msg.control_mode = ControlMode.CONTROL_MODE_ADMITTANCE
        self.control_msg.admittance_reference_mode = ControlMode.ADMITTANCE_CONSTANT_REFERENCE
        self.control_msg.reference_force = reference_force
        return self

    @property
    def control_msg(self):
        from magician_msgs.msg import ControlMode
        return cast(ControlMode, self.msg.control_mode)


class SanderMixin:

    def activate_sander(self):
        from magician_msgs.msg import SanderControl
        self.sander_msg.commanded_state = SanderControl.SANDER_CMD_ENABLE
        return self

    def deactivate_sander(self):
        from magician_msgs.msg import SanderControl
        self.sander_msg.commanded_state = SanderControl.SANDER_CMD_DISABLE
        return self

    def sander_leave_unchanged(self):
        from magician_msgs.msg import SanderControl
        self.sander_msg.commanded_state = SanderControl.SANDER_CMD_KEEPLAST
        return self

    @property
    def sander_msg(self):
        from magician_msgs.msg import SanderControl
        return cast(SanderControl, self.msg.sander)
