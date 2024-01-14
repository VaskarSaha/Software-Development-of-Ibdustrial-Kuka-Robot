from typing import List
from .postypes.configuration import configuration
from .postypes.trajectory import trajectory
from .postypes.SixDPos import SixDPos
from .kinematics.direct.fw_kinematics import FwKinematics
from .kinematics.inverse.inverse_kinematics import InvKinematics
from .pathplanner.ptp.ptp import Ptp
from .pathplanner.lin.lin import Lin


class SdirCtrl:
    def get_pos_from_config(self, config: configuration) -> SixDPos:
        fw_kinematics = FwKinematics()
        new_pos = fw_kinematics.get_fw_kinematics(config)
        return new_pos

    def get_config_from_pos(self, pos: SixDPos) -> List[configuration]:
        inv_kinematics = InvKinematics()
        new_cfg = inv_kinematics.get_inv_kinematics(pos)
        return new_cfg

    def move_robot_ptp(self, start, end) -> trajectory:
        if isinstance(start, SixDPos) and isinstance(end, SixDPos):
            return self.move_robot_ptp_position(start, end)
        elif isinstance(start, configuration) and isinstance(end, configuration):
            return self.move_robot_ptp_configuration(start, end)
        else:
            raise ValueError("Invalid argument types for move_robot_ptp")
    
    def move_robot_ptp_sync(self, start, end) -> trajectory:
        if isinstance(start, SixDPos) and isinstance(end, SixDPos):
            return self.move_robot_ptp_position(start, end)
        elif isinstance(start, configuration) and isinstance(end, configuration):
            return self.move_robot_ptp_configuration_sync(start, end)
        else:
            raise ValueError("Invalid argument types for move_robot_ptp_sync")
    

    def move_robot_lin(self, start, end) -> trajectory:
        if isinstance(start, SixDPos) and isinstance(end, SixDPos):
            return self.move_robot_lin_position(start, end)
        elif isinstance(start, configuration) and isinstance(end, configuration):
            return self.move_robot_lin_configuration(start, end)
        else:
            raise ValueError("Invalid argument types for move_robot_lin")

    def move_robot_ptp_position(self, start: SixDPos, end: SixDPos) -> trajectory:
        # Implement PTP trajectory calculation from positions here.
        pass

    def move_robot_ptp_configuration(self, start: configuration, end: configuration) -> trajectory:
        ptp = Ptp()
        return ptp.get_ptp_trajectory(start, end)

    def move_robot_ptp_configuration_sync(self, start: configuration, end: configuration) -> trajectory:
        ptp = Ptp()
        return ptp.get_ptp_trajectory_sync(start, end)

    def move_robot_lin_position(self, start: SixDPos, end: SixDPos) -> trajectory:
        # Implement LIN trajectory calculation from positions here.
        pass

    def move_robot_lin_configuration(self, start: configuration, end: configuration) -> trajectory:
        lin = Lin()
        return lin.get_lin_trajectory(start, end)