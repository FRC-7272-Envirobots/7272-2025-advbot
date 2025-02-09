package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.RobotConstants;

public class ElevatorSubsystem {

    TalonFX m_leftelevator = new TalonFX(RobotConstants.LeftElevatorCanId);
    TalonFX m_rightelevator = new TalonFX(RobotConstants.RightElevartorCanId);
}
