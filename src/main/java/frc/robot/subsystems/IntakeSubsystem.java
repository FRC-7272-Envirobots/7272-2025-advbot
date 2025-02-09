package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.RobotConstants;

public class IntakeSubsystem {


    TalonFX m_intake = new TalonFX(RobotConstants.IntakeCanId);
}
