package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.RobotConstants;

public class OuttakeSubsystem {



    TalonFX m_leftouttake = new TalonFX(RobotConstants.LeftOuttakeCanId);
    TalonFX m_rightouttake = new TalonFX(RobotConstants.RightOuttakeCanId);
}
