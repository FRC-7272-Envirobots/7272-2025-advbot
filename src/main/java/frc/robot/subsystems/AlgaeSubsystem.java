package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.RobotConstants;

public class AlgaeSubsystem {


    SparkMax m_algaespinner = new SparkMax(RobotConstants.AlgaeRemoverSpinnerCanID, MotorType.kBrushless);

    TalonFX m_algaearm = new TalonFX(RobotConstants.AlgaeRemoverArmCanId);
    
}
