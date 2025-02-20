package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class IntakeSubsystem extends SubsystemBase{


    SparkMax m_intake = new SparkMax(RobotConstants.IntakeCanId, MotorType.kBrushless);

    public void runIntake(){
        m_intake.set(.5);
    }
    public void stopIntake(){
        m_intake.set(0);
    }
}
