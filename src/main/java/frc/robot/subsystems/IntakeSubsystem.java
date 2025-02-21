package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class IntakeSubsystem extends SubsystemBase{


    TalonFX m_intakel = new TalonFX(RobotConstants.IntakelCanId);
    TalonFX m_intaker = new TalonFX(RobotConstants.IntakelCanId);

    public void runIntake(){
        m_intakel.set(.5);
        m_intaker.set(.5);
    }
    public void stopIntake(){
        m_intakel.set(0);
        m_intaker.set(0);
    }
}
