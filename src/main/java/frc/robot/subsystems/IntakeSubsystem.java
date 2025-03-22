package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class IntakeSubsystem extends SubsystemBase{


    SparkMax m_intakel = new SparkMax(RobotConstants.IntakelCanId,MotorType.kBrushless);
    SparkMax m_intaker = new SparkMax(RobotConstants.IntakerCanId,MotorType.kBrushless);

    public void runIntake(){
        m_intakel.set(.15);
        m_intaker.set(-.15);
    }
    public void stopIntake(){
        m_intakel.set(0);
        m_intaker.set(0);
    }
}
