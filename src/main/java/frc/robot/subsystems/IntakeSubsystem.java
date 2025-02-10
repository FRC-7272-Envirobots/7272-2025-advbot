package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class IntakeSubsystem extends SubsystemBase{


    TalonFX m_intake = new TalonFX(RobotConstants.IntakeCanId);

    public void runIntake(){
        m_intake.set(.3);
    }
    public void stopIntake(){
        m_intake.set(0);
    }
}
