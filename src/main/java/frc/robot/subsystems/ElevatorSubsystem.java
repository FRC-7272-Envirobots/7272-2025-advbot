package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class ElevatorSubsystem extends SubsystemBase{

    TalonFX m_leftelevator = new TalonFX(RobotConstants.LeftElevatorCanId);
    TalonFX m_rightelevator = new TalonFX(RobotConstants.RightElevartorCanId);

    public void elevatorUp(){
        m_leftelevator.set(.2);
        m_rightelevator.set(.2);
    }
    public void elevatorDown(){
        m_leftelevator.set(-.2);
        m_rightelevator.set(-.2);

    }
    public void elevatorStop(){
        m_leftelevator.set(0);
        m_rightelevator.set(0);
    }
}
