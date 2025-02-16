package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotConstants;

public class ElevatorSubsystem extends SubsystemBase{

    TalonFX m_leftelevator = new TalonFX(RobotConstants.LeftElevatorCanId);
    TalonFX m_rightelevator = new TalonFX(RobotConstants.RightElevartorCanId);

    DigitalInput limitSwitch1;
    DigitalInput limitSwitch2;
    
    public double lfourpos = ElevatorConstants.elevatorL4;
    public double lthreerpos = ElevatorConstants.elevatorL3;
    public double ltwopos = ElevatorConstants.elevatorL2;
    public double lonepos = ElevatorConstants.elevatorL1;
    public double intakepos = ElevatorConstants.elevatorIntake;


   

   
   

    //basic elevator commands 
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
