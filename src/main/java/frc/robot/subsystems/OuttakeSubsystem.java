package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class OuttakeSubsystem extends SubsystemBase{



    TalonFX m_leftouttake = new TalonFX(RobotConstants.LeftOuttakeCanId);
    TalonFX m_rightouttake = new TalonFX(RobotConstants.RightOuttakeCanId);

    public void normalOuttake(){
        m_leftouttake.set(.1);
        m_rightouttake.set(-.1);

    }
    public void stopOuttake(){
        m_leftouttake.set(0);
        m_rightouttake.set(0);
    }
    public void rightOuttake(){
        m_leftouttake.set(.4);
        m_rightouttake.set(.3);
    }
    public void leftOuttake(){
        m_leftouttake.set(.3);
        m_rightouttake.set(.4);
    }
}
   