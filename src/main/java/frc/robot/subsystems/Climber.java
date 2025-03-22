package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {


    public static final SparkMax m_climber = new SparkMax(24, MotorType.kBrushless);
    


    public void climb(){
        m_climber.set(.8);
    }
    public void climbStop(){
        m_climber.set(0);
    }
    public void startClimb(){
        m_climber.set(-.8);
    }
}


