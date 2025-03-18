package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Climber {


    public static final SparkMax m_climber = new SparkMax(24, MotorType.kBrushless);
    


    public void climb(){
        m_climber.set(.8);
    }
    public void climbStop(){
        m_climber.set(0);
    }
}


