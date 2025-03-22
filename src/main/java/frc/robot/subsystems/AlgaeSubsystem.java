package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class AlgaeSubsystem extends SubsystemBase {


   private static final SparkMax m_algaespinner = new SparkMax(RobotConstants.AlgaeRemoverSpinnerCanID, MotorType.kBrushless);
   private static final SparkMax m_algaearm = new SparkMax(RobotConstants.AlgaeRemoverArmCanId, MotorType.kBrushless);
   public final RelativeEncoder m_armEncoder;
   
   
  //  TalonFX m_algaearm = new TalonFX(RobotConstants.AlgaeRemoverArmCanId);
  //public double encodervalue = m_armEncoder.getPosition();

  public AlgaeSubsystem(){
    m_armEncoder = m_algaearm.getAlternateEncoder();
    
    
  }

  //commands 
  //algae spinner 
  public void alagespin (){
    m_algaespinner.set(.2);
  }
  public void algaestop(){
    m_algaespinner.set(0);
  }
 

  //algae arm 
  public void algaeraise(){
    m_algaearm.set(.6);
  }
  public void armstop(){
    m_algaearm.set(0);
  }
  public void loweralgae(){
    m_algaearm.set(-.6);
  }
  

  public Command algaelower(){
   return Commands.run(()->{algaelower();} ,  this);
    
  }
 
  public Command algaeRaise(){
    return Commands.startRun(()->{alagespin();},()->{algaeRaise();}, this);
}
  public Command aglaeLower(){
    return Commands.startRun(()->{algaestop();},()->{algaelower();}, this);
    
  }
  public void algaedown(){
    if (m_armEncoder.getPosition()>5){
      m_algaearm.set(-.6);
    }
    if (m_armEncoder.getPosition()<=5){
      m_algaearm.set(0);
    }
  }
  public void algaeup(){
    if (m_armEncoder.getPosition()<=70){
      m_algaearm.set(6);
    }
    if (m_armEncoder.getPosition()>=5){
      m_algaearm.set(0);
    }
  }
 
}



