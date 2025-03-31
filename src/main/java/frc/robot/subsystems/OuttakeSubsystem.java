package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class OuttakeSubsystem extends SubsystemBase {

    TalonFX m_leftouttake = new TalonFX(RobotConstants.LeftOuttakeCanId);
    TalonFX m_rightouttake = new TalonFX(RobotConstants.RightOuttakeCanId);
    TalonFXConfiguration config;

    public OuttakeSubsystem() {
        config = new TalonFXConfiguration();

    }

    public Command normalOuttake() {
        m_leftouttake.set(.1);
        m_rightouttake.set(-.1);
        return null;

    }

    public void stopOuttake() {
        m_leftouttake.set(-.005);
        m_rightouttake.set(0);
    }

}
