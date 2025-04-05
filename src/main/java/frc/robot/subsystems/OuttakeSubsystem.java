package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class OuttakeSubsystem extends SubsystemBase {

    TalonFX m_leftouttake = new TalonFX(RobotConstants.LeftOuttakeCanId);

    TalonFXConfiguration config;

    public OuttakeSubsystem() {
        config = new TalonFXConfiguration();

    }

    public Command normalOuttake() {
        m_leftouttake.set(1.);

        return null;

    }

    public Command stopOuttake() {
        m_leftouttake.set(0);
        return null;
    }

}
