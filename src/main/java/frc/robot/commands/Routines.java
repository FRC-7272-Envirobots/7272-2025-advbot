package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

public class Routines extends SubsystemBase {

    private OuttakeSubsystem outtake;
    private ElevatorSubsystem elevator;
    private DriveSubsystem drive;

    public Routines(ElevatorSubsystem m_elevator, OuttakeSubsystem m_outtake, DriveSubsystem m_robotDrive) {
        // TODO Auto-generated constructor stub
        this.drive = m_robotDrive;
        this.elevator = m_elevator;
        this.outtake = m_outtake;
        
    }
}
