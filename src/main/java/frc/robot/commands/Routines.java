package frc.robot.commands;

import java.awt.Color;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Lightstrip;
import frc.robot.subsystems.OuttakeSubsystem;

public class Routines extends SubsystemBase {

    private OuttakeSubsystem outtake;
    private ElevatorSubsystem elevator;
    private DriveSubsystem drive;

    public Routines(ElevatorSubsystem m_elevator, OuttakeSubsystem m_outtake, DriveSubsystem m_robotDrive) {
        // TODO Auto-generated constructor stub
        this.drive = drive;
        this.elevator = elevator;
        this.outtake = outtake;
    }

    public Command Systemtest() {
        return outtake.normalOuttake().andThen(new WaitCommand(3)).andThen(outtake.stopOuttake())
                .andThen(Commands.runOnce(() -> elevator.setElevatorL2())).andThen(new WaitCommand(2))
                .andThen(Commands.runOnce(() -> elevator.setElevatorL0())).andThen(new WaitCommand(2))
                .andThen(new InstantCommand(() -> drive.setX())).andThen(new WaitCommand(5))
                .andThen(new InstantCommand(() -> drive.setfoward())).andThen(new WaitCommand(4))
                .andThen(new InstantCommand(() -> drive.setbackwards())).andThen(new WaitCommand(4))
                .andThen(new InstantCommand(() -> drive.setleft())).andThen(new WaitCommand(4))
                .andThen(new InstantCommand(() -> drive.setright())).andThen(new WaitCommand(4));

    }

}
