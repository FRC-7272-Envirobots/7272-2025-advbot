package frc.robot.commands;

import java.awt.Color;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Lightstrip;
import frc.robot.subsystems.OuttakeSubsystem;

public class Routines {

    private IntakeSubsystem intake;
    private OuttakeSubsystem outtake;

    public Routines(IntakeSubsystem intake) {
        this.intake = intake;
        this.outtake = outtake;

    }

    // public Command Systemtest(){

    // }

    // public Command outtakeroutine(){
    // return outtake.normalOuttake()
    // .alongWith(Lightstrip.setcolorcommand(Color.CYAN));

}
