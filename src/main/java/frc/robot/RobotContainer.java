// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final OuttakeSubsystem m_outtake = new OuttakeSubsystem();
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

    // The driver's controller
    public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    Joystick m_arcadeBox = new Joystick(1);
    Joystick m_psoc = new Joystick(2);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

        private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        autoChooser =  AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);


        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                true),
                        m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    public void configureButtonBindings() {
        //  new JoystickButton(m_psoc,4)
        //        .whileTrue(new RunCommand(
        //                () -> m_robotDrive.setX(),
        //                 m_robotDrive));

        // intake button
         new JoystickButton(m_driverController, XboxController.Button.kB.value)
                .whileTrue(new RunCommand(
                 () -> m_intake.runIntake(),
                         m_intake))
                 .whileFalse(new RunCommand(() -> m_intake.stopIntake(), m_intake));

         // outtake button
         new JoystickButton(m_driverController , XboxController.Button.kA.value)
                 .whileTrue(new RunCommand(
                         () -> m_outtake.normalOuttake(),
                       m_outtake))
                .whileFalse(new RunCommand(
                         () -> m_outtake.stopOuttake(), m_outtake));

        // elevator up control
 

        new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                .whileTrue(m_elevator.elevatorDownStop());
        

        new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                .whileTrue(m_elevator.elevatorUpStop());

                
        new JoystickButton(m_psoc,4)
                .onTrue(Commands.runOnce(() -> m_elevator.setElevatorL0(), m_elevator));
        new JoystickButton(m_psoc,5)
                .onTrue(Commands.runOnce(() -> m_elevator.setElevatorL1(), m_elevator));
        new JoystickButton(m_psoc,6)
                .onTrue(Commands.runOnce(() -> m_elevator.setElevatorL2(), m_elevator));
        new JoystickButton(m_psoc,7)
                .onTrue(Commands.runOnce(() -> m_elevator.setElevatorL3(), m_elevator));
        new JoystickButton(m_psoc,8)
                .onTrue(Commands.runOnce(() -> m_elevator.setElevatorL4(), m_elevator));


        // new JoystickButton(m_psoc,5)
        //          .onTrue( new PathPlannerAuto("Example Auto"));

        // new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        //         .onTrue(Commands.runOnce(m_robotDrive::zeroHeading));
        

    //motion magic setup code 
    
        // new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        //         .onTrue(Commands.runOnce(() -> { SignalLogger.start(); System.out.println("Logging started clrkio");}));
        // new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        //         .onTrue(Commands.runOnce(() -> { SignalLogger.stop();}));
        
        // new JoystickButton(m_driverController, XboxController.Button.kA.value)
        //         .whileTrue(m_elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

        // new JoystickButton(m_driverController, XboxController.Button.kB.value)
        //         .whileTrue(m_elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        // new JoystickButton(m_driverController, XboxController.Button.kX.value)
        //         .whileTrue(m_elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));

        // new JoystickButton(m_driverController, XboxController.Button.kY.value)
        //         .whileTrue(m_elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    private final Command autoCommand = new PathPlannerAuto("Example Auto");



    public Command getAutonomousCommand() {
        return autoChooser.getSelected();


        // return autoCommand.andThen(Commands.runOnce(() -> {System.out.println("finished auto");}));


        // // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(
        //         AutoConstants.kMaxSpeedMetersPerSecond,
        //         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //         // Add kinematics to ensure max speed is actually obeyed
        //         .setKinematics(DriveConstants.kDriveKinematics);

        // // An example trajectory to follow. All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //         // Start at the origin facing the +X direction
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         // Pass through these two interior waypoints, making an 's' curve path
        //         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        //         // End 3 meters straight ahead of where we started, facing forward
        //         new Pose2d(3, 0, new Rotation2d(0)),
        //         config);

        // var thetaController = new ProfiledPIDController(
        //         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //         exampleTrajectory,
        //         m_robotDrive::getPose, // Functional interface to feed supplier
        //         DriveConstants.kDriveKinematics,

        //         // Position controllers
        //         new PIDController(AutoConstants.kPXController, 0, 0),
        //         new PIDController(AutoConstants.kPYController, 0, 0),
        //         thetaController,
        //         m_robotDrive::setModuleStates,
        //         m_robotDrive);

        // // Reset odometry to the starting pose of the trajectory.
        // m_robotDrive.resetPose(exampleTrajectory.getInitialPose());

        // // Run path following command, then stop at the end.
        // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));

    }
}
