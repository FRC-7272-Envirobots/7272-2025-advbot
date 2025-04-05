// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LightstripEnvirobots;
import frc.robot.commands.Routines;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Lightstrip;
import frc.robot.subsystems.OuttakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

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
        private final AlgaeSubsystem m_algae = new AlgaeSubsystem();
        private final Climber m_climber = new Climber();
        private final Lightstrip m_Lightstrip0 = new Lightstrip(0);
        private final Routines m_Routines = new Routines(m_elevator, m_outtake, m_robotDrive);
        // private final Lightstrip m_Lightstrip1 = new Lightstrip(1);
        // The driver's controller
        public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
        Joystick m_psoc = new Joystick(1);
        private boolean m_fieldRelative = true;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {

                m_Lightstrip0.setDefaultCommand(new LightstripEnvirobots(m_Lightstrip0));
                // m_Lightstrip1.setDefaultCommand(new LightstripEnvirobots(m_Lightstrip1));
                // Configure the button bindings
                configureButtonBindings();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband),
                                                                m_fieldRelative),
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
                // new JoystickButton(m_psoc,4)
                // .whileTrue(new RunCommand(
                // () -> m_robotDrive.setX(),
                // m_robotDrive));

                // CONTROLLER BUTTONS

                // Algae buttons
                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(new RunCommand(() -> m_algae.algaeup(), m_algae)
                                                .andThen(() -> m_algae.alagespin()))

                                .whileFalse(new RunCommand(() -> m_algae.armstop(), m_algae));
                // .whileFalse(new RunCommand(() -> m_algae.algaestop(), m_algae));

                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                // .whileTrue(new RunCommand(() -> m_algae.algaedown(), m_algae))
                                // .whileFalse(new RunCommand(() -> m_algae.armstop(), m_algae));
                                .whileTrue(new RunCommand(() -> m_fieldRelative = false))
                                .whileFalse(new RunCommand(() -> m_fieldRelative = true));

                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(new RunCommand(() -> m_algae.alagespin(), m_algae))
                                .whileFalse(new RunCommand(() -> m_algae.algaestop(), m_algae));

                // outtake button
                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(new RunCommand(() -> m_outtake.normalOuttake(), m_outtake))
                                .whileFalse(new RunCommand(() -> m_outtake.stopOuttake(), m_outtake));

                // BUTTON BOARD BUTTONS

                // elevator up control

                new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                                .whileTrue(m_elevator.elevatorDownStop());

                new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                                .whileTrue(m_elevator.elevatorUpStop());

                new JoystickButton(m_psoc, 1)
                                .onTrue(Commands.run(() -> m_elevator.setElevatorL0(), m_elevator));
                // new JoystickButton(m_psoc,)
                // .onTrue(Commands.runOnce(() -> m_elevator.setElevatorL1(), m_elevator));
                new JoystickButton(m_psoc, 2)
                                .onTrue(Commands.run(() -> m_elevator.setElevatorL2(), m_elevator));
                new JoystickButton(m_psoc, 3)
                                .onTrue(Commands.run(() -> m_elevator.setElevatorL3(), m_elevator));
                new JoystickButton(m_psoc, 4)
                                .onTrue(Commands.run(() -> m_elevator.setElevatorL4(), m_elevator));
                 new JoystickButton(m_psoc, 5)
                .onTrue(Commands.run(() -> m_robotDrive.zeroHeading(),m_robotDrive));
                // new JoystickButton(m_psoc, 5)
                // .onTrue(Commands.run(() -> m_algae.algaeup(), m_algae));
                // new JoystickButton(m_psoc, 6)
                // .onTrue(Commands.run(() -> m_algae.algaedown(), m_algae));

                // new JoystickButton(m_psoc, 7)
                // .onTrue(Commands.run(() -> m_elevator.elevatorUpStop(), m_elevator));

                // new JoystickButton(m_psoc, 8)
                // .onTrue(Commands.run(() -> m_elevator.elevatorDownStop(), m_elevator));

                new JoystickButton(m_psoc, 10)
                                .onTrue(new PrintCommand("")
                                                .andThen(m_robotDrive.driveTo(AutoDestination.CORAL_LEFT_1)));
                new JoystickButton(m_psoc, 11)
                                .onTrue(new PrintCommand("")
                                                .andThen(m_robotDrive.driveTo(AutoDestination.CORAL_LEFT_2)));
                new JoystickButton(m_psoc, 12)
                                .onTrue(new PrintCommand("")
                                                .andThen(m_robotDrive.driveTo(AutoDestination.CORAL_RIGHT_1)));
                new JoystickButton(m_psoc, 13)
                                .onTrue(new PrintCommand("")
                                                .andThen(m_robotDrive.driveTo(AutoDestination.CORAL_RIGHT_2)));

                new JoystickButton(m_psoc, 14)
                                .onTrue(new PrintCommand("")
                                                .andThen(m_robotDrive.driveTo(AutoDestination.REEF_6_RIGHT)));

                new JoystickButton(m_psoc, 15)
                                .onTrue(new PrintCommand("")
                                                .andThen(m_robotDrive.driveTo(AutoDestination.REEF_6_LEFT)));

                new JoystickButton(m_psoc, 29)
                                .onTrue(new PrintCommand("")
                                                .andThen(m_robotDrive.driveTo(AutoDestination.REEF_8_RIGHT)));

                new JoystickButton(m_psoc, 17)
                                .onTrue(new PrintCommand("")
                                                .andThen(m_robotDrive.driveTo(AutoDestination.REEF_8_LEFT)));

                new JoystickButton(m_psoc, 18)
                                .onTrue(new PrintCommand("")
                                                .andThen(m_robotDrive.driveTo(AutoDestination.REEF_10_RIGHT)));

                new JoystickButton(m_psoc, 19)
                                .onTrue(new PrintCommand("")
                                                .andThen(m_robotDrive.driveTo(AutoDestination.REEF_12_RIGHT)));

                new JoystickButton(m_psoc, 20)
                                .onTrue(new PrintCommand("")
                                                .andThen(m_robotDrive.driveTo(AutoDestination.REEF_12_LEFT)));

                new JoystickButton(m_psoc, 21)
                                .onTrue(new PrintCommand("")
                                                .andThen(m_robotDrive.driveTo(AutoDestination.REEF_2_RIGHT)));

                new JoystickButton(m_psoc, 22)
                                .onTrue(new PrintCommand("")
                                                .andThen(m_robotDrive.driveTo(AutoDestination.REEF_2_LEFT)));

                new JoystickButton(m_psoc, 23)
                                .onTrue(new PrintCommand("")
                                                .andThen(m_robotDrive.driveTo(AutoDestination.REEF_4_RIGHT)));

                new JoystickButton(m_psoc, 24)
                                .onTrue(new PrintCommand("")
                                                .andThen(m_robotDrive.driveTo(AutoDestination.REEF_4_LEFT)));

                new JoystickButton(m_psoc, 27)
                                .onTrue(new RunCommand(() -> m_Routines.Systemtest(), m_elevator, m_outtake,
                                                m_robotDrive));

                // new JoystickButton(m_psoc, 7)
                // .onTrue(Commands.run(()->m_robotDrive., null));

                // new JoystickButton(m_psoc,13)
                // .onTrue( new PathPlannerAuto("Example Auto"));

                // new JoystickButton(m_driverController,
                // XboxController.Button.kLeftBumper.value)
                // .onTrue(Commands.runOnce(m_robotDrive::zeroHeading));

                // motion magic setup code

                // new JoystickButton(m_driverController,
                // XboxController.Button.kLeftBumper.value)
                // .onTrue(Commands.runOnce(() -> { SignalLogger.start();
                // System.out.println("Logging started clrkio");}));
                // new JoystickButton(m_driverController,
                // XboxController.Button.kRightBumper.value)
                // .onTrue(Commands.runOnce(() -> { SignalLogger.stop();}));

                // new JoystickButton(m_driverController, XboxController.Button.kA.value)
                // .whileTrue(m_elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

                // new JoystickButton(m_driverController, XboxController.Button.kB.value)
                // .whileTrue(m_elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

                // new JoystickButton(m_driverController, XboxController.Button.kX.value)
                // .whileTrue(m_elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));

                // new JoystickButton(m_driverController, XboxController.Button.kY.value)
                // .whileTrue(m_elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

        private final Command autoCommand = new PathPlannerAuto("Example Auto");

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();

                // return autoCommand.andThen(Commands.runOnce(() ->
                // {System.out.println("finished auto");}));

                // // Create config for trajectory
                // TrajectoryConfig config = new TrajectoryConfig(
                // AutoConstants.kMaxSpeedMetersPerSecond,
                // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // // Add kinematics to ensure max speed is actually obeyed
                // .setKinematics(DriveConstants.kDriveKinematics);

                // // An example trajectory to follow. All units in meters.
                // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // // Start at the origin facing the +X direction
                // new Pose2d(0, 0, new Rotation2d(0)),
                // // Pass through these two interior waypoints, making an 's' curve path
                // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // // End 3 meters straight ahead of where we started, facing forward
                // new Pose2d(3, 0, new Rotation2d(0)),
                // config);

                // var thetaController = new ProfiledPIDController(
                // AutoConstants.kPThetaController, 0, 0,
                // AutoConstants.kThetaControllerConstraints);
                // thetaController.enableContinuousInput(-Math.PI, Math.PI);

                // SwerveControllerCommand swerveControllerCommand = new
                // SwerveControllerCommand(
                // exampleTrajectory,
                // m_robotDrive::getPose, // Functional interface to feed supplier
                // DriveConstants.kDriveKinematics,

                // // Position controllers
                // new PIDController(AutoConstants.kPXController, 0, 0),
                // new PIDController(AutoConstants.kPYController, 0, 0),
                // thetaController,
                // m_robotDrive::setModuleStates,
                // m_robotDrive);

                // // Reset odometry to the starting pose of the trajectory.
                // m_robotDrive.resetPose(exampleTrajectory.getInitialPose());

                // // Run path following command, then stop at the end.
                // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
                // false));

        }
}
