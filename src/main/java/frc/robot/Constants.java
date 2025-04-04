// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import java.util.Map;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final double MASS_KG = 31;
    public static final double MOMENT_OF_INERTIA = 6.883;

    public static final double DRIVE_LIMITER = 2;
    public static final double ELEVATOR_LIMITER = 60;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(27.0 - (2.0 * 1.75));
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(32.0 - (2.0 * 1.75));
    // Distance between front and back wheels on robot

    public static final Translation2d[] moduleTranslations = {
        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
    };

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(moduleTranslations);

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2.0;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2.0;

    // Falcon CAN IDs
    // RN MOSTLY JUST PLACE HOLDERS

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;
  }

  public static final class RobotConstants {

    public static final int RightElevartorCanId = 18;
    public static final int LeftElevatorCanId = 19;
    public static final int LeftOuttakeCanId = 20;
    public static final int RightOuttakeCanId = 21;

    public static final int IntakerCanId = 23;
    public static final int AlgaeRemoverArmCanId = 24;
    public static final int AlgaeRemoverSpinnerCanID = 25;
    public static final int ClimberCanId = 26;

  }

  public static final class VisionConstants {
    // public static final String OUTTAKE_LIMELIGHT_NAME = "limelight-outtake";
    // public static Distance OUTTAKE_LIMELIGHT_X_OFFSET = Distance.ofBaseUnits(14,
    // Inches); // front to back from center
    // public static Distance OUTTAKE_LIMELIGHT_Y_OFFSET = Distance.ofBaseUnits(3,
    // Inches); // left to right from center
    // public static Distance OUTTAKE_LIMELIGHT_Z_OFFSET = Distance.ofBaseUnits(9,
    // Inches);
    // public static Angle OUTTAKE_LIMELIGHT_ROLL_ANGLE = Angle.ofBaseUnits(0,
    // Degrees);
    // public static Angle OUTTAKE_LIMELIGHT_PITCH_ANGLE = Angle.ofBaseUnits(-45,
    // Degrees);
    // public static Angle OUTTAKE_LIMELIGHT_YAW_ANGLE = Angle.ofBaseUnits(0,
    // Degrees);
    public static final String OUTTAKE_LIMELIGHT_NAME = "limelight-outtake";
    public static Distance OUTTAKE_LIMELIGHT_X_OFFSET = Inches.of(16); // front to back from center
    public static Distance OUTTAKE_LIMELIGHT_Y_OFFSET = Inches.of(2); // left to right from center
    public static Distance OUTTAKE_LIMELIGHT_Z_OFFSET = Inches.of(7.5);
    public static Angle OUTTAKE_LIMELIGHT_ROLL_ANGLE = Degrees.of(0);
    public static Angle OUTTAKE_LIMELIGHT_PITCH_ANGLE = Degrees.of(0);
    public static Angle OUTTAKE_LIMELIGHT_YAW_ANGLE = Degrees.of(0);

    public static final String INTAKE_LIMELIGHT_NAME = "limelight-intake";
    public static Distance INTAKE_LIMELIGHT_X_OFFSET = Inches.of(-16); // front to back from center
    public static Distance INTAKE_LIMELIGHT_Y_OFFSET = Inches.of(-.5);
    public static Distance INTAKE_LIMELIGHT_Z_OFFSET = Inches.of(30);
    public static Angle INTAKE_LIMELIGHT_ROLL_ANGLE = Degrees.of(0);
    public static Angle INTAKE_LIMELIGHT_PITCH_ANGLE = Degrees.of(0);
    public static Angle INTAKE_LIMELIGHT_YAW_ANGLE = Degrees.of(180);

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double WHEEL_COEFFICIENT_OF_FRICTION = 0.7;
    public static final double DRIVE_CURRENT_LIMIT = 50;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final Map<AutoDestination, Pose2d> blueAuto_Map = Map.ofEntries(
        Map.entry(AutoDestination.REEF_12_LEFT, new Pose2d(6.059, 4.305, Rotation2d.fromDegrees(-84.685))),
        Map.entry(AutoDestination.REEF_12_RIGHT, new Pose2d(6.059, 4.305, Rotation2d.fromDegrees(-85.685))),
        Map.entry(AutoDestination.REEF_2_LEFT, new Pose2d(5.208, 2.641, Rotation2d.fromDegrees(-133.764))),
        Map.entry(AutoDestination.REEF_2_RIGHT, new Pose2d(5.492, 2.798, Rotation2d.fromDegrees(-133.764))),
        Map.entry(AutoDestination.REEF_4_LEFT, new Pose2d(3.769, 2.596, Rotation2d.fromDegrees(149.172))),
        Map.entry(AutoDestination.REEF_4_RIGHT, new Pose2d(3.999, 2.492, Rotation2d.fromDegrees(156.508))),
        Map.entry(AutoDestination.REEF_6_LEFT, new Pose2d(2.910, 4.066, Rotation2d.fromDegrees(-98.248))),
        Map.entry(AutoDestination.REEF_6_RIGHT, new Pose2d(2.888, 3.723, Rotation2d.fromDegrees(149.172))),
        Map.entry(AutoDestination.REEF_8_LEFT, new Pose2d(3.746, 5.409, Rotation2d.fromDegrees(-98.248))),
        Map.entry(AutoDestination.REEF_8_RIGHT, new Pose2d(3.514, 5.260, Rotation2d.fromDegrees(-98.248))),
        Map.entry(AutoDestination.REEF_10_LEFT, new Pose2d(5.328, 5.349, Rotation2d.fromDegrees(-117.440))),
        Map.entry(AutoDestination.REEF_10_RIGHT, new Pose2d(5.067, 5.521, Rotation2d.fromDegrees(-117.440))),

        Map.entry(AutoDestination.CORAL_LEFT_1, new Pose2d(1.492, 7.327, Rotation2d.fromDegrees(-117.440))),
        Map.entry(AutoDestination.CORAL_LEFT_2, new Pose2d(0.821, 6.842, Rotation2d.fromDegrees(-117.440))),
        Map.entry(AutoDestination.CORAL_RIGHT_1, new Pose2d(0.798, 1.208, Rotation2d.fromDegrees(-117.440))),
        Map.entry(AutoDestination.CORAL_RIGHT_2, new Pose2d(1.604, 0.611, Rotation2d.fromDegrees(-117.440)))

    );
    public static final Map<AutoDestination, Pose2d> RedAuto_Map = Map.ofEntries(
        Map.entry(AutoDestination.REEF_12_LEFT, new Pose2d(11.495, 4.010, Rotation2d.fromDegrees(-98.258))),
        Map.entry(AutoDestination.REEF_12_RIGHT, new Pose2d(11.480, 3.640, Rotation2d.fromDegrees(-98.258))),
        Map.entry(AutoDestination.REEF_2_LEFT, new Pose2d(12.384, 5.478, Rotation2d.fromDegrees(-117.440))),
        Map.entry(AutoDestination.REEF_2_RIGHT, new Pose2d(10.079, 1.135, Rotation2d.fromDegrees(5.932))),
        Map.entry(AutoDestination.REEF_4_LEFT, new Pose2d(13.881, 5.396, Rotation2d.fromDegrees(-98.248))),
        Map.entry(AutoDestination.REEF_4_RIGHT, new Pose2d(13.615, 5.552, Rotation2d.fromDegrees(-98.248))),
        Map.entry(AutoDestination.REEF_6_LEFT, new Pose2d(14.667, 4.077, Rotation2d.fromDegrees(-98.248))),
        Map.entry(AutoDestination.REEF_6_RIGHT, new Pose2d(14.667, 4.329, Rotation2d.fromDegrees(-98.248))),
        Map.entry(AutoDestination.REEF_8_LEFT, new Pose2d(13.830, 2.6248, Rotation2d.fromDegrees(-98.248))),
        Map.entry(AutoDestination.REEF_8_RIGHT, new Pose2d(14.126, 2.810, Rotation2d.fromDegrees(-98.248))),
        Map.entry(AutoDestination.REEF_10_LEFT, new Pose2d(12.273, 2.632, Rotation2d.fromDegrees(-117.440))),
        Map.entry(AutoDestination.REEF_10_RIGHT, new Pose2d(12.533, 2.483, Rotation2d.fromDegrees(-117.449))),

        Map.entry(AutoDestination.CORAL_LEFT_1, new Pose2d(16.110, 0.805, Rotation2d.fromDegrees(-117.440))),
        Map.entry(AutoDestination.CORAL_LEFT_2, new Pose2d(16.931, 1.313, Rotation2d.fromDegrees(-117.440))),
        Map.entry(AutoDestination.CORAL_RIGHT_1, new Pose2d(16.670, 6.842, Rotation2d.fromDegrees(-117.440))),
        Map.entry(AutoDestination.CORAL_RIGHT_2, new Pose2d(15.871, 7.425, Rotation2d.fromDegrees(-117.440)))

    );
    public static PathConstraints defaultPathConstraints = new PathConstraints(1.0, 1.0, .5 * Math.PI, 1 * Math.PI);

  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class FieldConstants {
    public static final Pose2d BLUE_REEF_NEARCENTER_LEFT_POLE_POSE = new Pose2d(2.911, 4.073,
        Rotation2d.fromDegrees(0));

  }
}
