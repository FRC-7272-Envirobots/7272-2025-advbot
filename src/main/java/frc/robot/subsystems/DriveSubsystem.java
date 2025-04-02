// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.AutoDestination;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  public final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_DrivePoseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getGyroHeading()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition() },
      Pose2d.kZero,
      VecBuilder.fill(0.05, 0.05, .05),
      VecBuilder.fill(0.5, 0.5, .5));

  private final Field2d drivefield = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    // double robotYaw = -m_gyro.getYaw();
    // LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    SmartDashboard.putData("Field", drivefield);

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      drivefield.getObject("path").setPoses(poses);
    });
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      drivefield.setRobotPose(pose);
    });
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> drivefield.getObject("target pose").setPose(pose));

    try {
      RobotConfig config = new RobotConfig(
          Constants.DriveConstants.MASS_KG,
          Constants.DriveConstants.MOMENT_OF_INERTIA,
          new ModuleConfig(
              Constants.ModuleConstants.kWheelDiameterMeters / 2,
              Constants.DriveConstants.kMaxSpeedMetersPerSecond,
              Constants.ModuleConstants.WHEEL_COEFFICIENT_OF_FRICTION,
              DCMotor.getNEO(1),
              Constants.ModuleConstants.kDrivingMotorReduction,
              Constants.ModuleConstants.DRIVE_CURRENT_LIMIT,
              1),
          Constants.DriveConstants.moduleTranslations);

      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> driveChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                                // ChassisSpeeds. Also optionally outputs individual
                                                                // module feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                          // holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0), // changed from 5 Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
          ),
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return false;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
      );
      System.out.println("configured AutoBuilder");
    } catch (Exception e) {
      System.out.println("AUTOBUILDER FAILED");

      // Handle exception as needed
      e.printStackTrace();
    }

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_DrivePoseEstimator.update(
        Rotation2d.fromDegrees(getGyroHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    // boolean doRejectUpdate = false;

    // System.out.println("outtake robot camera pose settings: \n" +
    // Constants.VisionConstants.OUTTAKE_LIMELIGHT_X_OFFSET.in(Meters) + " m x\n" +
    // Constants.VisionConstants.OUTTAKE_LIMELIGHT_Y_OFFSET.in(Meters) + " m y\n" +
    // Constants.VisionConstants.OUTTAKE_LIMELIGHT_Z_OFFSET.in(Meters) + " m z\n" +
    // Constants.VisionConstants.OUTTAKE_LIMELIGHT_ROLL_ANGLE.in(Degrees) + " deg
    // roll \n" +
    // Constants.VisionConstants.OUTTAKE_LIMELIGHT_PITCH_ANGLE.in(Degrees) + " deg
    // pitch\n" +
    // Constants.VisionConstants.OUTTAKE_LIMELIGHT_YAW_ANGLE.in(Degrees) + " deg
    // yaw"
    // );

    LimelightHelpers.SetRobotOrientation(Constants.VisionConstants.INTAKE_LIMELIGHT_NAME, getEstimatedHeading(), 0, 0,
        0, 0, 0);
    LimelightHelpers.SetRobotOrientation(Constants.VisionConstants.OUTTAKE_LIMELIGHT_NAME, getEstimatedHeading(), 0, 0,
        0, 0, 0);

    LimelightHelpers.PoseEstimate intakePoseEstimate = LimelightHelpers
        .getBotPoseEstimate_wpiBlue(Constants.VisionConstants.INTAKE_LIMELIGHT_NAME);
    LimelightHelpers.PoseEstimate outtakePoseEstimate = LimelightHelpers
        .getBotPoseEstimate_wpiBlue(Constants.VisionConstants.OUTTAKE_LIMELIGHT_NAME);

    LimelightHelpers.PoseEstimate bestEstimate = pickBestPoseEstimate(filterPoseEstimate(intakePoseEstimate),
        (filterPoseEstimate(outtakePoseEstimate)));

    if (bestEstimate == null || Math.abs(getGyroYawRate()) > 720) {
      // skip pose estimate
    } else {
      m_DrivePoseEstimator.addVisionMeasurement(
          bestEstimate.pose,
          bestEstimate.timestampSeconds);
    }
    drivefield.setRobotPose(m_DrivePoseEstimator.getEstimatedPosition());

  }

  private static LimelightHelpers.PoseEstimate filterPoseEstimate(LimelightHelpers.PoseEstimate pose) {

    if (pose == null) {
      return null;
    }
    if (pose.tagCount == 0) {
      return null;
    } else {
      boolean hastagcloseenough = false;
      for (RawFiducial tag : pose.rawFiducials) {
        // .println(tag.distToRobot)System.out;
        if (tag.distToRobot < 3.48) {
          hastagcloseenough = true;
        }
      }
      if (!hastagcloseenough) {
        return null;
      }
    }

    return pose;
  }

  // returns null if no pose estimate should be used, or returns the best pose
  // estimate to use
  private static LimelightHelpers.PoseEstimate pickBestPoseEstimate(LimelightHelpers.PoseEstimate pose1,
      LimelightHelpers.PoseEstimate pose2) {
    if ((pose1 == null || pose1.tagCount == 0) && (pose2 == null || pose2.tagCount == 0)) {
      // System.out.println("Skipping vision estimate because both are null");
      return null;
    }

    if (pose1 == null || pose1.tagCount == 0) {
      // System.out.println("using outtake camera");
      return pose2;
    }

    if (pose2 == null || pose2.tagCount == 0) {
      // System.out.println("using intake camera");
      return pose1;
    }

    if (pose1.avgTagDist < pose2.avgTagDist) {
      System.out.println("using intake camera because it is closer");
      return pose1;
    } else {
      System.out.println("using outtake camera because it is closer");
      return pose2;
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_DrivePoseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    m_DrivePoseEstimator.resetPosition(
        Rotation2d.fromDegrees(getGyroHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond * DriveConstants.DRIVE_LIMITER;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond * DriveConstants.DRIVE_LIMITER;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
            Rotation2d.fromDegrees(getEstimatedHeading()))
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    driveChassisSpeeds(chassisSpeeds);

  }

  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    // System.out.println(chassisSpeeds);
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public ChassisSpeeds getChassisSpeeds() {

    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    System.out.println("gyro reset");
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getGyroHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle() * -1).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getGyroYawRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getEstimatedHeading() {
    return m_DrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees();
  }

  List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
      new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
      new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90)));

  public static boolean alliance;

  // public PathConstraints(double maxVelocityMPS, double maxAccelerationMPSSq,
  // double maxAngularVelocityRadPerSec, double maxAngularAccelerationRadPerSecSq)
  // {
  // this(maxVelocityMPS, maxAccelerationMPSSq, maxAngularVelocityRadPerSec,
  // maxAngularAccelerationRadPerSecSq, 12.0, false);
  public Command driveTo(AutoDestination autoDriveto) {

    System.out.println("starting driveTo");

    if (DriverStation.getRawAllianceStation() == AllianceStationID.Red1) {
      alliance = false;
    }
    if (DriverStation.getRawAllianceStation() == AllianceStationID.Red2) {
      alliance = false;
    }
    if (DriverStation.getRawAllianceStation() == AllianceStationID.Red3) {
      alliance = false;
    } else {
      alliance = true;
    }

    if (alliance = false) {
      Pose2d chosen_auto = AutoConstants.RedAuto_Map.get(autoDriveto);
      System.out.println(chosen_auto);
      return AutoBuilder.pathfindToPose(chosen_auto, AutoConstants.defaultPathConstraints);
    }
    if (alliance = true) {
      Pose2d chosen_auto = AutoConstants.blueAuto_Map.get(autoDriveto);
      System.out.println(chosen_auto);
      return AutoBuilder.pathfindToPose(chosen_auto, AutoConstants.defaultPathConstraints);
    }
    return null;
  }

}
