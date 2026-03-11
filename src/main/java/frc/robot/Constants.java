// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

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

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double translationLimit = 1.0; //2.5 // for slewRateLimiters
    public static final double rotationLimit = 2.0;

    public static final double normalSpeed = 1; // Meters PS
    public static final double rotationNormalSpeed = 1; // Radians PS

    public static final double kTolerance = 0.15;
  }

  public static class DrivetrainConstants {

    public static final boolean kUsePigeon = false;

    public static final Velocity<VoltageUnit> kSysIdRampRate = null;
    public static final Voltage kStepVoltage = null;
    public static final Time kTimeout = null;

    public static final int pigeonID = 2;

    public static final double trackWidth = Units.inchesToMeters(28);
    public static final double wheelBase = Units.inchesToMeters(23);

    public static final double maxSpeed = 5.08257664976; // MPS Theoretical
    public static final double maxRotationSpeed = 11.0448342483; // Radians PS Theoretical

    public enum SwerveModules {
      frontLeft(
        new Translation2d(wheelBase / 2, trackWidth / 2),
        10,
        11,
        false,
        Rotation2d.fromDegrees(0)
      ),
      frontRight(
        new Translation2d(wheelBase / 2, -trackWidth / 2),
        20,
        21,
        false,
        Rotation2d.fromDegrees(0)
      ),
      backLeft(
        new Translation2d(-wheelBase / 2, trackWidth / 2),
        30,
        31,
        false,
        new Rotation2d(0)
      ),
      backRight(
        new Translation2d(-wheelBase / 2, -trackWidth / 2),
        40,
        41,
        false,
        new Rotation2d(0)
      );

      public Rotation2d offsetAngle;

      public Translation2d wheelPos;
      public int turnID;
      public int driveID;
      public boolean driveReversed;
      public double chassisAngularOffset;

      public double moduleAngularOffset;

      private SwerveModules(
        Translation2d wheelPos,
        int turnID,
        int driveID,
        boolean driveReversed,
        Rotation2d offsetAngle
      ) {
        this.wheelPos = wheelPos;
        this.turnID = turnID;
        this.driveID = driveID;
        this.driveReversed = driveReversed;
        this.offsetAngle = offsetAngle;
      }
    }
  }

  public static class IntakeConstants {

    public static final int kLeadIntakeDeploymentID = 50;
    public static final int kFollowIntakeDeploymentID = 51;
    public static final int kIntakeRunningID = 52;

    public static final int kDeployedLimitSwitchPort = 0; // DIO ports for limit switches
    public static final int kRetractedLimitSwitchPort = 1;

    // PID values
    private static final double kDeployPEstimate =
      (1.0 / 3.0) * (1.0 / 3.0) * 12;
    // 1.0/3.0 - meters to travel - 1.0/3.0 - 1/seconds travel time - 12 - Volts
    public static final double kDeployP = kDeployPEstimate; // Estimate of volts - 0 meters to 1/3 in 3 seconds
    public static final double kDeployI = 0;
    public static final double kDeployD = 0;

    // FF values
    public static final double kDeployS = 0;
    public static final double kDeployV = 0;
    public static final double kDeployA = 0;

    public static final double kRotationsToMeters = Units.inchesToMeters(
      0.5 / 1.0
    ); // Jackscrew rotations to meters
    // travelled
    public static final double kGearRatio = 1.0 / 3.0; // gear ratio
    private static final double kMotorMaxSpeedRPM = 11000; // NEO 550 free speed

    public static final double kMaxDeploySpeedMPS =
      ((kMotorMaxSpeedRPM * kGearRatio) / 60.0) * kRotationsToMeters;

    public static final double kDeployDistanceMeters = Units.inchesToMeters(
      13.25
    ); // 13.25 inches extension
    public static final double kDeployToleranceMeters = Units.inchesToMeters(
      1.0
    ); // 1 inch tolerance

    public static final double kDeploymentSpeed = 0.2; // % speed
    public static final double kRunningVolts = 8.0; // Volts

    public static final boolean kDeploymentInverted = false;

    public static final boolean kLeftDeployEncoderInverted = false;
    public static final boolean kRightDeployEncoderInverted = false;
  }

  public static class IndexerConstants {

    public static final int kLeadIndexMotorID = 53;
    public static final int kFollowIndexMotorID = 54;
    public static final int kTopRollerMotorID = 55;

    //PID values
    public static final double kIndexP = 20;
    public static final double kIndexI = 0;
    public static final double kIndexD = 0;

    public static final double kTopRollerP = 20;
    public static final double kTopRollerI = 0;
    public static final double kTopRollerD = 0;

    public static final double kPowerCorrectionP = 0; //TODO: figure this out if needed
    public static final double kPowerCorrectionI = 0;
    public static final double kPowerCorrectionD = 0;

    //Setpoint for PID controller //TODO: setpoint calculations
    public static final double kTopMotorMaxSpeed = 5760.0 / 5.0;
    public static final double kBottomMotorMaxSpeed = 5760.0 / 5.0;
    public static final double kBottomMotorVelocitySetpoint = (2880.0 / 5.0); //1/2 Max rpm / gear ratio
    public static final double kTopRollerVelocitySetpoint = (2880.0 / 3.0); //Same

    public static final double kVoltage = 3.0;
    public static final Velocity<VoltageUnit> kSysIdRampRate = null;
    public static final Voltage kStepVoltage = null;
    public static final Time kTimeout = null;
  }

  public static class ShooterConstants {

    public static final int kLeftHoodServoChannel = 0;
    public static final int kRightHoodServoChannel = 1;

    public static final Rotation2d kHoodMinimumAngle = Rotation2d.fromDegrees(
      -74.2
    ); //Angles roughly measured
    public static final Rotation2d kHoodMaximumAngle = Rotation2d.fromDegrees(
      -43
    );
    public static final double kHoodMaximumMechanicalExtension = 0.80; //Percent of extension to safely retract

    public static final int kLeadShooterMotorID = 57;
    public static final int kFollowShooterMotorID = 56;

    public static final double kMaxSpeed = 5676.0;

    public static final double kP = 0.5;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kShooterVelocitySetpoint = 1200; //Setpoint for PID controller //TODO: setpoint calculations

    public static final double kVoltage = 6.0;

    //Sysid shtuff

    public static final Velocity<VoltageUnit> kSysIdRampRate = null;
    public static final Voltage kStepVoltage = null;
    public static final Time kTimeout = null;
  }

  public static class AutoConstants {}

  public static class VisionConstants {}
}
