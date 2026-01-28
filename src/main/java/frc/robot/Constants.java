// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final double translationLimit = 2.5; // for slewRateLimiters
    public static final double rotationLimit = 4;

    public static final double normalSpeed = 2; // Meters PS
    public static final double rotationNormalSpeed = 2.5; // Radians PS
  }

  public static class DrivetrainConstants {

    public static final boolean kUsePigeon = false;

    public static final Velocity<VoltageUnit> kSysIdRampRate = null;
    public static final Voltage kStepVoltage = null;
    public static final Time kTimeout = null;



    public static final int pigeonID = 2;

    public static final double trackWidth = Units.inchesToMeters(28);
    public static final double wheelBase = Units.inchesToMeters(23);

    public static final double maxSpeed = 5.08257664976; // MPS
    public static final double maxRotationSpeed = 11.0448342483; // Radians PS //11.0448342483

    public enum SwerveModules {
      frontLeft(new Translation2d(wheelBase / 2, trackWidth / 2), 10, 11, true, new Rotation2d(5.459)),
      frontRight(new Translation2d(wheelBase / 2, -trackWidth / 2), 20, 21, false, new Rotation2d(1.757)),
      backLeft(new Translation2d(-wheelBase / 2, trackWidth / 2), 30, 31, true, new Rotation2d(4.001)),
      backRight(new Translation2d(-wheelBase / 2, -trackWidth / 2), 40, 41, false, new Rotation2d(0.122));

      public Rotation2d offsetAngle;

      public Translation2d wheelPos;
      public int turnID;
      public int driveID;
      public boolean driveReversed;
      public double chassisAngularOffset;

      public double moduleAngularOffset;

      private SwerveModules(Translation2d wheelPos, int turnID, int driveID, boolean driveReversed,
          Rotation2d offsetAngle) {
        this.wheelPos = wheelPos;
        this.turnID = turnID;
        this.driveID = driveID;
        this.driveReversed = driveReversed;
        this.offsetAngle = offsetAngle;
      }
    }
  }

  public static class IntakeConstants {
    public static final int kIntakeDeploymentID = 50;
    public static final int kIntakeRunningID = 51;

    public static final int kDeployedLimitSwitchPort = 0; // DIO ports for limit switches
    public static final int kRetractedLimitSwitchPort = 1;

    public static final double kDeploymentSpeed = 0.2; //% speed
    public static final double kRunningVolts = 6.0; //Volts

    public static final boolean kDeploymentInverted = false;

  }

  public static class IndexerConstants {
    public static final int kIndexMotorID = 52;

    public static final double kVoltage = 3.0;
  }

  public static class ShooterConstants {
    public static final int kShooterMotorID = 53;

    public static final double kVoltage = 6.0;

  }

  public static class AutoConstants {

  }

  public static class VisionConstants {

  }

}
