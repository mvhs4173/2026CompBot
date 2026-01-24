// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;

public class DriveBase extends SubsystemBase {
  SwerveDriveKinematics swerveDriveKinematics;
  MAXSwerveModule[] modules = new MAXSwerveModule[4];
  private final Pigeon2 pigeon = new Pigeon2(DrivetrainConstants.pigeonID, "rio"); // Pigeon is on roboRIO CAN Bus with
                                                                                   // device ID 1
  private SlewRateLimiter translationLimiter = new SlewRateLimiter(DrivetrainConstants.translationLimit); // will be
                                                                                                          // constants
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(DrivetrainConstants.rotationLimit);

  private Config m_sysIdConfig;
  private Mechanism m_sysIdMechanism;
  private SysIdRoutine m_sysIdRoutine;
  private MecanumDriveOdometry m_odometry;

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  /** Creates a new DriveBase. */
  public DriveBase() {
    for (int i = 0; i < 4; i++) {
      modules[i] = new MAXSwerveModule(DrivetrainConstants.SwerveModules.values()[i]);
    }
    swerveDriveKinematics = new SwerveDriveKinematics(
        DrivetrainConstants.SwerveModules.frontLeft.wheelPos,
        DrivetrainConstants.SwerveModules.frontRight.wheelPos,
        DrivetrainConstants.SwerveModules.backLeft.wheelPos,
        DrivetrainConstants.SwerveModules.backRight.wheelPos);

    pigeon.reset();

    m_sysIdConfig = new Config(DrivetrainConstants.kSysIdRampRate, DrivetrainConstants.kStepVoltage,
        DrivetrainConstants.kTimeout);

    m_sysIdMechanism = new Mechanism(this::applyVolts, log -> {
      log.motor("drive-front-left")
          .voltage(
              m_appliedVoltage.mut_replace(
                  modules[0].getVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(modules[0].getDistanceMeters(),
              Meters))
          .linearVelocity(
              m_velocity.mut_replace(
                  modules[0].getSpeedMetersPerSecond(),
                  MetersPerSecond));
      log.motor("drive-front-right")
          .voltage(
              m_appliedVoltage.mut_replace(
                  modules[1].getVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(modules[1].getDistanceMeters(),
              Meters))
          .linearVelocity(
              m_velocity.mut_replace(
                  modules[1].getSpeedMetersPerSecond(),
                  MetersPerSecond));
      log.motor("drive-back-left")
          .voltage(
              m_appliedVoltage.mut_replace(
                  modules[2].getVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(modules[2].getDistanceMeters(),
              Meters))
          .linearVelocity(
              m_velocity.mut_replace(
                  modules[2].getSpeedMetersPerSecond(),
                  MetersPerSecond));
      log.motor("drive-back-right")
          .voltage(
              m_appliedVoltage.mut_replace(
                  modules[3].getVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(modules[3].getDistanceMeters(),
              Meters))
          .linearVelocity(
              m_velocity.mut_replace(
                  modules[3].getSpeedMetersPerSecond(),
                  MetersPerSecond));
    }, this, "sysIdRoutine");

    m_sysIdRoutine = new SysIdRoutine(m_sysIdConfig, m_sysIdMechanism);
  }

  public void applyVolts(Voltage v) {
    for (int i = 0; i < 4; i++) {
      modules[i].setSwerveAngle(new Rotation2d());
      modules[i].setDriveVoltage(v.in(Volts));
    }
  }

  public void resetGyro() {
    pigeon.reset();
  }

  public Rotation2d getAngle() {
    return pigeon.getRotation2d();
  }

  public void userDrive(double forward, double side, double rotate, boolean fieldOrient, boolean boost) {
    forward = Math.pow(forward, 3);
    side = Math.pow(side, 3);
    rotate = Math.pow(rotate, 3);

    convertSpeeds(forward, side, rotate, fieldOrient, boost);
  }

  private void convertSpeeds(double forward, double side, double rotate, boolean fieldOrient, boolean boost) {
    double forwardVelocity = boost ? forward * DrivetrainConstants.maxSpeed : forward * OperatorConstants.normalSpeed; // convert
                                                                                                                       // %
                                                                                                                       // speed
                                                                                                                       // to
                                                                                                                       // MPS
                                                                                                                       // and
                                                                                                                       // apply
                                                                                                                       // boost
    double sideVelocity = boost ? side * DrivetrainConstants.maxSpeed : side * OperatorConstants.normalSpeed; // same ^
    double rotateVelocity = boost ? rotate : rotate; // same^
    ChassisSpeeds speeds = new ChassisSpeeds(forwardVelocity, sideVelocity, rotateVelocity);
    if (fieldOrient) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle()); // needs function to get angle from pig
    }
    double movementAngle = Math.atan2(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double linearVelocity = Math.sqrt((Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2))); // pythagorean
                                                                                                                        // theorem
    linearVelocity = translationLimiter.calculate(linearVelocity);
    rotateVelocity = rotationLimiter.calculate(rotateVelocity);
    forwardVelocity = Math.sin(movementAngle) * linearVelocity;
    sideVelocity = Math.cos(movementAngle) * linearVelocity;
    speeds = new ChassisSpeeds(forwardVelocity, sideVelocity, rotateVelocity);
    applySpeeds(speeds);
  }
  //TODO: somewhere between userdrive and apply speeds the xyz's are mixed up
  public void applySpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] states = swerveDriveKinematics.toWheelSpeeds(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.maxSpeed);
    for (int i = 0; i < 4; i++) {
      // apply speeds 2 each wheel
      modules[i].setDesiredState(states[i]);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Front Left Angle Degrees", modules[0].getState().angle.getDegrees());
    SmartDashboard.putNumber("Front Right Angle Degrees", modules[1].getState().angle.getDegrees());
    SmartDashboard.putNumber("Back Left Angle Degrees", modules[2].getState().angle.getDegrees());
    SmartDashboard.putNumber("Back Right Angle Degrees", modules[3].getState().angle.getDegrees());
  }
}
