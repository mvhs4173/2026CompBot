// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

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
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;

public class DriveBase extends SubsystemBase {
  SwerveDriveKinematics m_swerveDriveKinematics;
  MAXSwerveModule[] m_modules = new MAXSwerveModule[4];
  private final Pigeon2 m_pigeon = new Pigeon2(DrivetrainConstants.pigeonID, "rio");
  // Pigeon is on roboRIO CAN Bus with device ID 1

  private final AHRS m_navX = new AHRS(NavXComType.kMXP_SPI);

  private SlewRateLimiter m_translationLimiter = new SlewRateLimiter(OperatorConstants.translationLimit);
  // will be constants
  private SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(OperatorConstants.rotationLimit);

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
      m_modules[i] = new MAXSwerveModule(DrivetrainConstants.SwerveModules.values()[i]);
    }
    m_swerveDriveKinematics = new SwerveDriveKinematics(
        DrivetrainConstants.SwerveModules.frontLeft.wheelPos,
        DrivetrainConstants.SwerveModules.frontRight.wheelPos,
        DrivetrainConstants.SwerveModules.backLeft.wheelPos,
        DrivetrainConstants.SwerveModules.backRight.wheelPos);

    resetGyro();

    m_sysIdConfig = new Config(DrivetrainConstants.kSysIdRampRate, DrivetrainConstants.kStepVoltage,
        DrivetrainConstants.kTimeout);

    m_sysIdMechanism = new Mechanism(this::applyVolts, log -> {
      log.motor("drive-front-left")
          .voltage(
              m_appliedVoltage.mut_replace(
                  m_modules[0].getVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(m_modules[0].getDistanceMeters(),
              Meters))
          .linearVelocity(
              m_velocity.mut_replace(
                  m_modules[0].getSpeedMetersPerSecond(),
                  MetersPerSecond));
      log.motor("drive-front-right")
          .voltage(
              m_appliedVoltage.mut_replace(
                  m_modules[1].getVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(m_modules[1].getDistanceMeters(),
              Meters))
          .linearVelocity(
              m_velocity.mut_replace(
                  m_modules[1].getSpeedMetersPerSecond(),
                  MetersPerSecond));
      log.motor("drive-back-left")
          .voltage(
              m_appliedVoltage.mut_replace(
                  m_modules[2].getVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(m_modules[2].getDistanceMeters(),
              Meters))
          .linearVelocity(
              m_velocity.mut_replace(
                  m_modules[2].getSpeedMetersPerSecond(),
                  MetersPerSecond));
      log.motor("drive-back-right")
          .voltage(
              m_appliedVoltage.mut_replace(
                  m_modules[3].getVoltage(), Volts))
          .linearPosition(m_distance.mut_replace(m_modules[3].getDistanceMeters(),
              Meters))
          .linearVelocity(
              m_velocity.mut_replace(
                  m_modules[3].getSpeedMetersPerSecond(),
                  MetersPerSecond));
    }, this, "sysIdRoutine");

    m_sysIdRoutine = new SysIdRoutine(m_sysIdConfig, m_sysIdMechanism);
  }

  public void applyVolts(Voltage v) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].setSwerveAngle(new Rotation2d());
      m_modules[i].setDriveVoltage(v.in(Volts));
    }
  }

  public void resetGyro() {
    if (Constants.DrivetrainConstants.kUsePigeon) {
      m_pigeon.reset();
    } else {
      m_navX.reset();
    }
  }

  public Rotation2d getAngle() {
    return Constants.DrivetrainConstants.kUsePigeon ? m_pigeon.getRotation2d() : m_navX.getRotation2d();
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
    double rotateVelocity = boost ? rotate * DrivetrainConstants.maxRotationSpeed
        : rotate * OperatorConstants.rotationNormalSpeed; // same^
    ChassisSpeeds speeds = new ChassisSpeeds(forwardVelocity, sideVelocity, rotateVelocity);
    SmartDashboard.putNumber("Intended Forward Speed", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Intended Sideway Speed", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Intended Rotation Speed", speeds.omegaRadiansPerSecond);
    if (fieldOrient) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle()); // needs function to get angle from pig
    }
    double movementAngle = Math.atan2(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double linearVelocity = Math.sqrt((Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2))); // pythagorean
                                                                                                                        // theorem
    linearVelocity = m_translationLimiter.calculate(linearVelocity);
    // rotateVelocity = m_rotationLimiter.calculate(rotateVelocity);
    forwardVelocity = Math.sin(movementAngle) * linearVelocity;
    sideVelocity = Math.cos(movementAngle) * linearVelocity;
    speeds = new ChassisSpeeds(forwardVelocity, sideVelocity, rotateVelocity);
    applySpeeds(speeds);
  }

  public void applySpeeds(ChassisSpeeds speeds) {
    SmartDashboard.putNumber("Forward Speed", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Sideway Speed", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Rotation Speed", speeds.omegaRadiansPerSecond);
    SwerveModuleState[] states = m_swerveDriveKinematics.toWheelSpeeds(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.maxSpeed);
    for (int i = 0; i < 4; i++) {
      // apply speeds to each wheel
      m_modules[i].setDesiredState(states[i]);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Front Left Angle Degrees", m_modules[0].getState().angle.getDegrees());
    SmartDashboard.putNumber("Front Right Angle Degrees", m_modules[1].getState().angle.getDegrees());
    SmartDashboard.putNumber("Back Left Angle Degrees", m_modules[2].getState().angle.getDegrees());
    SmartDashboard.putNumber("Back Right Angle Degrees", m_modules[3].getState().angle.getDegrees());
  }
}
