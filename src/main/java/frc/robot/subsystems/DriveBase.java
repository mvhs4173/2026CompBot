// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Gyro;
import java.util.Arrays;
import java.util.List;

public class DriveBase extends SubsystemBase implements Sendable {

  public SwerveDriveKinematics m_swerveDriveKinematics;
  MAXSwerveModule[] m_modules = new MAXSwerveModule[4];
  // Pigeon is on roboRIO CAN Bus with device ID 1

  private final Gyro m_gyro = new Gyro(DrivetrainConstants.kUsePigeon);
  private SwerveDriveOdometry m_odometer;
  private Field2d m_field = new Field2d();

  private SlewRateLimiter m_translationLimiter = new SlewRateLimiter(
    OperatorConstants.translationLimit
  );
  // will be constants
  private SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(
    OperatorConstants.rotationLimit
  );

  TrajectoryConfig m_trajConfig = new TrajectoryConfig(
    LinearVelocity.ofBaseUnits(2, MetersPerSecond),
    LinearAcceleration.ofBaseUnits(4, MetersPerSecondPerSecond)
  );

  private Config m_sysIdConfig;
  private Mechanism m_sysIdMechanism;
  private SysIdRoutine m_sysIdRoutine;

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  /** Creates a new DriveBase. */
  public DriveBase() {
    for (int i = 0; i < 4; i++) {
      m_modules[i] = new MAXSwerveModule(
        DrivetrainConstants.SwerveModules.values()[i]
      );
    }
    m_swerveDriveKinematics = new SwerveDriveKinematics(
      DrivetrainConstants.SwerveModules.frontLeft.wheelPos,
      DrivetrainConstants.SwerveModules.frontRight.wheelPos,
      DrivetrainConstants.SwerveModules.backLeft.wheelPos,
      DrivetrainConstants.SwerveModules.backRight.wheelPos
    );

    resetGyro();

    m_odometer = new SwerveDriveOdometry(
      m_swerveDriveKinematics,
      new Rotation2d(),
      getModulePositions()
    );

    m_sysIdConfig = new Config(
      DrivetrainConstants.kSysIdRampRate,
      DrivetrainConstants.kStepVoltage,
      DrivetrainConstants.kTimeout
    );

    m_sysIdMechanism = new Mechanism(
      this::applyVolts,
      log -> {
        log
          .motor("drive-front-left")
          .voltage(
            m_appliedVoltage.mut_replace(m_modules[0].getVoltage(), Volts)
          )
          .linearPosition(
            m_distance.mut_replace(m_modules[0].getDistanceMeters(), Meters)
          )
          .linearVelocity(
            m_velocity.mut_replace(
              m_modules[0].getSpeedMetersPerSecond(),
              MetersPerSecond
            )
          );
        log
          .motor("drive-front-right")
          .voltage(
            m_appliedVoltage.mut_replace(m_modules[1].getVoltage(), Volts)
          )
          .linearPosition(
            m_distance.mut_replace(m_modules[1].getDistanceMeters(), Meters)
          )
          .linearVelocity(
            m_velocity.mut_replace(
              m_modules[1].getSpeedMetersPerSecond(),
              MetersPerSecond
            )
          );
        log
          .motor("drive-back-left")
          .voltage(
            m_appliedVoltage.mut_replace(m_modules[2].getVoltage(), Volts)
          )
          .linearPosition(
            m_distance.mut_replace(m_modules[2].getDistanceMeters(), Meters)
          )
          .linearVelocity(
            m_velocity.mut_replace(
              m_modules[2].getSpeedMetersPerSecond(),
              MetersPerSecond
            )
          );
        log
          .motor("drive-back-right")
          .voltage(
            m_appliedVoltage.mut_replace(m_modules[3].getVoltage(), Volts)
          )
          .linearPosition(
            m_distance.mut_replace(m_modules[3].getDistanceMeters(), Meters)
          )
          .linearVelocity(
            m_velocity.mut_replace(
              m_modules[3].getSpeedMetersPerSecond(),
              MetersPerSecond
            )
          );
      },
      this,
      "sysIdRoutine"
    );

    m_sysIdRoutine = new SysIdRoutine(m_sysIdConfig, m_sysIdMechanism);
  }

  public void applyVolts(Voltage v) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].setSwerveAngle(new Rotation2d());
      m_modules[i].setDriveVoltage(v.in(Volts));
    }
  }

  public void resetGyro() {
    m_gyro.resetGyro();
  }

  public Rotation2d getAngle() {
    return m_gyro.getAngle();
  }

  private double getAngleDegrees() {
    return getAngle().getDegrees();
  }

  public void userDrive(
    double forward,
    double side,
    double rotate,
    boolean fieldOrient,
    boolean boost
  ) {
    forward = Math.pow(forward, 3);
    side = Math.pow(side, 3);
    rotate = Math.pow(rotate, 3);

    convertSpeeds(forward, side, rotate, fieldOrient, boost);
  }

  private void convertSpeeds(
    double forward,
    double side,
    double rotate,
    boolean fieldOrient,
    boolean boost
  ) {
    double forwardVelocity = boost
      ? forward * DrivetrainConstants.maxSpeed
      : forward * OperatorConstants.normalSpeed;
    // convert % speed to MPS and apply boost
    double sideVelocity = boost
      ? side * DrivetrainConstants.maxSpeed
      : side * OperatorConstants.normalSpeed; // same ^
    double rotateVelocity = boost
      ? rotate * DrivetrainConstants.maxRotationSpeed
      : rotate * OperatorConstants.rotationNormalSpeed; // same^
    ChassisSpeeds speeds = new ChassisSpeeds(
      forwardVelocity,
      sideVelocity,
      rotateVelocity
    );
    SmartDashboard.putNumber(
      "Intended Forward Speed",
      speeds.vxMetersPerSecond
    );
    SmartDashboard.putNumber(
      "Intended Sideway Speed",
      speeds.vyMetersPerSecond
    );
    SmartDashboard.putNumber(
      "Intended Rotation Speed",
      speeds.omegaRadiansPerSecond
    );
    if (fieldOrient) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle()); // needs function to get angle from pig
    }
    double movementAngle = Math.atan2(
      speeds.vxMetersPerSecond,
      speeds.vyMetersPerSecond
    );
    double linearVelocity = Math.hypot(
      speeds.vxMetersPerSecond,
      speeds.vyMetersPerSecond
    ); // pythagorean theorem
    linearVelocity = linearVelocity < Constants.OperatorConstants.kTolerance
      ? 0
      : linearVelocity;
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
    SwerveDriveKinematics.desaturateWheelSpeeds(
      states,
      DrivetrainConstants.maxSpeed
    );
    applyModuleStates(states);
  }

  public void applyModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < 4; i++) {
      // apply speeds to each wheel
      m_modules[i].setDesiredState(states[i]);
    }
    m_odometer.update(getAngle(), getModulePositions());
    m_field.setRobotPose(getPose());
  }

  /*
   * Temporary Mentor code for testing FIXME
   */
  private SwerveModulePosition[] getModulePositions() {
    return (SwerveModulePosition[]) Arrays.stream(m_modules)
      .map(MAXSwerveModule::getPosition)
      .toArray();
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("Field", m_field);
  }

  public Pose2d getPose() {
    return m_odometer.getPoseMeters();
  }

  public Trajectory constructTrajectory(Pose2d end) {
    return constructTrajectory(getPose(), end);
  }

  public Trajectory constructTrajectory(Pose2d start, Pose2d end) {
    return TrajectoryGenerator.generateTrajectory(
      start,
      List.of(),
      end,
      m_trajConfig
    );
  }

  public SwerveControllerCommand followTrajectoryCommand(
    Trajectory trajectory
  ) {
    return new SwerveControllerCommand(
      trajectory,
      this::getPose,
      m_swerveDriveKinematics,
      null, //FIXME: Create a holonomic drive controller object with pid constants.
      this::applyModuleStates,
      this
    );
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModule");
    builder.addDoubleProperty("gyroAngle", this::getAngleDegrees, null);
    builder.addDoubleProperty("FL Heading", m_modules[0]::getWheelAngle, null);
    builder.addDoubleProperty("FR Heading", m_modules[1]::getWheelAngle, null);
    builder.addDoubleProperty("BL Heading", m_modules[2]::getWheelAngle, null);
    builder.addDoubleProperty("BR Heading", m_modules[3]::getWheelAngle, null);
  }
}
