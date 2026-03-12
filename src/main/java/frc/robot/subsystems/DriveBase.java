package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Gyro;
import java.util.Arrays;
import java.util.List;
import java.util.stream.IntStream;

public class DriveBase extends SubsystemBase implements Sendable {

  public SwerveDriveKinematics m_swerveDriveKinematics;
  MAXSwerveModule[] m_modules = new MAXSwerveModule[4];
  // Pigeon is on roboRIO CAN Bus with device ID 1

  private final Gyro m_gyro = new Gyro(DrivetrainConstants.kUsePigeon);
  private SwerveDriveOdometry m_odometer;
  private Field2d m_field = new Field2d();

  private SlewRateLimiter m_translationLimiter = new SlewRateLimiter(
      OperatorConstants.translationLimit);
  // will be constants
  private SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(
      OperatorConstants.rotationLimit);

  TrajectoryConfig m_trajConfig = new TrajectoryConfig(
      LinearVelocity.ofBaseUnits(2, MetersPerSecond),
      LinearAcceleration.ofBaseUnits(4, MetersPerSecondPerSecond));

  private Config m_sysIdConfig;
  private Mechanism m_sysIdMechanism;
  private SysIdRoutine m_sysIdRoutine;

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutAngle m_distance = Rotations.mutable(0);
  private final MutAngularVelocity m_velocity = RPM.mutable(0);

  private HolonomicDriveController m_driveController;

  /** Creates a new DriveBase. */
  public DriveBase() {
    for (int i = 0; i < 4; i++) {
      m_modules[i] = new MAXSwerveModule(
          DrivetrainConstants.SwerveModules.values()[i]);
    }
    m_swerveDriveKinematics = new SwerveDriveKinematics(
        DrivetrainConstants.SwerveModules.frontLeft.wheelPos,
        DrivetrainConstants.SwerveModules.frontRight.wheelPos,
        DrivetrainConstants.SwerveModules.backLeft.wheelPos,
        DrivetrainConstants.SwerveModules.backRight.wheelPos);

    resetGyro();

    m_odometer = new SwerveDriveOdometry(
        m_swerveDriveKinematics,
        new Rotation2d(),
        getModulePositions());

    m_sysIdConfig = new Config(
        DrivetrainConstants.kSysIdRampRate,
        DrivetrainConstants.kStepVoltage,
        DrivetrainConstants.kTimeout);

    m_sysIdMechanism = new Mechanism(
        this::applyVolts,
        log -> {
          log
              .motor("drive-front-left")
              .voltage(
                  m_appliedVoltage.mut_replace(m_modules[0].getDriveVoltage(), Volts))
              .angularPosition(
                  m_distance.mut_replace(m_modules[0].getRawEncoderPosition(), Rotations))
              .angularVelocity(
                  m_velocity.mut_replace(
                      m_modules[0].getRawEncoderVelocity(),
                      RPM));
          log
              .motor("drive-front-right")
              .voltage(
                  m_appliedVoltage.mut_replace(m_modules[1].getDriveVoltage(), Volts))
              .angularPosition(
                  m_distance.mut_replace(m_modules[1].getRawEncoderPosition(), Rotations))
              .angularVelocity(
                  m_velocity.mut_replace(
                      m_modules[1].getRawEncoderVelocity(),
                      RPM));
          log
              .motor("drive-back-left")
              .voltage(
                  m_appliedVoltage.mut_replace(m_modules[2].getDriveVoltage(), Volts))
              .angularPosition(
                  m_distance.mut_replace(m_modules[2].getRawEncoderPosition(), Rotations))
              .angularVelocity(
                  m_velocity.mut_replace(
                      m_modules[2].getRawEncoderVelocity(),
                      RPM));
          log
              .motor("drive-back-right")
              .voltage(
                  m_appliedVoltage.mut_replace(m_modules[3].getDriveVoltage(), Volts))
              .angularPosition(
                  m_distance.mut_replace(m_modules[3].getRawEncoderPosition(), Rotations))
              .angularVelocity(
                  m_velocity.mut_replace(
                      m_modules[3].getRawEncoderVelocity(),
                      RPM));
        },
        this,
        "sysIdRoutine");

    m_sysIdRoutine = new SysIdRoutine(m_sysIdConfig, m_sysIdMechanism);

    // FIXME: Needs constants
    PIDController xPIDController = new PIDController(0, 0, 0);
    PIDController yPIDController = new PIDController(0, 0, 0);
    ProfiledPIDController thetaPIDController = new ProfiledPIDController(0, 0, 0,
        new Constraints(0, 0)); // radians per second (per second)
    thetaPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_driveController = new HolonomicDriveController(xPIDController, yPIDController, thetaPIDController);
  }

  public void applyVolts(Voltage v) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].setSwerveAngle(Rotation2d.fromDegrees(180));
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
      boolean boost) {
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
      boolean boost) {
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
        rotateVelocity);
    SmartDashboard.putNumber(
        "Intended Forward Speed",
        speeds.vxMetersPerSecond);
    SmartDashboard.putNumber(
        "Intended Sideway Speed",
        speeds.vyMetersPerSecond);
    SmartDashboard.putNumber(
        "Intended Rotation Speed",
        speeds.omegaRadiansPerSecond);
    if (fieldOrient) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle()); // needs function to get angle from pig
    }
    double movementAngle = Math.atan2(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond);
    double linearVelocity = Math.hypot(
        speeds.vyMetersPerSecond,
        speeds.vxMetersPerSecond); // pythagorean theorem
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
        DrivetrainConstants.maxSpeed);
    applyModuleStates(states);
  }

  public void applyModuleStates(SwerveModuleState[] states) {
    IntStream.range(0, states.length)
        .forEach(i -> m_modules[i].setDesiredState(states[i]));
    m_odometer.update(getAngle(), getModulePositions());
    m_field.setRobotPose(getPose());
  }

  public Command getSysIDCommand() {
    return new SequentialCommandGroup(
        m_sysIdRoutine.dynamic(Direction.kForward),
        m_sysIdRoutine.dynamic(Direction.kReverse),
        m_sysIdRoutine.quasistatic(Direction.kForward),
        m_sysIdRoutine.quasistatic(Direction.kReverse));
  }

  private SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(m_modules)
        .map(MAXSwerveModule::getPosition)
        .toArray(SwerveModulePosition[]::new);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("FL", m_modules[0]);
    SmartDashboard.putData("FR", m_modules[1]);
    SmartDashboard.putData("BL", m_modules[2]);
    SmartDashboard.putData("BR", m_modules[3]);
    SmartDashboard.putNumber("Gyro", getAngleDegrees());
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
        m_trajConfig);
  }

  public SwerveControllerCommand followTrajectoryCommand(
      Trajectory trajectory) {
    return new SwerveControllerCommand(
        trajectory,
        this::getPose,
        m_swerveDriveKinematics,
        m_driveController,
        this::applyModuleStates,
        this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModule");
    builder.addDoubleProperty("gyroAngle", this::getAngleDegrees, null);
    builder.addDoubleProperty("FL Heading", m_modules[0]::getAngleDegrees, null);
    builder.addDoubleProperty("FR Heading", m_modules[1]::getAngleDegrees, null);
    builder.addDoubleProperty("BL Heading", m_modules[2]::getAngleDegrees, null);
    builder.addDoubleProperty("BR Heading", m_modules[3]::getAngleDegrees, null);
  }
}
