// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Constants.DrivetrainConstants;

public class MAXSwerveModule implements Sendable {

  private TalonFX m_drivingTalon;
  private SparkMax m_turningSpark;

  private AbsoluteEncoder m_turningEncoder;

  private VoltageOut m_driveVoltage;

  private final SparkClosedLoopController m_turningClosedLoopController;

  public MAXSwerveModule(DrivetrainConstants.SwerveModules module) {
    m_driveVoltage = new VoltageOut(0);
    m_drivingTalon = new TalonFX(module.driveID);
    m_turningSpark = new SparkMax(module.turnID, MotorType.kBrushless);

    var m_turnMotorConfig = new SparkMaxConfig()
      .smartCurrentLimit(20)
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    m_turnMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pid(1, 0, 0)
      .outputRange(-1, 1)
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(0, 2 * Math.PI);
    m_turnMotorConfig.absoluteEncoder
      .inverted(false)
      .positionConversionFactor(2 * Math.PI)
      .velocityConversionFactor((2 * Math.PI) / 60.0)
      .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

    m_turningSpark.configure(
      m_turnMotorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    var talonFXConfigs = new TalonFXConfiguration()
      .withMotorOutput(
        new MotorOutputConfigs()
          .withInverted(
            module.driveReversed
              ? InvertedValue.Clockwise_Positive
              : InvertedValue.CounterClockwise_Positive
          )
          .withNeutralMode(NeutralModeValue.Brake)
      )
      .withCurrentLimits(
        new CurrentLimitsConfigs()
          .withStatorCurrentLimit(Amps.of(120))
          .withStatorCurrentLimitEnable(true)
      )
      .withSlot0(
        new Slot0Configs()
          .withKS(0.0)
          .withKV(0.0)
          .withKA(0.0)
          .withKP(0.0)
          .withKI(0.0)
          .withKD(0.0)
      );

    m_drivingTalon.getConfigurator().apply(talonFXConfigs);

    m_turningSpark.configure(
      Configs.MAXSwerveModule.turningConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_drivingTalon.setPosition(0.0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getWheelVelocity(), getAngle());
  }

  public double getWheelVelocity() {
    return convertRPMtoMPS(
      m_drivingTalon.getRotorVelocity().getValueAsDouble()
    );
  }

  public Rotation2d getAngle() {
    return new Rotation2d(m_turningEncoder.getPosition()).unaryMinus();
  }

  public double getAngleDegrees() {
    return getAngle().getDegrees();
  }

  public double getOutputAngleDegrees() {
    return getAngle()
      .plus(
        Rotation2d.fromRotations(
          Math.signum(m_drivingTalon.get()) < 0 ? 0.5 : 0
        )
      )
      .getDegrees();
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDistanceMeters(), getAngle());
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState.optimize(getAngle());
    SmartDashboard.putNumber(
      "Swerve" + m_turningSpark.getDeviceId() + " desiredAngle",
      desiredState.angle.getDegrees()
    );
    SmartDashboard.putNumber(
      "Swerve" + m_turningSpark.getDeviceId() + " desiredSpeed",
      desiredState.speedMetersPerSecond
    );
    setDriveVoltage(
      (desiredState.speedMetersPerSecond / DrivetrainConstants.maxSpeed) * 12
    );
    //m_drivingTalon.setControl(new VelocityVoltage(convertMPStoRPM(desiredState.speedMetersPerSecond)));
    setSwerveAngle(desiredState.angle);
  }

  public void setSwerveAngle(Rotation2d angle) {
    m_turningClosedLoopController.setSetpoint(
      angle.unaryMinus().getRadians() % (2 * Math.PI),
      ControlType.kPosition
    );
  }

  public void setDriveVoltage(double volts) {
    m_drivingTalon.setControl(m_driveVoltage.withOutput(volts));
    SmartDashboard.putNumber(
      "Swerve" + m_drivingTalon.getDeviceID() + " desired Voltage",
      m_driveVoltage.Output
    );
  }

  public double getDriveVoltage() {
    return m_drivingTalon.getMotorVoltage().getValueAsDouble();
  }

  private double getDriveCurrent() {
    return m_drivingTalon.getStatorCurrent().getValueAsDouble();
  }

  private double getTurnVoltage() {
    return m_turningSpark.getAppliedOutput() * m_turningSpark.getBusVoltage();
  }

  private double getTurnCurrent() {
    return m_turningSpark.getOutputCurrent();
  }

  public double getRPM() {
    return m_drivingTalon.getRotorVelocity().getValueAsDouble();
  }

  private double getDistanceMeters() {
    return rot2meter(m_drivingTalon.getRotorPosition().getValueAsDouble());
  }

  public double getRawEncoderPosition() {
    return m_drivingTalon.getRotorPosition().getValueAsDouble();
  }

  public double getRawEncoderVelocity() {
    return m_drivingTalon.getRotorVelocity().getValueAsDouble();
  }

  private double convertMPStoRPM(double speedMetersPerSecond) {
    return (
      ((speedMetersPerSecond * 4.71) / Units.inchesToMeters(3 * Math.PI)) * 60.0
    );
  }

  private double convertRPMtoMPS(double rotationsPerMinute) {
    return (
      (rotationsPerMinute * Units.inchesToMeters(3 * Math.PI)) / 4.71 / 60.0
    );
  }

  private double rot2meter(double rotations) {
    return (rotations / 4.71) * Units.inchesToMeters(3 * Math.PI);
  }

  public void resetEncoders() {
    m_drivingTalon.setPosition(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModule");
    builder.addDoubleProperty("angle", this::getAngleDegrees, null);
    builder.addDoubleProperty("outputAngle", this::getOutputAngleDegrees, null);
    builder.addDoubleProperty("speed", this::getWheelVelocity, null);
    builder.addDoubleProperty("driveVoltage", this::getDriveVoltage, null);
    builder.addDoubleProperty("driveCurrent", this::getDriveCurrent, null);
    builder.addDoubleProperty("turnVoltage", this::getTurnVoltage, null);
    builder.addDoubleProperty("turnCurrent", this::getTurnCurrent, null);
  }
}
