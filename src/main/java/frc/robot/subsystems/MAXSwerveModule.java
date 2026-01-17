// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DrivetrainConstants;

public class MAXSwerveModule {
  private final TalonFX m_drivingTalon;
  private final SparkMax m_turningSpark;

  //private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  //private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(DrivetrainConstants.SwerveModules module) {
    m_drivingTalon = new TalonFX(module.driveID);
    m_turningSpark = new SparkMax(module.turnID, MotorType.kBrushless);

    
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    //m_drivingClosedLoopController = m_drivingTalon.get();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    // m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
    //     PersistMode.kPersistParameters);
    // m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
    //     PersistMode.kPersistParameters);

    m_chassisAngularOffset = module.chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingTalon.setPosition(0.0);
  }

  /*public double getVoltage() {
    return m_
  }*/

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingTalon.getRotorVelocity().getValueAsDouble(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingTalon.getRotorPosition().getValueAsDouble(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingTalon.setControl(new MotionMagicVelocityVoltage(correctedDesiredState.speedMetersPerSecond));
    m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  public void setSwerveAngle(Rotation2d angle) {
    m_turningClosedLoopController.setSetpoint(angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset)).getRadians(), ControlType.kPosition);
  }

  public void setDriveVoltage(double volts) {
    m_drivingTalon.setControl(new VoltageOut(volts));
  }

  public double getVoltage() {
    return m_drivingTalon.getMotorVoltage().getValueAsDouble();
  }

  public double getDistanceMeters() {
    return Units.inchesToMeters(m_drivingTalon.getRotorPosition().getValueAsDouble() / 4.71 * 3 * Math.PI); //3 inch wheel diameter, 4.71 : 1 gear ratio
  }

  public double getSpeedMetersPerSecond() {
    return Units.inchesToMeters(m_drivingTalon.getRotorVelocity().getValueAsDouble() / 4.71 * 3 * Math.PI); //4.71 : 1 gear ratio
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingTalon.setPosition(0.0);
  }
}