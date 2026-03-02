// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final SparkMax m_leadShooterMotor = new SparkMax(ShooterConstants.kLeadShooterMotorID, MotorType.kBrushless);
  private final SparkMax m_followShooterMotor = new SparkMax(ShooterConstants.kFollowShooterMotorID, MotorType.kBrushless);

  private SparkMaxConfig m_leadShooterConfig = new SparkMaxConfig();
  private SparkMaxConfig m_followShooterConfig = new SparkMaxConfig();

  private final RelativeEncoder m_shooterEncoder = m_leadShooterMotor.getEncoder();

  private final PIDController m_shooterPidController = 
      new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

  

  /** Creates a new Shooter. */
  public Shooter() {
    m_leadShooterConfig
    .inverted(false).smartCurrentLimit(40).idleMode(IdleMode.kCoast);
    m_leadShooterMotor.configure(m_leadShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_followShooterConfig
    .follow(ShooterConstants.kLeadShooterMotorID)
    .inverted(false).smartCurrentLimit(40).idleMode(IdleMode.kCoast);
    m_followShooterMotor.configure(m_followShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void shoot() {
    //m_ShooterMotor.setVoltage(Constants.ShooterConstants.kVoltage);
    double volts = m_shooterPidController.calculate(
      m_shooterEncoder.getVelocity() / 60, 0); //encoder rpm / 60 = rps //TODO: setpoint calculations
  }

  public void stop() {
    m_leadShooterMotor.setVoltage(0);
  }

  public void reverse() {
    m_leadShooterMotor.setVoltage(-Constants.ShooterConstants.kVoltage);
  }

  //Adjustable hood





  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
