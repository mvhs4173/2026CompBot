// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  private final SparkMax m_leadIndexMotor = new SparkMax(IndexerConstants.kLeadIndexMotorID, MotorType.kBrushless);
  private final SparkMax m_followIndexMotor = new SparkMax(IndexerConstants.kFollowIndexMotorID, MotorType.kBrushless);

  private SparkMaxConfig m_leadIndexConfig = new SparkMaxConfig();
  private SparkMaxConfig m_followIndexConfig = new SparkMaxConfig();

  private RelativeEncoder m_leadEncoder = m_leadIndexMotor.getEncoder();

  private final PIDController m_indexPIDController = 
    new PIDController(IndexerConstants.kP, IndexerConstants.kI, IndexerConstants.kD);


  /** Creates a new Indexer. */
  public Indexer() {
    m_leadIndexConfig
    .inverted(false).smartCurrentLimit(40).idleMode(IdleMode.kBrake);
    m_leadIndexMotor.configure(m_leadIndexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_followIndexConfig
    .follow(IndexerConstants.kLeadIndexMotorID)
    .inverted(false).smartCurrentLimit(40).idleMode(IdleMode.kBrake);
    m_followIndexMotor.configure(m_followIndexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void indexIn() {
    double volts =
      m_indexPIDController.calculate(m_leadEncoder.getVelocity() / 60, IndexerConstants.kIndexVelocitySetpoint); //rpm / 60 = rps
    m_leadIndexMotor.setVoltage(volts);
  }

  public void indexStop() {
    m_leadIndexMotor.setVoltage(0.0);
  }

  public void indexReverse() {
    m_leadIndexMotor.setVoltage(-IndexerConstants.kVoltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
