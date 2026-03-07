// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
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
  private final SparkMax m_topRollerMotor = new SparkMax(IndexerConstants.kTopRollerMotorID, MotorType.kBrushless);

  private SparkMaxConfig m_leadIndexConfig = new SparkMaxConfig();
  private SparkMaxConfig m_followIndexConfig = new SparkMaxConfig();
  private SparkMaxConfig m_topRollerConfig = new SparkMaxConfig();

  private RelativeEncoder m_leadEncoder = m_leadIndexMotor.getEncoder();
  private RelativeEncoder m_topRollerEncoder = m_topRollerMotor.getEncoder();

  private final PIDController m_leadIndexPIDController = 
    new PIDController(IndexerConstants.kIndexP, IndexerConstants.kIndexI, IndexerConstants.kIndexD);

    private final PIDController m_powerCorrectionPIDController = 
  new PIDController(IndexerConstants.kPowerCorrectionP, IndexerConstants.kPowerCorrectionI, IndexerConstants.kPowerCorrectionD);

  private final PIDController m_topRollerPIDController = 
    new PIDController(IndexerConstants.kTopRollerP, IndexerConstants.kTopRollerI, IndexerConstants.kTopRollerD);


  /** Creates a new Indexer. */
  public Indexer() {
    m_leadIndexConfig
    .inverted(false).smartCurrentLimit(40).idleMode(IdleMode.kBrake);
    m_leadIndexMotor.configure(m_leadIndexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_followIndexConfig
    .follow(IndexerConstants.kLeadIndexMotorID)
    .inverted(false).smartCurrentLimit(40).idleMode(IdleMode.kBrake);
    m_followIndexMotor.configure(m_followIndexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_topRollerConfig
    .inverted(false).smartCurrentLimit(40).idleMode(IdleMode.kBrake);
    m_topRollerMotor.configure(m_topRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void indexIn() {
    double indexVolts =
      m_leadIndexPIDController.calculate(
        m_leadEncoder.getVelocity() / 60, IndexerConstants.kIndexVelocitySetpoint); //rpm / 60 = rps
    m_leadIndexMotor.setVoltage(indexVolts);

    double topRollerVolts = 
      m_topRollerPIDController.calculate(
        m_topRollerEncoder.getVelocity() / 60, IndexerConstants.kTopRollerVelocitySetpoint); //rpm / 60 = rps
    m_topRollerMotor.setVoltage(topRollerVolts);
  }

  public void indexStop() {
    m_leadIndexMotor.setVoltage(0.0);
    m_topRollerMotor.setVoltage(0);
  }

  public void indexReverse() {
    m_leadIndexMotor.setVoltage(-IndexerConstants.kVoltage);
    m_topRollerMotor.setVoltage(-IndexerConstants.kVoltage);
  }

/*  public void topRollerIn() {
    double topRollerVolts = 
      m_topRollerPIDController.calculate(
        m_topRollerEncoder.getVelocity() / 60, IndexerConstants.kTopRollerVelocitySetpoint); //rpm / 60 = rps
    m_topRollerMotor.setVoltage(topRollerVolts);
  } 
  

  public void topRollerStop() {
    m_topRollerMotor.setVoltage(0);
  }

  public void topRollerReverse() {
    m_topRollerMotor.setVoltage(-IndexerConstants.kVoltage);
  }
    */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
