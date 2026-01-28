// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private final SparkMax m_indexMotor = new SparkMax(Constants.IndexerConstants.kIndexMotorID, MotorType.kBrushless);

  /** Creates a new Indexer. */
  public Indexer() {
  }

  public void indexIn() {
    m_indexMotor.setVoltage(Constants.IndexerConstants.kVoltage);
  }

  public void indexStop() {
    m_indexMotor.setVoltage(0.0);
  }

  public void indexReverse() {
    m_indexMotor.setVoltage(-Constants.IndexerConstants.kVoltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
