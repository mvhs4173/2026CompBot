// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  public final SparkMax m_ShooterMotor = new SparkMax(Constants.ShooterConstants.kShooterMotorID, MotorType.kBrushless);


  /** Creates a new Shooter. */
  public Shooter() {

  }

  public void shoot() {
    m_ShooterMotor.setVoltage(Constants.ShooterConstants.kVoltage);
  }

  public void stop() {
    m_ShooterMotor.setVoltage(0);
  }

  public void reverse() {
    m_ShooterMotor.setVoltage(-Constants.ShooterConstants.kVoltage);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
