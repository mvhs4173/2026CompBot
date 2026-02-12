// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  public final SparkMax m_shooterMotor = new SparkMax(ShooterConstants.kShooterMotorID, MotorType.kBrushless);
  public final RelativeEncoder m_shooterEncoder = m_shooterMotor.getEncoder();

  public final PIDController m_shooterPidController = 
      new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

  /** Creates a new Shooter. */
  public Shooter() {

  }

  //Might make code for adjustable hood

  public void shoot() {
    //m_ShooterMotor.setVoltage(Constants.ShooterConstants.kVoltage);
    double volts = m_shooterPidController.calculate(
      m_shooterEncoder.getVelocity() / 60, 0); //encoder rpm / 60 = rps //TODO: setpoint calculations
  }

  public void stop() {
    m_shooterMotor.setVoltage(0);
  }

  public void reverse() {
    m_shooterMotor.setVoltage(-Constants.ShooterConstants.kVoltage);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
