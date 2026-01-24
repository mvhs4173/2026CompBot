// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final TalonFX m_deployMotor = new TalonFX(Constants.IntakeConstants.kIntakeDeploymentID);
  private final TalonFX m_runningMotor = new TalonFX(Constants.IntakeConstants.kIntakeRunningID);

  private final DigitalInput m_deployedLimitSwitch = new DigitalInput(Constants.IntakeConstants.kDeployedLimitSwitchPort);
  private final DigitalInput m_retractedLimitSwitch = new DigitalInput(Constants.IntakeConstants.kRetractedLimitSwitchPort);

  private boolean m_deploymentStatus; //T means deployed F means retracted




  /** Creates a new Intake. */
  public Intake() {

  }

  public void setDeployment(boolean deploy) {
    m_deploymentStatus = deploy;
  }

  public void deploy(){
    double speed = Constants.IntakeConstants.kDeploymentSpeed;
    speed *= Constants.IntakeConstants.kDeploymentInverted ? -1 : 1;
    m_deployMotor.set(speed);



  }

 // public Command 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
