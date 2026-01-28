// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final SparkMax m_deployMotor = new SparkMax(Constants.IntakeConstants.kIntakeDeploymentID,
      MotorType.kBrushless);
  private final SparkMax m_runningMotor = new SparkMax(Constants.IntakeConstants.kIntakeRunningID,
      MotorType.kBrushless);

  private final DigitalInput m_deployedLimitSwitch = new DigitalInput(
      Constants.IntakeConstants.kDeployedLimitSwitchPort);
  private final DigitalInput m_retractedLimitSwitch = new DigitalInput(
      Constants.IntakeConstants.kRetractedLimitSwitchPort);

  private boolean m_deploymentStatus = false; // T means deployed F means retracted

  /** Creates a new Intake. */
  public Intake() {

  }

  public void runIntake() {
    m_runningMotor.setVoltage(Constants.IntakeConstants.kRunningVolts);
  }

  public void stopIntake() {
    m_runningMotor.setVoltage(0);
  }

  public void reverseIntake() {
    m_runningMotor.setVoltage(Constants.IntakeConstants.kRunningVolts);
  }

  private boolean isDeployed() {
    return m_deploymentStatus = m_deployedLimitSwitch.get();
  }

  private boolean isRetracted() {
    return m_deploymentStatus = m_retractedLimitSwitch.get();
  }

  public void setDeployment(boolean deploy) {
    if (deploy) {
      CommandScheduler.getInstance().schedule(getDeployCommand());
    } else {
      CommandScheduler.getInstance().schedule(getRetractCommand());
    }
  }

  public void toggleDeploy() {
    setDeployment(!m_deploymentStatus);
  }

  private void deploy() {
    double speed = Constants.IntakeConstants.kDeploymentSpeed;
    speed *= Constants.IntakeConstants.kDeploymentInverted ? -1 : 1;
    m_deployMotor.set(speed);
  }

  private void retract() {
    double speed = -Constants.IntakeConstants.kDeploymentSpeed;
    speed *= Constants.IntakeConstants.kDeploymentInverted ? -1 : 1;
    m_deployMotor.set(speed);
  }

  public void stopDeployMotor() {
    m_deployMotor.set(0);
  }

  private Command getDeployCommand() {
    return new RunCommand(this::deploy, this)
        .until(this::isDeployed)
        .finallyDo(this::stopDeployMotor);
  }

  private Command getRetractCommand() {
    return new RunCommand(this::retract, this)
        .until(this::isRetracted)
        .finallyDo(this::stopDeployMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
