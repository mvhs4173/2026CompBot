// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  // private final SparkMax m_leadDeployMotor = new SparkMax(Constants.IntakeConstants.kIntakeLeadDeploymentID,
  //     MotorType.kBrushless);
  // private final SparkMax m_followDeployMotor = new SparkMax(Constants.IntakeConstants.kIntakeFollowDeploymentID,
  //     MotorType.kBrushless);
  private final SparkMax m_runningMotor = new SparkMax(Constants.IntakeConstants.kIntakeRunningID,
      MotorType.kBrushless);

  private SparkMaxConfig m_followDeployConfig = new SparkMaxConfig();

  private PIDController m_deploymentPIDController = new PIDController(
      Constants.IntakeConstants.kDeployP, Constants.IntakeConstants.kDeployI, Constants.IntakeConstants.kDeployD);
  private SimpleMotorFeedforward m_deploymentFFController = new SimpleMotorFeedforward(
      Constants.IntakeConstants.kDeployS, Constants.IntakeConstants.kDeployV, Constants.IntakeConstants.kDeployA);

  private final DigitalInput m_deployedLimitSwitch = new DigitalInput(
      Constants.IntakeConstants.kDeployedLimitSwitchPort);
  private final DigitalInput m_retractedLimitSwitch = new DigitalInput(
      Constants.IntakeConstants.kRetractedLimitSwitchPort);

  private boolean m_deploymentStatus = false; // T means deployed F means retracted

  /** Creates a new Intake. */
  public Intake() {
    m_followDeployConfig.follow(Constants.IntakeConstants.kIntakeLeadDeploymentID);

    // m_followDeployMotor.configure(
    //     m_followDeployConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Running

  public void runIntake() {
    m_runningMotor.setVoltage(Constants.IntakeConstants.kRunningVolts);
  }

  public void stopIntake() {
    m_runningMotor.setVoltage(0);
  }

  public void reverseIntake() {
    m_runningMotor.setVoltage(Constants.IntakeConstants.kRunningVolts);
  }

  // Deployment

  public double getDeploymentExtensionMeters() {
    return 0;//m_leadDeployMotor.getEncoder().getPosition() * Constants.IntakeConstants.kGearRatio
        // * Constants.IntakeConstants.kRotationsToMeters;
  }

  private boolean isDeployed() {
    return m_deploymentStatus = m_deployedLimitSwitch.get()
        || Math.abs(getDeploymentExtensionMeters()
            - Constants.IntakeConstants.kDeployDistanceMeters) < Constants.IntakeConstants.kDeployToleranceMeters;
  }

  private boolean isRetracted() {
    return m_deploymentStatus = m_retractedLimitSwitch.get()
        || Math.abs(getDeploymentExtensionMeters()) < Constants.IntakeConstants.kDeployToleranceMeters;
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
    double volts = m_deploymentPIDController.calculate(
        getDeploymentExtensionMeters(), Constants.IntakeConstants.kDeployDistanceMeters);
    // m_leadDeployMotor.setVoltage(volts);
    ;
  }

  private void retract() {
    double volts = m_deploymentPIDController.calculate(
        getDeploymentExtensionMeters(), 0.0);
    // m_leadDeployMotor.setVoltage(volts);
  }

  public void stopDeployMotor() {
    // m_leadDeployMotor.setVoltage(0);
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
    SmartDashboard.putNumber("Intake Deployment Extension Meters", getDeploymentExtensionMeters());
  }
}
