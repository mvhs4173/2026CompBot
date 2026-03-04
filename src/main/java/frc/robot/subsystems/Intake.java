// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final SparkMax m_leadDeployMotor = new SparkMax(Constants.IntakeConstants.kLeadIntakeDeploymentID,
      MotorType.kBrushless);
  private final SparkMax m_followDeployMotor = new SparkMax(Constants.IntakeConstants.kFollowIntakeDeploymentID,
      MotorType.kBrushless);
  private final SparkMax m_runningMotor = new SparkMax(IntakeConstants.kIntakeRunningID,
      MotorType.kBrushless);

  private SparkMaxConfig m_leadDeployConfig = new SparkMaxConfig();
  private SparkMaxConfig m_followDeployConfig = new SparkMaxConfig();
  private SparkMaxConfig m_runningConfig = new SparkMaxConfig();

  private PIDController m_deploymentPIDController = new PIDController(
      IntakeConstants.kDeployP, IntakeConstants.kDeployI, IntakeConstants.kDeployD);
  private SimpleMotorFeedforward m_deploymentFFController = new SimpleMotorFeedforward(
      IntakeConstants.kDeployS, IntakeConstants.kDeployV, IntakeConstants.kDeployA);

  private final DigitalInput m_deployedLimitSwitch = new DigitalInput(
      IntakeConstants.kDeployedLimitSwitchPort);
  private final DigitalInput m_retractedLimitSwitch = new DigitalInput(
      IntakeConstants.kRetractedLimitSwitchPort);

  private boolean m_deploymentStatus = false; // T means deployed F means retracted

  /** Creates a new Intake. */
  public Intake() {
    m_leadDeployConfig
    .inverted(false).smartCurrentLimit(40).idleMode(IdleMode.kCoast);
    m_leadDeployMotor.configure(m_followDeployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_followDeployConfig
    .follow(IntakeConstants.kLeadIntakeDeploymentID)
    .inverted(false).smartCurrentLimit(40).idleMode(IdleMode.kCoast);
    m_followDeployMotor.configure(m_followDeployConfig, com.revrobotics.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
 
    m_runningConfig.inverted(false).smartCurrentLimit(40).idleMode(IdleMode.kBrake);
    m_runningMotor.configure(m_runningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  // Running

  public void runIntake() {
    m_runningMotor.setVoltage(IntakeConstants.kRunningVolts);
  }

  public void stopIntake() {
    m_runningMotor.setVoltage(0);
  }

  public void reverseIntake() {
    m_runningMotor.setVoltage(IntakeConstants.kRunningVolts);
  }

  // Deployment

  public double getDeploymentExtensionMeters() {
    return 0;//m_leadDeployMotor.getEncoder().getPosition() * IntakeConstants.kGearRatio
        // * IntakeConstants.kRotationsToMeters;
  }

  private boolean isDeployed() {
    return m_deploymentStatus = m_deployedLimitSwitch.get()
        || Math.abs(getDeploymentExtensionMeters()
            - IntakeConstants.kDeployDistanceMeters) < IntakeConstants.kDeployToleranceMeters;
  }

  private boolean isRetracted() {
    return m_deploymentStatus = m_retractedLimitSwitch.get()
        || Math.abs(getDeploymentExtensionMeters()) < IntakeConstants.kDeployToleranceMeters;
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
        getDeploymentExtensionMeters(), IntakeConstants.kDeployDistanceMeters);
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
