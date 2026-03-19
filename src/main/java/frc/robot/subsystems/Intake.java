// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final SparkMax m_leftDeployMotor = new SparkMax(
    Constants.IntakeConstants.kLeadIntakeDeploymentID,
    MotorType.kBrushless
  );
  private final SparkMax m_rightDeployMotor = new SparkMax(
    Constants.IntakeConstants.kFollowIntakeDeploymentID,
    MotorType.kBrushless
  );
  private final SparkMax m_runningMotor = new SparkMax(
    IntakeConstants.kIntakeRunningID,
    MotorType.kBrushless
  );

  private final RelativeEncoder m_leftDeployEncoder =
    m_leftDeployMotor.getEncoder();
  private final RelativeEncoder m_rightDeployEncoder =
    m_rightDeployMotor.getEncoder();

  private SparkMaxConfig m_leftDeployConfig = new SparkMaxConfig();
  private SparkMaxConfig m_rightDeployConfig = new SparkMaxConfig();
  private SparkMaxConfig m_runningConfig = new SparkMaxConfig();

  private EncoderConfig m_leftDeployEncoderConfig = new EncoderConfig();
  private EncoderConfig m_rightDeployEncoderConfig = new EncoderConfig();

  private PIDController m_leftDeploymentPIDController = new PIDController(
    IntakeConstants.kDeployP,
    IntakeConstants.kDeployI,
    IntakeConstants.kDeployD
  );
  private PIDController m_rightDeploymentPIDController = new PIDController(
    IntakeConstants.kDeployP,
    IntakeConstants.kDeployI,
    IntakeConstants.kDeployD
  );
  private SimpleMotorFeedforward m_deploymentFFController =
    new SimpleMotorFeedforward(
      IntakeConstants.kDeployS,
      IntakeConstants.kDeployV,
      IntakeConstants.kDeployA
    );

  // private final DigitalInput m_deployedLimitSwitch = new DigitalInput(
  //  IntakeConstants.kDeployedLimitSwitchPort);
  private final DigitalInput m_retractedLimitSwitch = new DigitalInput(
    IntakeConstants.kRetractedLimitSwitchPort
  );

  private boolean m_deploymentStatus = false; // T means deployed F means retracted

  /** Creates a new Intake. */
  public Intake() {
    m_leftDeployConfig
      .inverted(true)
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kCoast);
    m_leftDeployMotor.configure(
      m_leftDeployConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_leftDeployEncoderConfig
      .positionConversionFactor(0.0127)
      .inverted(IntakeConstants.kLeftDeployEncoderInverted);

    m_rightDeployConfig //.follow(IntakeConstants.kLeadIntakeDeploymentID)
      .inverted(true)
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kCoast);
    m_rightDeployMotor.configure(
      m_rightDeployConfig,
      com.revrobotics.ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_rightDeployEncoderConfig
      .positionConversionFactor(0.0127)
      .inverted(IntakeConstants.kRightDeployEncoderInverted);

    m_runningConfig
      .inverted(false)
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake);
    m_runningMotor.configure(
      m_runningConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_leftDeploymentPIDController.setTolerance(
      IntakeConstants.kDeployToleranceMeters
    );
    m_rightDeploymentPIDController.setTolerance(
      IntakeConstants.kDeployToleranceMeters
    );
  }

  // Running

  /**
   * Sets the voltage of the intake roller to IntakeConstants.kRunningVOlts
   */
  public void runIntake() {
    m_runningMotor.setVoltage(IntakeConstants.kRunningVolts);
  }

  /**
   * Sets the voltage of the intake roller to 0.0
   */
  public void stopIntake() {
    m_runningMotor.setVoltage(0.0);
  }

  /**
   * Sets the voltage of the intake roller to -IntakeConstants.kRunningVolts
   */
  public void reverseIntake() {
    m_runningMotor.setVoltage(-IntakeConstants.kRunningVolts);
  }

  // Deployment

  /**
   * Calculates the deployment of the left motor in meters
   * @return Encoder position * Gear ratio * Rotation to Meters values
   */
  public double getLeftDeploymentExtensionMeters() {
    return (m_leftDeployEncoder.getPosition() * 0.0127) / 2;
  }

  /**
   * Calculates the deployment of the right motor in meters
   * @return Encoder position * Gear ratio * Rotation to Meters values
   */
  public double getRightDeploymentExtensionMeters() {
    return (m_rightDeployEncoder.getPosition() * 0.0127) / 2;
  }

  /**
   * Calculates if the intake is fully deployed
   * @return True if either the deployed limit switch is true
   *   or the extension - the distance for deployment is less than a certain tolerance. False if neither
   */
  private boolean isDeployed() {
    return (
      m_deploymentStatus = // m_deployedLimitSwitch.get() ||
        Math.abs(
          getLeftDeploymentExtensionMeters() -
          IntakeConstants.kDeployDistanceMeters
        ) <
        IntakeConstants.kDeployToleranceMeters
    );
  }

  /**
   * Calculates if the intake is fully retracted
   * @return True if either the retracted limit switch is true or the extension is less than a certain tolerance. False if neither
   */
  private boolean isRetracted() {
    return (
      m_deploymentStatus = m_retractedLimitSwitch.get()
      //  ||
      // Math.abs(getLeftDeploymentExtensionMeters()) <
      // IntakeConstants.kDeployToleranceMeters
    );
  }

  /**
   * Schedules either the getDeployCommand or the getRetractCommand
   * @param deploy True: Schedules deploy command, False: Schedules retract command
   */
  public Command setDeployment(boolean deploy) {
    m_deploymentStatus = deploy;
    return deploy ? getDeployCommand() : getRetractCommand();
  }

  /**
   * Calls setDeployment, with a boolean value opposite of the current deployment status
   */
  public void toggleDeploy() {
    m_deploymentStatus = !m_deploymentStatus;
    if (m_deploymentStatus) {
      CommandScheduler.getInstance().schedule(getRetractCommand());
    } else {
      CommandScheduler.getInstance().schedule(getDeployCommand());
    }
  }

  /**
   * Calculates PID for both motors with a setpoint of IntakeConstants.kDeployDistanceMeters and sets the voltage of the motors
   */
  private void deploy() {
    double leftVolts = m_leftDeploymentPIDController.calculate(
      getLeftDeploymentExtensionMeters(),
      IntakeConstants.kDeployDistanceMeters
    );
    double rightVolts = m_rightDeploymentPIDController.calculate(
      getRightDeploymentExtensionMeters(),
      IntakeConstants.kDeployDistanceMeters
    );

    m_leftDeployMotor.setVoltage(leftVolts);
    m_rightDeployMotor.setVoltage(rightVolts);
  }

  /**
   * Calculates PID for both motors with a setpoint of 0 meters and sets the voltage of the motors
   */
  private void retract() {
    double leftVolts = m_leftDeploymentPIDController.calculate(
      getLeftDeploymentExtensionMeters(),
      0.0
    );
    double rightVolts = m_rightDeploymentPIDController.calculate(
      getRightDeploymentExtensionMeters(),
      0.0
    );

    m_leftDeployMotor.setVoltage(leftVolts);
    m_rightDeployMotor.setVoltage(rightVolts);
  }

  /**
   *  Sets the voltage of both deployment motors to 0.0
   */
  public void stopDeployMotors() {
    m_leftDeployMotor.setVoltage(0.0);
    m_rightDeployMotor.setVoltage(0.0);

    m_leftDeploymentPIDController.reset();
    m_rightDeploymentPIDController.reset();
  }

  public Command getDeployCommand() {
    return new RunCommand(this::deploy, this)
      .until(this::isDeployed)
      .withTimeout(3)
      .finallyDo(this::stopDeployMotors);
  }

  public Command getRetractCommand() {
    return new RunCommand(this::retract, this)
      .until(this::isRetracted)
      .withTimeout(4)
      .finallyDo(this::stopDeployMotors);
  }

  public Command getIntakeCommand(double time) {
    return new RunCommand(this::runIntake, this)
      .withTimeout(time)
      .finallyDo(this::stopIntake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Retracted", m_retractedLimitSwitch.get());
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(
      "Intake Deployment Extension Inches",
      Units.metersToInches(getLeftDeploymentExtensionMeters())
    );
    SmartDashboard.putBoolean("DepStatus", m_deploymentStatus);
    SmartDashboard.putBoolean("isdeployed", isDeployed());
    SmartDashboard.putBoolean("isRetracted", isRetracted());
    SmartDashboard.putNumber(
      "IntakeRPM",
      m_runningMotor.getEncoder().getVelocity() / 3.0
    );

    SmartDashboard.putNumber(
      "LeftDeployCurrent",
      m_leftDeployMotor.getOutputCurrent()
    );
    SmartDashboard.putNumber(
      "LeftDeployVoltage",
      m_leftDeployMotor.getAppliedOutput() * m_leftDeployMotor.getBusVoltage()
    );
    SmartDashboard.putNumber(
      "RightDeployCurrent",
      m_rightDeployMotor.getOutputCurrent()
    );
    SmartDashboard.putNumber(
      "RightDeployVoltage",
      m_rightDeployMotor.getAppliedOutput() * m_rightDeployMotor.getBusVoltage()
    );
  }
}
