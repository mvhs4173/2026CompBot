// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

//import java.io.ObjectInputFilter.Config;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ShooterConstants;



public class Shooter extends SubsystemBase {

  private class Hood {
    private final Servo m_leftHoodServo;
    private final Servo m_rightHoodServo;

    public Hood(int leftChannel, int rightChannel) {
      m_leftHoodServo = new Servo(leftChannel);
      m_rightHoodServo = new Servo(rightChannel);
    }

    /**Clamps the angle to the max + min values, 
     * Calculates percentage to set servos to
     * 
     * @param angle The angle to set the hood to
     */
    public void set(Rotation2d angle) {
      angle = Rotation2d.fromDegrees(
        MathUtil.clamp(
          angle.getDegrees(), 
          ShooterConstants.kHoodMinimumAngle.getDegrees(), 
          ShooterConstants.kHoodMaximumAngle.getDegrees()));
      double percent = 
        (angle.minus(ShooterConstants.kHoodMinimumAngle)).getRadians()
         / (ShooterConstants.kHoodMaximumAngle.minus(ShooterConstants.kHoodMinimumAngle).getRadians());
      m_leftHoodServo.set(percent);
      m_rightHoodServo.set(percent);
    }
  }

  private final SparkMax m_leadShooterMotor = new SparkMax(ShooterConstants.kLeadShooterMotorID, MotorType.kBrushless);
  private final SparkMax m_followShooterMotor = new SparkMax(ShooterConstants.kFollowShooterMotorID, MotorType.kBrushless);

  private SparkMaxConfig m_leadShooterConfig = new SparkMaxConfig();
  private SparkMaxConfig m_followShooterConfig = new SparkMaxConfig();

  private final RelativeEncoder m_shooterEncoder = m_leadShooterMotor.getEncoder();

  private final PIDController m_shooterPIDController = 
      new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

  private final Hood m_hood;

  private Config m_sysIdConfig;
  private Mechanism m_sysIdMechanism;
  private SysIdRoutine m_sysIdRoutine;

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);


  /** Creates a new Shooter. */
  public Shooter() {
    m_leadShooterConfig
    .inverted(false).smartCurrentLimit(40).idleMode(IdleMode.kCoast);
    m_leadShooterMotor.configure(m_leadShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_followShooterConfig
    .follow(ShooterConstants.kLeadShooterMotorID)
    .inverted(false).smartCurrentLimit(40).idleMode(IdleMode.kCoast);
    m_followShooterMotor.configure(m_followShooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_hood = new Hood(ShooterConstants.kLeftHoodServoChannel, ShooterConstants.kRightHoodServoChannel);

  //  m_sysIdConfig = new Config(ShooterConstants.kSysIdRampRate, ShooterConstants.kStepVoltage,
  //       ShooterConstants.kTimeout);

  //       m_sysIdMechanism = new Mechanism(this::applyVolts, log -> {
  //     log.motor("Lead shoot (Left)")
  //         .voltage(
  //             m_appliedVoltage.mut_replace(
  //                 m_leadShooterMotor.get, Volts))
  //         .linearPosition(m_distance.mut_replace(m_leadShooterMotor.getDistanceMeters(),
  //             Meters))
  //         .linearVelocity(
  //             m_velocity.mut_replace(
  //                 m_leadShooterMotor.getSpeedMetersPerSecond(),
  //                 MetersPerSecond));
  //     log.motor("Follow Shoot (Right)")
  //         .voltage(
  //             m_appliedVoltage.mut_replace(
  //                 m_followShooterMotor.getVoltage(), Volts))
  //         .linearPosition(m_distance.mut_replace(m_followShooterMotor.getDistanceMeters(),
  //             Meters))
  //         .linearVelocity(
  //             m_velocity.mut_replace(
  //                 m_followShooterMotor.getSpeedMetersPerSecond(),
  //                 MetersPerSecond));
  //   }, this, "sysIdRoutine");

  //   m_sysIdRoutine = new SysIdRoutine(m_sysIdConfig, m_sysIdMechanism);
  }

  public void applyVolts(Voltage v) {
    m_leadShooterMotor.setVoltage(v);
    m_followShooterMotor.setVoltage(v);
  }



  public void setHoodAngle(Rotation2d angle) {
    m_hood.set(angle);
  }

  public void shoot() {
    double volts = m_shooterPIDController.calculate(
      m_shooterEncoder.getVelocity() / 60, ShooterConstants.kShooterVelocitySetpoint); //encoder rpm / 60 = rps
    m_leadShooterMotor.setVoltage(volts);
  }

  public void stop() {
    m_leadShooterMotor.setVoltage(0);

    m_shooterPIDController.reset();
  }

  public void reverse() {
    m_leadShooterMotor.setVoltage(-Constants.ShooterConstants.kVoltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
