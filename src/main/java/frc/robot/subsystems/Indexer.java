// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;

public class Indexer extends SubsystemBase {

  SendableChooser<Command> m_topSysIDAutoChooser;
  SendableChooser<Command> m_bottomSysIDAutoChooser;

  private final SparkMax m_leadIndexMotor = new SparkMax(
    IndexerConstants.kLeadIndexMotorID,
    MotorType.kBrushless
  );
  private final SparkMax m_followIndexMotor = new SparkMax(
    IndexerConstants.kFollowIndexMotorID,
    MotorType.kBrushless
  );
  private final SparkMax m_topRollerMotor = new SparkMax(
    IndexerConstants.kTopRollerMotorID,
    MotorType.kBrushless
  );

  private SparkMaxConfig m_leadIndexConfig = new SparkMaxConfig();
  private SparkMaxConfig m_followIndexConfig = new SparkMaxConfig();
  private SparkMaxConfig m_topRollerConfig = new SparkMaxConfig();

  private RelativeEncoder m_leadEncoder = m_leadIndexMotor.getEncoder();
  private RelativeEncoder m_topRollerEncoder = m_topRollerMotor.getEncoder();

  private final PIDController m_leadIndexPIDController = new PIDController(
    IndexerConstants.kIndexP,
    IndexerConstants.kIndexI,
    IndexerConstants.kIndexD
  );

  private final PIDController m_powerCorrectionPIDController =
    new PIDController(
      IndexerConstants.kPowerCorrectionP,
      IndexerConstants.kPowerCorrectionI,
      IndexerConstants.kPowerCorrectionD
    );

  private final PIDController m_topRollerPIDController = new PIDController(
    IndexerConstants.kTopRollerP,
    IndexerConstants.kTopRollerI,
    IndexerConstants.kTopRollerD
  );

  private SimpleMotorFeedforward m_topFF = new SimpleMotorFeedforward(
    IndexerConstants.kTopS,
    0,
    0
  );
  private SimpleMotorFeedforward m_bottomFF = new SimpleMotorFeedforward(
    0,
    0,
    0
  );

  private Config m_sysIdTopConfig;
  private Mechanism m_sysIdTopMechanism;
  private SysIdRoutine m_sysIdTopRoutine;

  private Config m_sysIdBottomConfig;
  private Mechanism m_sysIdBottomMechanism;
  private SysIdRoutine m_sysIdBottomRoutine;

  private final MutVoltage m_appliedTopVoltage = Volts.mutable(0);
  private final MutAngle m_topAngle = Rotations.mutable(0);
  private final MutAngularVelocity m_topVelocity = RPM.mutable(0);
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutAngle m_angle = Rotations.mutable(0);
  private final MutAngularVelocity m_velocity = RPM.mutable(0);

  /** Creates a new Indexer. */
  public Indexer() {
    m_leadIndexConfig
      .inverted(true)
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake);
    m_leadIndexMotor.configure(
      m_leadIndexConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_followIndexConfig
      .follow(IndexerConstants.kLeadIndexMotorID, true)
      .inverted(true)
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake);
    m_followIndexMotor.configure(
      m_followIndexConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_topRollerConfig
      .inverted(false)
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake);
    m_topRollerMotor.configure(
      m_topRollerConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_sysIdTopConfig = new Config(
      IndexerConstants.kSysIdRampRate,
      IndexerConstants.kStepVoltage,
      IndexerConstants.kTimeout
    );
    m_sysIdBottomConfig = new Config(
      IndexerConstants.kSysIdRampRate,
      IndexerConstants.kStepVoltage,
      IndexerConstants.kTimeout
    );
    m_sysIdTopMechanism = new Mechanism(
      m_topRollerMotor::setVoltage,
      log -> {
        log
          .motor("Top Index")
          .voltage(
            m_appliedTopVoltage.mut_replace(
              m_topRollerMotor.getAppliedOutput() *
              m_topRollerMotor.getBusVoltage(),
              Volts
            )
          )
          .angularPosition(
            m_topAngle.mut_replace(
              m_topRollerEncoder.getPosition() / 5.0,
              Rotations
            )
          )
          .angularVelocity(
            m_topVelocity.mut_replace(
              m_topRollerEncoder.getVelocity() / 5.0,
              RPM
            )
          );
      },
      this,
      "top_sysIdRoutine"
    );

    m_sysIdBottomMechanism = new Mechanism(
      m_leadIndexMotor::setVoltage,
      log -> {
        log
          .motor("Bottom Index")
          .voltage(
            m_appliedVoltage.mut_replace(
              m_leadIndexMotor.getAppliedOutput() *
              m_leadIndexMotor.getBusVoltage(),
              Volts
            )
          )
          .angularPosition(
            m_angle.mut_replace(m_leadEncoder.getPosition() / 5.0, Rotations)
          )
          .angularVelocity(
            m_velocity.mut_replace(m_leadEncoder.getVelocity() / 5.0, RPM)
          );
      },
      this,
      "bottom_sysIdRoutine"
    );

    m_sysIdTopRoutine = new SysIdRoutine(m_sysIdTopConfig, m_sysIdTopMechanism);
    m_sysIdBottomRoutine = new SysIdRoutine(
      m_sysIdBottomConfig,
      m_sysIdBottomMechanism
    );

    m_topSysIDAutoChooser = new SendableChooser<Command>();
    m_bottomSysIDAutoChooser = new SendableChooser<Command>();
    m_topSysIDAutoChooser.setDefaultOption("Noop", new InstantCommand());
    m_bottomSysIDAutoChooser.setDefaultOption("Noop", new InstantCommand());
    m_topSysIDAutoChooser.addOption(
      "DynamicForward",
      m_sysIdTopRoutine.dynamic(Direction.kForward)
    );
    m_topSysIDAutoChooser.addOption(
      "DynamicReverse",
      m_sysIdTopRoutine.dynamic(Direction.kReverse)
    );
    m_topSysIDAutoChooser.addOption(
      "QuasistaticForward",
      m_sysIdTopRoutine.quasistatic(Direction.kForward)
    );
    m_topSysIDAutoChooser.addOption(
      "QuasistaticReverse",
      m_sysIdTopRoutine.quasistatic(Direction.kReverse)
    );
    m_bottomSysIDAutoChooser.addOption(
      "DynamicForward",
      m_sysIdBottomRoutine.dynamic(Direction.kForward)
    );
    m_bottomSysIDAutoChooser.addOption(
      "DynamicReverse",
      m_sysIdBottomRoutine.dynamic(Direction.kReverse)
    );
    m_bottomSysIDAutoChooser.addOption(
      "QuasistaticForward",
      m_sysIdBottomRoutine.quasistatic(Direction.kForward)
    );
    m_bottomSysIDAutoChooser.addOption(
      "QuasistaticReverse",
      m_sysIdBottomRoutine.quasistatic(Direction.kReverse)
    );
  }

  /**public void indexBottomIn() {
    double ff =
      m_bottomFF.calculate(IndexerConstants.kBottomMotorVelocitySetpoint);
    double fb = m_leadIndexPIDController.calculate(
      m_leadEncoder.getVelocity() / 5.0,
      IndexerConstants.kBottomMotorVelocitySetpoint
    );
    double volts = fb + ff;
    m_leadIndexMotor.setVoltage(volts);
  }

  public void indexTopIn() {
    double ff =
      m_topFF.calculate(IndexerConstants.kTopRollerVelocitySetpoint);
    double fb = m_topRollerPIDController.calculate(
      m_topRollerEncoder.getVelocity() / 5.0,
      IndexerConstants.kTopRollerVelocitySetpoint
    );
    double volts = fb + ff;
    m_topRollerMotor.setVoltage(volts);
  }*/

  public void indexIn() {
    // double ff = m_bottomFF.calculate(
    //   IndexerConstants.kBottomMotorVelocitySetpoint
    // );
    // double fb = m_leadIndexPIDController.calculate(
    //   m_leadEncoder.getVelocity() / 5.0,
    //   IndexerConstants.kBottomMotorVelocitySetpoint
    // );
    // double volts = fb + ff;
    m_leadIndexMotor.setVoltage(IndexerConstants.kBottomVoltage);
  }

  /**
   * stops both index motors
   */
  public void indexStop() {
    m_leadIndexMotor.setVoltage(0.0);
    m_topRollerMotor.setVoltage(0);
  }

  public void indexReverse() {
    m_leadIndexMotor.setVoltage(-IndexerConstants.kBottomVoltage);
    m_topRollerMotor.setVoltage(-IndexerConstants.kTopVoltage);
  }

  public void topRollerIn() {
    // double topRollerVolts =
    //   m_topRollerPIDController.calculate(
    //     m_topRollerEncoder.getVelocity() / 60, IndexerConstants.kTopRollerVelocitySetpoint); //rpm / 60 = rps
    // m_topRollerMotor.setVoltage(topRollerVolts);
    // double ff = m_topFF.calculate(IndexerConstants.kTopRollerVelocitySetpoint);
    // double fb = m_topRollerPIDController.calculate(
    //   m_topRollerEncoder.getVelocity() / 5.0,
    //   IndexerConstants.kTopRollerVelocitySetpoint
    // );
    // double volts = fb + ff;
    m_topRollerMotor.setVoltage(IndexerConstants.kTopVoltage);
  }

  public void topRollerStop() {
    m_topRollerMotor.setVoltage(0);
  }

  public void topRollerReverse() {
    m_topRollerMotor.setVoltage(-IndexerConstants.kTopVoltage);
  }

  public Command runTopSysID() {
    return new SequentialCommandGroup(
      m_sysIdTopRoutine.dynamic(Direction.kForward),
      m_sysIdTopRoutine.dynamic(Direction.kReverse),
      m_sysIdTopRoutine.quasistatic(Direction.kForward),
      m_sysIdTopRoutine.quasistatic(Direction.kReverse)
    );
  }

  public Command runBottomSysID() {
    return new SequentialCommandGroup(
      m_sysIdBottomRoutine.dynamic(Direction.kForward),
      m_sysIdBottomRoutine.dynamic(Direction.kReverse),
      m_sysIdBottomRoutine.quasistatic(Direction.kForward),
      m_sysIdBottomRoutine.quasistatic(Direction.kReverse)
    );
  }

  public void runBothIndexers() {
    topRollerIn();
    indexIn();
  }

  public Command getIndexCommand(double time) {
    return new RunCommand(this::runBothIndexers, this).withTimeout(time);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IndexBottom", m_leadEncoder.getVelocity() / 5.0);
    SmartDashboard.putNumber(
      "IndexTop",
      m_topRollerEncoder.getVelocity() / 5.0
    );
    SmartDashboard.putNumber(
      "TopIndexCurrent",
      m_topRollerMotor.getOutputCurrent()
    );
    SmartDashboard.putNumber(
      "BottomIndexCurrent",
      m_leadIndexMotor.getOutputCurrent()
    );
    // This method will be called once per scheduler run
  }
}
