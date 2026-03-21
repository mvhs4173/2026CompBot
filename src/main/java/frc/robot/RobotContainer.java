// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.HubOutpostShoot;
import frc.robot.commands.LTrenchCenterDoubleDip;
import frc.robot.commands.LTrenchCenterShoot;
import frc.robot.commands.LockTarget;
import frc.robot.commands.RTrenchCenterShoot;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import java.io.File;
import java.nio.file.FileSystem;
import java.nio.file.Path;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final DriveBase m_driveBase = new DriveBase();
  private final Intake m_intake = new Intake();
  private final Indexer m_indexer = new Indexer();
  private final Shooter m_shooter = new Shooter();
  private final AutoFactory autoFactory;

  public final SendableChooser<Command> m_autoChooser = new SendableChooser<
    Command
  >();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
    new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
    new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  //CommandXboxController m_operatorController = m_driverController;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    autoFactory = new AutoFactory(
      m_driveBase::getPose,
      m_driveBase::resetOdometry,
      m_driveBase::applySwerveSample,
      true,
      m_driveBase
    );
    m_autoChooser.setDefaultOption("Disable Auto", new InstantCommand());

    m_driveBase.setDefaultCommand(
      new RunCommand(
        () -> {
          m_driveBase.userDrive(
            -m_driverController.getLeftY(),
            m_driverController.getLeftX(),
            m_driverController.getRightX(),
            !m_driverController.leftBumper().getAsBoolean(),
            m_driverController.rightTrigger().getAsBoolean()
          );
        },
        m_driveBase
      )
    );

    // Configure the trigger bindings
    configureBindings();

    // m_autoChooser.addOption(
    //   "HuOuSh-FollowPath",
    //   Commands.sequence(
    //     autoFactory.resetOdometry("HuOuSh"),
    //     autoFactory.trajectoryCmd("HuOuSh")
    //   )
    // );

    m_autoChooser.addOption(
      "HubOutpostShoot",
      new HubOutpostShoot(m_driveBase, m_intake, m_shooter, m_indexer)
    );
    m_autoChooser.addOption(
      "LTrenchCenterDoubleDip",
      new LTrenchCenterDoubleDip(m_driveBase, m_intake, m_shooter, m_indexer)
    );
    m_autoChooser.addOption(
      "RTrenchCenterShoot",
      new RTrenchCenterShoot(m_driveBase, m_intake, m_shooter, m_indexer)
    );
    m_autoChooser.addOption(
      "RTrenchCenterShoot",
      new RTrenchCenterShoot(m_driveBase, m_intake, m_shooter, m_indexer)
    );
    m_autoChooser.addOption(
      "LTrenchCenterShoot",
      new LTrenchCenterShoot(m_driveBase, m_intake, m_shooter, m_indexer)
    );
    SmartDashboard.putData("Autos", m_autoChooser);
    // m_fieldTest.setRobotPose(huOuShTrajectory.getInitialPose());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // m_driverController
    //   .povDown()
    //   .whileTrue(
    //     new RunCommand(
    //       () -> {
    //         m_driveBase.setSwerveAngle(
    //           Math.atan2(
    //             m_driverController.getRightX(),
    //             -m_driverController.getRightY()
    //           )
    //         );
    //       },
    //       m_driveBase
    //     )
    //   );

    m_driverController
      .y()
      .onTrue(new InstantCommand(m_driveBase::resetGyro, m_driveBase));

    m_driverController
      .rightBumper()
      .whileTrue(
        new LockTarget(
          m_driveBase,
          m_driverController::getLeftX,
          m_driverController::getLeftY
        )
      );

    //Operator

    // Toggle deployed
    m_operatorController.x().onTrue(new InstantCommand(m_intake::toggleDeploy));

    // Run the Intake
    m_operatorController
      .a()
      .whileTrue(
        new RunCommand(m_intake::runIntake, m_intake).finallyDo(
          m_intake::stopIntake
        )
      );

    //Flush - Reverse intake, indexer, and shooter
    m_operatorController
      .rightBumper()
      .whileTrue(
        new ParallelCommandGroup(
          new RunCommand(m_intake::reverseIntake, m_intake).finallyDo(
            m_intake::stopIntake
          ),
          new RunCommand(m_indexer::indexReverse, m_indexer).finallyDo(
            m_indexer::indexStop
          ),
          new RunCommand(m_shooter::reverse, m_shooter).finallyDo(
            m_shooter::stop
          )
        )
      );

    //Adjust hood
    m_operatorController
      .povUp()
      // .onTrue(m_shooter.getSetHoodCommand(Constants.ShooterConstants.kHoodMaximumAngle));
      .whileTrue(new RunCommand(m_shooter::raiseHood, m_shooter));

    m_operatorController
      .povLeft()
      .onTrue(
        m_shooter.getSetHoodCommand(
          Constants.ShooterConstants.kHoodLowMiddleAngle
        )
      );

    m_operatorController
      .povRight()
      .onTrue(
        m_shooter.getSetHoodCommand(
          Constants.ShooterConstants.kHoodHighMiddleAngle
        )
      );

    m_operatorController
      .povDown()
      .whileTrue(new RunCommand(m_shooter::lowerHood, m_shooter));

    m_operatorController
      .b()
      .onTrue(
        m_shooter.getSetHoodPercentCommand(
          Constants.ShooterConstants.kHoodPercent
        )
      );

    //Index in
    m_operatorController
      .leftBumper()
      .whileTrue(
        new RunCommand(m_indexer::runBothIndexers).finallyDo(
          m_indexer::indexStop
        )
      );

    m_operatorController
      .y()
      .whileTrue(
        new RunCommand(
          () -> m_shooter.shooterSpeedControl(m_operatorController::getLeftY),
          m_shooter
        )
      );

    //Shoot
    m_operatorController
      .rightTrigger()
      .whileTrue(
        new ParallelCommandGroup(
          new RunCommand(m_indexer::runBothIndexers).finallyDo(
            m_indexer::indexStop
          ),
          new RunCommand(m_shooter::shoot, m_shooter).finallyDo(m_shooter::stop)
        )
      );
    m_operatorController
      .leftTrigger()
      .whileTrue(
        new ParallelCommandGroup(
          new RunCommand(m_indexer::runBothIndexers).finallyDo(
            m_indexer::indexStop
          ),
          new RunCommand(m_shooter::lowShoot, m_shooter).finallyDo(
            m_shooter::stop
          )
        )
      );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }

  public void zeroYaw() {
    m_driveBase.resetGyro();
  }
}
