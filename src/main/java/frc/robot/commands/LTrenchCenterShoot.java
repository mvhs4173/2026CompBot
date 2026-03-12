package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class LTrenchCenterShoot extends SequentialCommandGroup {
  DriveBase m_driveBase;
  Intake m_intake;
  Shooter m_shooter;
  Indexer m_indexer;

  /** Creates a new LTrenchCenterShoot
   *  Start at left trench, collect from center, go back and shoot
   */
  public LTrenchCenterShoot(DriveBase driveBase, Intake intake, Shooter shooter, Indexer indexer) {
    m_driveBase = driveBase;
    m_intake = intake;
    m_shooter = shooter;
    m_indexer = indexer;
    addCommands(
      // Start at trench

      // Deploy Intake
      m_intake.getDeployCommand(),   

      // Go to center

      // Move while intaking
      new ParallelRaceGroup(
        //move and
        m_intake.getIntakeCommand(20) // so that the intaking will always end after. Stop intaking once it gets to the point
      ),

      // Move back to under trench

      // Set shooter angle
      m_shooter.getSetHoodCommand(Constants.ShooterConstants.kHoodLowMiddleAngle),

      // Shoot
      new ParallelCommandGroup(
        m_indexer.getIndexCommand(10),
        m_shooter.getShootCommand(10) //TODO: set time to shoot for
      )

    );
  }
}
