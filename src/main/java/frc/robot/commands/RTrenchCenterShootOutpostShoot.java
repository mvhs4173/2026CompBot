// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RTrenchCenterShootOutpostShoot extends SequentialCommandGroup {
  DriveBase m_driveBase;
  Intake m_intake;
  Shooter m_shooter;
  Indexer m_indexer;

  /** Creates a new RTrenchCenterShootOutpostShoot. 
   *  Start at right trench, collects in center, shoots from under trench, goes to outpost, shoots
  */
  public RTrenchCenterShootOutpostShoot(DriveBase driveBase, Intake intake, Shooter shooter, Indexer indexer) {
    m_driveBase = driveBase;
    m_intake = intake;
    m_shooter = shooter;
    m_indexer = indexer;

    addCommands(
      //Start under right trench

      //Deploy intake
      m_intake.getDeployCommand(),

      //Move to center

      new ParallelRaceGroup(
        //move and
        m_intake.getIntakeCommand(20) // so that the intaking will always end after. Stop intaking once it gets to the point
      ),

      //Move to trench

      // Shoot
      new ParallelCommandGroup(
        m_indexer.getIndexCommand(10),
        m_shooter.getShootCommand(10) //TODO: set time to shoot for
      ),

      //Move to outpost

      //Wait maybe

      //Move to scoring position

      //Adjust Hood
      m_shooter.getSetHoodCommand(Rotation2d.fromDegrees(0)), //TODO: determine angle

      //Shoot
      new ParallelCommandGroup(
        m_indexer.getIndexCommand(10),
        m_shooter.getShootCommand(10) //TODO: set time to shoot for
      )



    );
  }
}
