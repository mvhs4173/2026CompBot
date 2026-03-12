// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LTrenchCenterDoubleDip extends SequentialCommandGroup {

  /** Creates a new LTrenchCenterDoubleDip.
   *  Start at left trench, collects in center, goes under trench, shoots X2
   */
  public LTrenchCenterDoubleDip() {

    addCommands(
      // Start at left trench

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
