// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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

    addCommands(
      



    );
  }
}
