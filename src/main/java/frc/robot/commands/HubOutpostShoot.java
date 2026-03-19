// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import choreo.Choreo;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import java.nio.file.Path;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HubOutpostShoot extends SequentialCommandGroup {

  DriveBase m_driveBase;
  Intake m_intake;
  Shooter m_shooter;
  Indexer m_indexer;

  /** Creates a new HubOutpostShoot.
   *  Start at hub, goes to Outpost to get fuel, goes back, shoots
   */
  public HubOutpostShoot(
    DriveBase driveBase,
    Intake intake,
    Shooter shooter,
    Indexer indexer
  ) {
    var huOuShTrajectory = Choreo.loadTrajectory("HuOuSh.traj");

    m_driveBase = driveBase;
    m_intake = intake;
    m_shooter = shooter;
    m_indexer = indexer;

    addCommands(
      //Start at Hub

      //Deploy intake
      m_intake.getDeployCommand(),
      //Drive to outpost

      //Wait a second or two maybe

      //Go back

      //Shoot
      new ParallelCommandGroup(
        m_shooter.getShootCommand(4),
        m_indexer.getIndexCommand(4)
      )
    );
  }
}
