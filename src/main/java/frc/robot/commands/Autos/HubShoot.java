// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HubShoot extends SequentialCommandGroup {

  final Shooter m_shooter;
  final Indexer m_indexer;

  /** Creates a new Shoot. */
  public HubShoot(Shooter shooter, Indexer indexer) {
    m_shooter = shooter;
    m_indexer = indexer;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      m_shooter.getSetHoodPercentCommand(0.20).withTimeout(3),
      m_shooter.getShootCommand(2),
      new ParallelCommandGroup(
        m_indexer.getIndexCommand(5),
        m_shooter.getShootCommand(5)
      ).withTimeout(5),
      m_indexer.getStopCommand()
    );
  }
}
