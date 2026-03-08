package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveBase;

public class AutoAlpha extends SequentialCommandGroup {
  DriveBase m_driveBase;
  public AutoAlpha(DriveBase driveBase) {
    m_driveBase = driveBase;
    addCommands();
  }
}
