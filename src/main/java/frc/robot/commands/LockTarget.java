// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveBase;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LockTarget extends Command {

  final DriveBase m_driveBase;
  final DoubleSupplier m_leftYSupplier; //Forward
  final DoubleSupplier m_leftXSupplier; //Side

  final PIDController m_turningController;

  final double kP = 6.0 / 25.0; //radians per second / max offset

  /** Creates a new LockTarget. */
  public LockTarget(
    DriveBase driveBase,
    DoubleSupplier leftXSupplier,
    DoubleSupplier leftYSupplier
  ) {
    m_driveBase = driveBase;
    m_leftXSupplier = leftXSupplier;
    m_leftYSupplier = leftYSupplier;
    m_turningController = new PIDController(kP, 0, 0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds = new ChassisSpeeds(
      m_leftXSupplier.getAsDouble(),
      m_leftYSupplier.getAsDouble(),
      m_turningController.calculate(LimelightHelpers.getTX(""), 0)
    );
    m_driveBase.applySpeeds(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveBase.applySpeeds(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
