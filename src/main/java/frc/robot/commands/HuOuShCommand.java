// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.Optional;

// import choreo.Choreo;
// import choreo.trajectory.SwerveSample;
// import choreo.trajectory.Trajectory;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DriveBase;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class HuOuShCommand extends Command {
//   private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("HuOuSh");
//     private final Timer timer = new Timer();

//   /** Creates a new HuOuShCommand. */
//   public HuOuShCommand(DriveBase m_drive) {
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//      if (trajectory.isPresent()) {
//             // Get the initial pose of the trajectory
//             Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());

//             if (initialPose.isPresent()) {
//                 // Reset odometry to the start of the trajectory
//                 m_drive.resetOdometry(initialPose.get());
//             }
//         }

//         // Reset and start the timer when the autonomous period begins
//         timer.restart();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (trajectory.isPresent()) {
//             // Sample the trajectory at the current time into the autonomous period
//             Optional<SwerveSample> sample = trajectory.get().sampleAt(timer.get(), isRedAlliance());

//             if (sample.isPresent()) {
//                 m_drive.followTrajectory(sample);
//             }
//         }
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
