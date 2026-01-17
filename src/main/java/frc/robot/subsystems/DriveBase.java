// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;

public class DriveBase extends SubsystemBase {
  SwerveDriveKinematics swerveDriveKinematics;
  MAXSwerveModule[] modules = new MAXSwerveModule[4];
  private final Pigeon2 pigeon = new Pigeon2(DrivetrainConstants.pigeonID, "rio"); // Pigeon is on roboRIO CAN Bus with device ID 1
  private SlewRateLimiter translationLimiter = new SlewRateLimiter(DrivetrainConstants.translationLimit); //will be constants
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(DrivetrainConstants.rotationLimit);


  /** Creates a new DriveBase. */
  public DriveBase() {
    for (int i = 0; i < 4; i++){
      modules[i] = new MAXSwerveModule(DrivetrainConstants.SwerveModules.values()[i]);
    }
    swerveDriveKinematics = new SwerveDriveKinematics(
        DrivetrainConstants.SwerveModules.frontLeft.wheelPos,
        DrivetrainConstants.SwerveModules.frontRight.wheelPos,
        DrivetrainConstants.SwerveModules.backLeft.wheelPos,
        DrivetrainConstants.SwerveModules.backRight.wheelPos);
  }

  public void userDrive(double forward, double side, double rotate, boolean fieldOrient, boolean boost) {
    forward = Math.pow(forward, 3);
    side = Math.pow(side, 3);
    rotate = Math.pow(rotate, 3);

    convertSpeeds(forward, side, rotate, fieldOrient, boost);
  }

  public void convertSpeeds(double forward, double side, double rotate, boolean fieldOrient, boolean boost) {
    double forwardVelocity = boost ? forward * DrivetrainConstants.maxSpeed : forward * OperatorConstants.normalSpeed; //convert % speed to MPS and apply boost
    double sideVelocity = boost ? side *  DrivetrainConstants.maxSpeed : side * OperatorConstants.normalSpeed; //same ^
    double rotateVelocity = boost ? rotate : rotate; // same^
    ChassisSpeeds speeds = new ChassisSpeeds(forwardVelocity, sideVelocity, rotateVelocity);
    if(fieldOrient){
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, null); //needs function to get angle from pig
    }
    double movementAngle = Math.atan2(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double linearVelocity = Math.sqrt((Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2))); //pythagorean theorem
    linearVelocity = translationLimiter.calculate(linearVelocity);
    rotateVelocity = rotationLimiter.calculate(rotateVelocity);
    forwardVelocity = Math.sin(movementAngle) * linearVelocity;
    sideVelocity = Math.cos(movementAngle) * linearVelocity;
    speeds = new ChassisSpeeds(forwardVelocity, sideVelocity, rotateVelocity);
    applySpeeds(speeds);
  }

  public void applySpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] states = swerveDriveKinematics.toWheelSpeeds(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.maxSpeed);
    for(int i = 0; i < 4; i++){
      //apply speeds 2 each wheel
      modules[i].setDesiredState(states[i]);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
