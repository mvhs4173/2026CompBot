// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {
  
    public double trackWidth = Units.inchesToMeters(0);
    public enum SwerveModules {
      frontLeft(null, 10, 11, false), 
      frontRight(null, 20, 21, false),
      backLeft(null, 30, 31, false),
      backRight(null, 40, 41, false);
      public Translation2d wheelPos;
      public int turnID;
      public int driveID;
      public boolean driveReversed;
      public double chassisAngularOffset;
      private SwerveModules(Translation2d wheelPos, int turnID, int driveID, boolean driveReversed) {
        this.wheelPos = wheelPos;
        this.turnID = turnID;
        this.driveID = driveID;
        this.driveReversed = driveReversed;
      }
    }
  }

  public static class IntakeConstants {

  }

  public static class ShooterConstants {

  }

  public static class AutoConstants {
    
  }
  
  public static class VisionConstants {

  }

}
