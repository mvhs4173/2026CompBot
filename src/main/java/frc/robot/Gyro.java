package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;

public class Gyro extends SubsystemBase {

  /** Creates a new Gyro. */
  private Pigeon2 m_pigeon;
  private AHRS m_navX;
  private boolean m_usePigeon;
  private double m_yawOffsetPigeon2;
  private double m_yawOffsetNavX;

  public Gyro(boolean usePigeon) {
    m_usePigeon = true;
    m_pigeon = new Pigeon2(DrivetrainConstants.pigeonID);
    // StatusCode statusCode = m_pigeon.getConfigurator().apply(new Pigeon2Configuration()); // replaces .configFactoryDefault()
    // if (statusCode.isError()) {
    //   System.out.println("Pigeon2 has a problem: " + statusCode);
    //   m_pigeon = null;
    //   m_usePigeon = false;
    // }
    m_pigeon.reset();
    m_navX = new AHRS(NavXComType.kMXP_SPI);
    // calibrateNavX();
    // m_usePigeon = Constants.Swerve.usePigeon;
    setYaw(0.0);
  }

  public void setYaw(double degreesCcw) {
    if (m_pigeon != null) {
      m_pigeon.reset();
    }
    m_navX.reset();
    m_yawOffsetPigeon2 = degreesCcw;
    m_yawOffsetNavX = degreesCcw + m_navX.getYaw();
  }

  private double getYawPigeon2() {
    return m_pigeon != null
      ? m_pigeon.getYaw().getValueAsDouble() + m_yawOffsetPigeon2
      : 0.0;
  }

  private double getYawNavX() {
    return -m_navX.getYaw() + m_yawOffsetNavX;
  }

  public double getYaw() {
    if (m_usePigeon) {
      return getYawPigeon2();
    } else {
      return getYawNavX();
    }
  }

  public double getRoll() {
    if (m_usePigeon) {
      return m_pigeon != null ? m_pigeon.getRoll().getValueAsDouble() : 0.0;
    } else {
      return m_navX.getRoll();
    }
  }

  public double getPitch() {
    if (m_usePigeon) {
      return m_pigeon != null ? m_pigeon.getPitch().getValueAsDouble() : 0.0;
    } else {
      return m_navX.getPitch();
    }
  }

  public void toggleGyro() {
    if (m_pigeon == null) {
      System.out.println("Cannot toggle gyro, pigeon2 does not work");
    } else {
      m_usePigeon = !(m_usePigeon);
    }
  }

  public void resetGyro() {
    if (m_usePigeon) {
      m_pigeon.reset();
    } else {
      m_navX.reset();
    }
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(getYaw());
  }

  /**
   * Make sure gyro is done calibrating before using it
   * @return
   * true if calibration was successful, false if we could not connect to or could not calibrate NavX
   */
  private boolean calibrateNavX() {
    // calibration only needed for NavX
    int nTries = 1;
    boolean retval = true;
    while (m_navX.isCalibrating() && nTries < 100) { //wait to zero yaw if calibration is still running
      try {
        Thread.sleep(20);
        System.out.println("----calibrating gyro---- " + nTries);
      } catch (InterruptedException e) {}
      nTries++;
      if (nTries >= 50 && nTries % 10 == 0) {
        System.out.println("Having trouble calibrating NavX");
      }
    }
    try {
      Thread.sleep(60); // sometimes isConnected returns false immediately after calibration
    } catch (InterruptedException e) {
      // do nothing
    }
    if (m_navX.isCalibrating()) {
      System.out.println("Could not calibrate NavX, will use Pigeon2");
      retval = false;
    } else if (!m_navX.isConnected()) {
      System.out.println(
        "NavX is not connected (is SPI dip switch not ON?), will use Pigeon2"
      );
      retval = false;
    }
    return retval;
  }

  @Override
  public void periodic() {}
}
