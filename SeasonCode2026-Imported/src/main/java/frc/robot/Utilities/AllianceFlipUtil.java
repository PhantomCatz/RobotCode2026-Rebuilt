package frc.robot.Utilities;

import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.driverstation.Alliance;

public class AllianceFlipUtil {
  public static boolean shouldFlip() {
    return DriverStationBackend.getAlliance().isPresent()
        && DriverStationBackend.getAlliance().get() == Alliance.RED;
  }
}
