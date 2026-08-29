package frc.robot;

import org.wpilib.framework.RobotBase;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.Alert.AlertType;

public final class CatzConstants {

  // ----------------------------------------------------
  //
  //  Robot Modes
  //
  // --------------------------------------------------
  public static final RobotScenario robotScenario = RobotScenario.TUNING;
  public static final RobotHardwareMode hardwareMode = RobotHardwareMode.SIM;
  private static RobotID robotType = RobotID.SN_TEST;

  public static final double LOOP_TIME = 0.02;

  public static enum RobotScenario {
    TUNING, // In PID enviroment with logged tunable numbers
    PRACTICE, // Driver Practice + Testing
    COMPETITION // Competition Setting
  }

  public static enum RobotHardwareMode {
    REAL,
    SIM,
    REPLAY
  }

  public static RobotID getRobotType() {
    // Checks to ensure that the selected robot Hardware mode is not paired with an illegal Robot Id
    if (RobotBase.isReal() && robotType == RobotID.SN_TEST) {
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.kError)
          .set(true);
      robotType = RobotID.SN_MANTA;
    }
    return robotType;
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (robotType == RobotID.SN_TEST) {
        System.err.println("Cannot deploy, invalid robot selected: " + robotType);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (robotType != RobotID.SN_TEST || robotScenario == RobotScenario.TUNING) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }

  public static enum RobotID {
    SN1,
    SN_MANTA,
    SN2,
    SN1_OLD,
    SN_TEST, // Select alternate test robot parameters
    BUBBLES
  }

  public static final class XboxInterfaceConstants {
    // Xbox Driver Ports
    public static final int XBOX_DRV_PORT = 0;

    // Deadbands
    public static final double kDeadband = 0.1;
  }
}
