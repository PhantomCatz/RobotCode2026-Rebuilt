package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;

import org.wpilib.system.RobotController;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import frc.robot.CatzConstants.RobotHardwareMode;
import frc.robot.CatzConstants.RobotID;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.Utilities.VirtualSubsystem;

public class Robot extends LoggedRobot {
  private RobotContainer m_robotContainer;

  private Command m_autonomousCommand;

  private BaseStatusSignal[] allSignals;

  public static double autonStartTime = 0.0;
  public static boolean climbedInAuton = false;

  public Robot() {
    System.gc();
    switch (CatzConstants.hardwareMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter("/U/logs")); ///"home/lvuser/logs"
        Logger.addDataReceiver(new RLOGServer());
        Logger.addDataReceiver(new WPILOGWriter("/Logs"));

        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        // Logger.addDataReceiver(new WPILOGWriter("F:/robotics code
        // projects/loggingfiles/"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();

    // Log active commands
    // Map<String, Integer> commandCounts = new HashMap<>();
    // BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
    //   String name = command.getName();
    //   int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
    //   commandCounts.put(name, count);
    //   Logger.recordOutput(
    //       "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
    //   Logger.recordOutput("CommandsAll/" + name, count > 0);
    // };
    // CommandScheduler.getInstance()
    //     .onCommandInitialize(
    //         (Command command) -> {
    //           logCommandFunction.accept(command, true);
    //         });
    // CommandScheduler.getInstance()
    //     .onCommandFinish(
    //         (Command command) -> {
    //           logCommandFunction.accept(command, false);
    //         });
    // CommandScheduler.getInstance()
    //     .onCommandInterrupt(
    //         (Command command) -> {
    //           logCommandFunction.accept(command, false);
    //         });

    // Set Brownout Voltage to WPILIB recommendations
    RobotController.setBrownoutVoltage(6.3);

    // Print out Catz Constant enums
    System.out.println("Enviroment: " + CatzConstants.robotScenario.toString());
    System.out.println("Mode: " + CatzConstants.hardwareMode.toString());
    System.out.println("Type: " + CatzConstants.getRobotType().toString());
    SignalLogger.enableAutoLogging(false);

    // Run hardware mode check
    if (Robot.isReal()) { // REAL ROBOT
      if (CatzConstants.hardwareMode == RobotHardwareMode.SIM) {
        System.out.println("Wrong Robot Constant selection, Check CatzConstants hardwareMode");
        System.exit(0);
      }

      if (CatzConstants.getRobotType() == RobotID.SN_TEST) {
        System.out.println("Wrong Robot ID selection, Check CatzConstants robotID");
        System.exit(0);
      }

    } else { // SIM ROBOT
      if (CatzConstants.hardwareMode == RobotHardwareMode.REAL) {
        System.out.println("Wrong Robot Constant selection, Check CatzConstants hardwareMode");
        System.exit(0);
      }

      if (CatzConstants.getRobotType() != RobotID.SN_TEST) {
        if (CatzConstants.hardwareMode == RobotHardwareMode.SIM) {
          System.out.println("Wrong Robot ID selection, Check CatzConstants robotID");
          System.exit(0);
        }
      }
    }

    m_robotContainer = new RobotContainer();

      // Notifier coralDetectionThread = new Notifier(Detection.Instance::setNearestGroupPose);
      // Notifier.setHALThreadPriority(false, 0);
      // System.out.println("Starting deteciton threaadf==================");
      // coralDetectionThread.startPeriodic(0.1);
      SmartDashboard.putBoolean("Won Auton?", false);
      SmartDashboard.putBoolean("Swiping?", false);
  }

  @Override
  public void robotPeriodic() {
    VirtualSubsystem.periodicAll();
    if(allSignals.length > 0) {
      BaseStatusSignal.refreshAll(allSignals);
    }
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle_set").setNumber(200);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    CatzDrivetrain.getInstance().setNormalConfig();

    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle_set").setNumber(0);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // if (climbedInAuton) {
    //   CatzSuperstructure.Instance.autoClimbLowerCommand().schedule();
    // }
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {
  }


}
