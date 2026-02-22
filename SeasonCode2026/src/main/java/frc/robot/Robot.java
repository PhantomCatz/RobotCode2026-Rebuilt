package frc.robot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;

import choreo.auto.AutoFactory;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.CatzConstants.RobotHardwareMode;
import frc.robot.CatzConstants.RobotID;
import frc.robot.Autonomous.AutoRoutineSelector;
import frc.robot.CatzAbstractions.Bases.GenericMotorSubsystem;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimb;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzIndexer.CatzSpindexer.CatzSpindexer;
import frc.robot.CatzSubsystems.CatzIndexer.CatzYdexer.CatzYdexer;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeDeploy.CatzIntakeDeploy;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzShooter.CatzFlywheels.CatzFlywheels;
import frc.robot.CatzSubsystems.CatzShooter.CatzHood.CatzHood;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.Utilities.VirtualSubsystem;

public class Robot extends LoggedRobot {
  private RobotContainer m_robotContainer;

  private Command m_autonomousCommand;

  private BaseStatusSignal[] allSignals;
  private GenericMotorSubsystem[] allSubsystems = new GenericMotorSubsystem[8];

  public static double autonStartTime = 0.0;

  public Robot() {
  }

  @Override
  public void robotInit() {

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
    allSubsystems[0] = CatzClimb.Instance;
    allSubsystems[1] = CatzSpindexer.Instance;
    allSubsystems[2] = CatzYdexer.Instance;
    allSubsystems[3] = CatzIntakeDeploy.Instance;
    allSubsystems[4] = CatzIntakeRoller.Instance;
    allSubsystems[5] = CatzFlywheels.Instance;
    allSubsystems[6] = CatzHood.Instance;
    allSubsystems[7] = CatzTurret.Instance;

    m_robotContainer = new RobotContainer();

    CatzConstants.autoFactory = new AutoFactory(
                                                  CatzRobotTracker.getInstance()::getEstimatedPose,
                                                  CatzRobotTracker.getInstance()::resetPose,
                                                  CatzDrivetrain.getInstance()::followChoreoTrajectoryExecute,
                                                  true,
                                                  CatzDrivetrain.getInstance()
                                                ); //it is apparently a good idea to initialize these variables not statically because there can be race conditions
    System.out.println(AutoRoutineSelector.Instance);

      DriverStation.silenceJoystickConnectionWarning(true);

      if (CatzConstants.hardwareMode == CatzConstants.RobotHardwareMode.REAL ||
        CatzConstants.hardwareMode == CatzConstants.RobotHardwareMode.REPLAY) {
        List<BaseStatusSignal> signalList = new ArrayList<>();
        for(GenericMotorSubsystem subsystem : allSubsystems){
          if(subsystem == null){
            System.out.println("subsystem is null !!!!!!!!!!!!!\n\n\n\n\n\n\n\n\nwowwwwwwwwwwwwwwww\n\n\n\n\n\n\n\n!!!!!!!!!!!!!!!!!!!!");
          }
          Collections.addAll(signalList, subsystem.getSignals());
        }
        allSignals = signalList.toArray(new BaseStatusSignal[0]);
      }else{
        allSignals = new BaseStatusSignal[0];
      }

      System.out.println("Chooser: " + AutoRoutineSelector.Instance);

      // Notifier coralDetectionThread = new Notifier(Detection.Instance::setNearestGroupPose);
      // Notifier.setHALThreadPriority(false, 0);
      // System.out.println("Starting deteciton threaadf==================");
      // coralDetectionThread.startPeriodic(0.1);
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
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    autonStartTime = Timer.getFPGATimestamp();
    m_autonomousCommand = AutoRoutineSelector.Instance.getSelectedCommand();
    CatzTurret.Instance.setCurrentPosition(Units.Rotations.of(CatzTurret.Instance.getCANCoderAbsPos()));

    System.out.println("auton: " + m_autonomousCommand);
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
