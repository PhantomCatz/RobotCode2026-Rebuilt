package frc.robot;


import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import choreo.auto.AutoFactory;
import org.wpilib.units.Units;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.driverstation.Alliance;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import frc.robot.CatzConstants.RobotHardwareMode;
import frc.robot.CatzConstants.RobotID;
import frc.robot.Autonomous.AutoRoutineSelector;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimb;
import frc.robot.CatzSubsystems.CatzClimb.ClimbConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeDeploy.CatzIntakeDeploy;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeDeploy.IntakeDeployConstants;
import frc.robot.CatzSubsystems.CatzLEDs.CatzLED;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.CatzTurret;
import frc.robot.CatzSubsystems.CatzVision.Detection.DetectionConstants;
import frc.robot.Utilities.CANBusManager;
import frc.robot.Utilities.LimelightHelpers;
import frc.robot.CatzSubsystems.CatzVision.Detection.Detection;
import frc.robot.Utilities.Setpoint;
import frc.robot.Utilities.VirtualSubsystem;

public class Robot extends LoggedRobot {
  private RobotContainer m_robotContainer;

  private Command m_autonomousCommand;

  public static double autonStartTime = 0.0;
  public static boolean climbedInAuton = false;

  private int iterations = 0;

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

    CatzIntakeDeploy.Instance.setDefaultCommand(
      Commands.run(() -> {
        CatzIntakeDeploy.Instance.applySetpoint(Setpoint.withMotionMagicSetpoint(CatzSuperstructure.Instance.intakeSetpoint));
      }, CatzIntakeDeploy.Instance)
  );

    CatzClimb.Instance.setDefaultCommand(
      Commands.run(() -> {
        CatzClimb.Instance.applySetpoint(Setpoint.withMotionMagicSetpoint(CatzSuperstructure.Instance.blockerSetpoint));
      }, CatzClimb.Instance)
    );

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
    RobotController.setBrownoutVoltages(6.3, 7);

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

    CatzRobotTracker.getInstance();
    m_robotContainer = new RobotContainer();

    CatzConstants.autoFactory = new AutoFactory(
                                                  CatzRobotTracker.getInstance()::getEstimatedPose,
                                                  CatzRobotTracker.getInstance()::resetPose,
                                                  CatzDrivetrain.getInstance()::followChoreoTrajectoryExecute,
                                                  true,
                                                  CatzDrivetrain.getInstance()
                                                ); //it is apparently a good idea to initialize these variables not statically because there can be race conditions

      DriverStationBackend.silenceJoystickConnectionAlert(true);

      System.out.println("Chooser: " + AutoRoutineSelector.Instance);
      System.out.println("Led "+CatzLED.Instance);
      System.out.println("Initializing " + Detection.Instance.getName());

      // Notifier coralDetectionThread = new Notifier(Detection.Instance::setNearestGroupPose);
      // Notifier.setHALThreadPriority(false, 0);
      // System.out.println("Starting deteciton threaadf==================");
      // coralDetectionThread.startPeriodic(0.1);
      Telemetry.log("Won Auton?", false);
      Telemetry.log("Swiping?", false);

      LimelightHelpers.setRewindEnabled(DetectionConstants.kLimelightName, true);
  }

  @Override
  public void robotPeriodic() {
    if (CatzConstants.hardwareMode == CatzConstants.RobotHardwareMode.REAL) {
      CANBusManager.refreshAll(); //chat is this tuff or is this tuff
    }

    VirtualSubsystem.periodicAll();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle_set").setNumber(200);
    LimelightHelpers.triggerRewindCapture(DetectionConstants.kLimelightName, 165);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle_set").setNumber(0);
    autonStartTime = Timer.getTimestamp();
    CatzTurret.Instance.setCurrentPosition(Units.Rotations.of(CatzTurret.Instance.getCANCoderAbsPos()));
    CatzIntakeDeploy.Instance.setCurrentPosition(IntakeDeployConstants.HOME_POSITION);
    CatzClimb.Instance.setCurrentPosition(ClimbConstants.STOW_POSITION);
    m_autonomousCommand = AutoRoutineSelector.Instance.getSelectedCommand();

    System.out.println("-------- SELECTED AUTON: " + m_autonomousCommand);
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    CommandScheduler.getInstance().schedule(CatzSuperstructure.Instance.trackStaticHub());
  }

  @Override
  public void teleopInit() {
    CatzSuperstructure.Instance.intakeSetpoint = IntakeDeployConstants.DEPLOY_POSITION;
    CatzSuperstructure.Instance.isIntakeDeployed = true;

    CatzSuperstructure.Instance.isBlockerDeployed = false;
    CatzSuperstructure.Instance.blockerSetpoint = ClimbConstants.STOW_POSITION;

    CommandScheduler.getInstance().schedule(CatzSuperstructure.Instance.cmdShooterStop());
    CatzDrivetrain.getInstance().setNormalConfig();

    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle_set").setNumber(0);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // if (climbedInAuton) {
    //   CatzSuperstructure.Instance.autoClimbLowerCommand().schedule();
    // }
    // CommandScheduler.getInstance().schedule(CatzSuperstructure.Instance.trackStaticHub());
  }

  @Override
  public void teleopPeriodic() {
    if(iterations < 20) {
      Alliance alliance = DriverStationBackend.getAlliance().orElse(Alliance.BLUE);
      // System.out.println("hello world!\n" + "\"" + DriverStationBackend.getGameData() + "\"" + " \n boom it");
      try{
        if ((DriverStationBackend.getGameData().toString().charAt(0) == 'B'
          && alliance == Alliance.BLUE)
          ||(DriverStationBackend.getGameData().toString().charAt(0) == 'R'
          && alliance == Alliance.RED)
          )
        {
          Telemetry.log("Won Auton?", true);
          iterations = 21;
        }
      } catch(Exception e){
        e.printStackTrace();
      }
      iterations++;
    }
  }

  @Override
  public void teleopExit() {
  }

  // @Override
  // public void utilityInit() {
  //   CommandScheduler.getInstance().cancelAll();
  // }

  // @Override
  // public void utilityPeriodic() {
  // }

  // @Override
  // public void utilityExit() {
  // }

  }
