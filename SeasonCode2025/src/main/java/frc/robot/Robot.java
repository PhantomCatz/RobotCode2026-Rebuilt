// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.CatzConstants.RobotHardwareMode;
import frc.robot.CatzConstants.RobotID;
import frc.robot.CatzSubsystems.CatzArm.CatzArm;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private static Pose3d[] simMechanismPoses = {new Pose3d()};

    // public static final AutoFactory autoFactory = new AutoFactory(
    //                                               CatzRobotTracker.Instance::getEstimatedPose,
    //                                               CatzRobotTracker.Instance::resetPose,
    //                                               CatzDrivetrain.Instance::followChoreoTrajectoryExecute,
    //                                               true,
    //                                               CatzDrivetrain.Instance
    //                                             );

    // public Robot() {
    //     Logger.recordMetadata("ProjectName", "SeasonCode2025"); // Set a metadata value

    //     if (isReal()) {
    //         Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    //         Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    //     } else {
    //         setUseTiming(false); // Run as fast as possible
    //         String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    //         Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    //         Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    //     }

    //     Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
      
    // }

    @Override
    public void robotInit() {
        System.gc();
        System.out.println("robot init");
        switch (CatzConstants.hardwareMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
                Logger.addDataReceiver(new RLOGServer());
                Logger.addDataReceiver(new WPILOGWriter("/Logs"));

                Logger.addDataReceiver(new NT4Publisher());
                break;
            
            case SIM:
                // Running a physics simulator, log to NT
                // Logger.addDataReceiver(new WPILOGWriter("F:/robotics code projects/loggingfiles/"));
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
        Map<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction =
            (Command command, Boolean active) -> {
            String name = command.getName();
            int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
            commandCounts.put(name, count);
            Logger.recordOutput(
                "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
            Logger.recordOutput("CommandsAll/" + name, count > 0);
            };
        CommandScheduler.getInstance()
            .onCommandInitialize(
                (Command command) -> {
                logCommandFunction.accept(command, true);
                });
        CommandScheduler.getInstance()
            .onCommandFinish(
                (Command command) -> {
                logCommandFunction.accept(command, false);
                });
        CommandScheduler.getInstance()
            .onCommandInterrupt(
                (Command command) -> {
                logCommandFunction.accept(command, false);
                });

        // Set Brownout Voltage to WPILIB recommendations
        RobotController.setBrownoutVoltage(6.3);

        // Print out Catz Constant enums
        System.out.println("Enviroment: " + CatzConstants.robotScenario.toString());
        System.out.println("Mode: " + CatzConstants.hardwareMode.toString());
        System.out.println("Type: " + CatzConstants.getRobotType().toString());

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

        // DELETE forcing instantiation because of lazy initialization, idk how to fix
        System.out.println("Forcing arm instance: " + CatzArm.Instance);
        System.out.println("Forcing robot container instance" + RobotContainer.Instance);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = RobotContainer.Instance.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        Logger.recordOutput("FinalComponentPoses", simMechanismPoses);
    }

    public static void setSimPose(int index, Pose3d pose) {
        simMechanismPoses[index] = pose;
    }
}
