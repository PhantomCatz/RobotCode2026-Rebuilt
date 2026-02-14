package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
<<<<<<< Updated upstream
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;
import frc.robot.CatzSubsystems.CatzShooter.regressions.ShooterRegression.RegressionMode;

public class R2_IAS extends AutoRoutineBase {
    public R2_IAS(){
        super("R2_IAS");

        AutoTrajectory traj1 = getTrajectory("R2IAS",0);
        AutoTrajectory traj2 = getTrajectory("R2IAS",1);
        AutoTrajectory traj3 = getTrajectory("R2IAS",2);
        AutoTrajectory traj4 = getTrajectory("R2IAS",3);
        AutoTrajectory traj5 = getTrajectory("R2IAS",4);
        AutoTrajectory traj6 = getTrajectory("R2IAS",5);

        traj1.atTime("Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj2.atTime("IntakeStop+RampUp").onTrue(CatzSuperstructure.Instance.trackTargetAndRampUp(RegressionMode.HUB)
                                                   .alongWith(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)));
        traj3.atTime("Intake4").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj3.atTime("StopIntake+RampUp").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                                                   .alongWith(CatzSuperstructure.Instance.trackTargetAndRampUp(RegressionMode.HUB)));

        prepRoutine(
            traj1,
            shootAllBalls(AutonConstants.PRELOAD_SHOOTING_WAIT),
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT + AutonConstants.PRELOAD_SHOOTING_WAIT),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT),
            followTrajectoryWithAccuracy(traj5),
            followTrajectoryWithAccuracy(traj6),
            Commands.print("Climb"),
            Commands.print("done")
=======
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;
import frc.robot.Commands.DriveAndRobotOrientationCmds.PIDDriveCmd;

<<<<<<<< Updated upstream:SeasonCode2026/src/main/java/frc/robot/Autonomous/routines/R2IASObjDet.java
public class R2IASObjDet extends AutoRoutineBase {
    public R2IASObjDet(){
        super("R2IAS");
========
public class R2_IAS extends AutoRoutineBase {
    public R2_IAS(){
        super("R2_IAS");
>>>>>>>> Stashed changes:SeasonCode2026/src/main/java/frc/robot/Autonomous/routines/R2_IAS.java

        AutoTrajectory traj1 = getTrajectory("R2IASOut",0);
        AutoTrajectory traj2 = getTrajectory("R2IASIn",1);

        // PIDDriveCmdFuel collectCoral = new PIDDriveCmdFuel(new Pose2d(new Translation2d(8.270783424377441, 4.06059074401855), traj2.getInitialPose().get().getRotation()), AutonConstants.TRAJ_GOAL_VELOCITY);

        PIDDriveCmd returnToTrench = new PIDDriveCmd(
                                        traj2.getInitialPose().get(),
                                        AutonConstants.TRAJ_GOAL_VELOCITY,
                                        1.0,
                                        0.1,
                                        5,
                                        false
                                    );
        prepRoutine(
            traj1,
            CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT),
            followTrajectory(traj1),
            // collectCoral,
            CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT),
            returnToTrench,
            followTrajectoryWithAccuracy(traj2)
>>>>>>> Stashed changes
        );
    }
}
