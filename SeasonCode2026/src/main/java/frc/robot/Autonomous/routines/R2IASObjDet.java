package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
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
        );
    }
}
