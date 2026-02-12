package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.Commands.DriveAndRobotOrientationCmds.PIDDriveCmd;
import frc.robot.Commands.DriveAndRobotOrientationCmds.PIDDriveCmdFuel;
import frc.robot.Utilities.AllianceFlipUtil;

public class Testy extends AutoRoutineBase {
    public Testy(){
        super("Testy");

        AutoTrajectory traj1 = getTrajectory("TestyOut",0);
        AutoTrajectory traj2 = getTrajectory("TestyIn",1);

        PIDDriveCmdFuel collectCoral = new PIDDriveCmdFuel(
            new Pose2d(AllianceFlipUtil.apply(new Translation2d(2.3418190479278564, 1.5577303171157837)), traj2.getInitialPose().get().getRotation()),
            AutonConstants.TRAJ_GOAL_VELOCITY,
            traj2.getInitialPose().get(),
            traj2.getRawTrajectory().getTotalTime());

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
            followTrajectory(traj1),
            Commands.print("finish traj1"),
            collectCoral,
            Commands.print("finish collecting"),
            returnToTrench,
            Commands.print("finish returning to trench"),
            followTrajectoryWithAccuracy(traj2)
        );
    }
}
