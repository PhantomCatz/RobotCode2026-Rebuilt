package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;

public class DepotClimb extends AutoRoutineBase{
    public DepotClimb(){
        super("DepotClimb");
        AutoTrajectory traj1 = getTrajectory("DepotClimb",0);
        AutoTrajectory traj2 = getTrajectory("DepotClimb",1);
        AutoTrajectory traj3 = getTrajectory("DepotClimb",2);
        AutoTrajectory traj4 = getTrajectory("DepotClimb",3);
        AutoTrajectory traj5 = getTrajectory("DepotClimb",4);

        prepRoutine(
            traj1,
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    followTrajectoryWithAccuracy(traj1),
                    Commands.waitUntil(traj1.atPose("Score1", AutonConstants.EM_ACCEPTABLE_DIST_METERS, AutonConstants.EM_ACCEPTABLE_RADS))
                            .andThen(() -> System.out.println("ONE \n"))
                ),
                Commands.waitSeconds(5),
                new ParallelCommandGroup(
                    followTrajectoryWithAccuracy(traj2),
                    Commands.waitUntil(traj2.atPose("Intake2", AutonConstants.EM_ACCEPTABLE_DIST_METERS, AutonConstants.EM_ACCEPTABLE_RADS))
                            .andThen(() -> System.out.println("TWO \n"))
                )
            )
        );
    }
}
