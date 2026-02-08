package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;

public class DepotClimb extends AutoRoutineBase{
    public DepotClimb(){
        super("DepotClimb");

        AutoTrajectory traj1 = getTrajectory("DepotClimb",0);
        AutoTrajectory traj2 = getTrajectory("DepotClimb",1);
        AutoTrajectory traj3 = getTrajectory("DepotClimb",2);
        AutoTrajectory traj4 = getTrajectory("DepotClimb",3);
        AutoTrajectory traj5 = getTrajectory("DepotClimb",4);

        traj1.atTime("Score1").onTrue(Commands.print("Score1"));
        traj1.atTime("Intake2").onTrue(Commands.print("Intake2"));
        traj3.atTime("Score3").onTrue(Commands.print("Score3"));

        prepRoutine(
            traj1,
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            followTrajectoryWithAccuracy(traj5),
            Commands.print("Climb")
        );
    }
}
