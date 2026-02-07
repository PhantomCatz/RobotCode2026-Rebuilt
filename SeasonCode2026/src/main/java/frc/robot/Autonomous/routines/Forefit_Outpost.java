package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomous.AutoRoutineBase;

public class Forefit_Outpost extends AutoRoutineBase{
    public Forefit_Outpost(){
        super("Forefit_Outpost");

        AutoTrajectory traj1 = getTrajectory("Forefit_Outpost",0);
        AutoTrajectory traj2 = getTrajectory("Forefit_Outpost",1);
        AutoTrajectory traj3 = getTrajectory("Forefit_Outpost",2);
        AutoTrajectory traj4 = getTrajectory("Forefit_Outpost",3);
        AutoTrajectory traj5 = getTrajectory("Forefit_Outpost",4);

        traj1.atTime("Score1").onTrue(Commands.print("Shoot"));
        traj1.atTime("Intake2").onTrue(Commands.print("Intake"));
        traj2.atTime("Hoard3").onTrue(Commands.print("Hoard"));


        prepRoutine(
            traj1,
            new SequentialCommandGroup(
                followTrajectoryWithAccuracy(traj1),
                followTrajectoryWithAccuracy(traj2),
                followTrajectoryWithAccuracy(traj3),
                followTrajectoryWithAccuracy(traj4),
                followTrajectoryWithAccuracy(traj5)
            ),
            Commands.print("done")
        );
    }
}
