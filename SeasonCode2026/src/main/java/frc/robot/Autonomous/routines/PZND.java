package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autonomous.AutoRoutineBase;

public class PZND extends AutoRoutineBase{
    public PZND(){
        super("PZND");

        AutoTrajectory traj1 = getTrajectory("PZND",0);
        AutoTrajectory traj2 = getTrajectory("PZND",1);
        AutoTrajectory traj3 = getTrajectory("PZND",2);

        traj1.atTime("Score1").onTrue(Commands.print("Score1"));
        traj1.atTime("Intake2").onTrue(Commands.print("Intake2"));
        traj3.atTime("Score3").onTrue(Commands.print("Score3"));
        traj3.atTime("Intake4").onTrue(Commands.print("Intake4"));

        prepRoutine(
            traj1,
            new SequentialCommandGroup(
                followTrajectoryWithAccuracy(traj1),
                followTrajectoryWithAccuracy(traj2),
                followTrajectoryWithAccuracy(traj3),
                Commands.print("Score5")
            )
        );
    }
}
