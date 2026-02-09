package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;

public class R3IAS extends AutoRoutineBase{
    public R3IAS(){
        super("R3IAS");

        AutoTrajectory traj1 = getTrajectory("R3IAS",0);
        AutoTrajectory traj2 = getTrajectory("R3IAS",1);
        AutoTrajectory traj3 = getTrajectory("R3IAS",2);
        AutoTrajectory traj4 = getTrajectory("R3IAS",3);
        AutoTrajectory traj5 = getTrajectory("R3IAS",4);
        AutoTrajectory traj6 = getTrajectory("R3IAS",5);

        traj1.atTime("Intake2").onTrue(Commands.print("Intake2"));
        traj2.atTime("RampUp3").onTrue(Commands.print("RampUp3"));
        traj2.atTime("Score3").onTrue(Commands.print("Score3"));
        traj3.atTime("Intake4").onTrue(Commands.print("Intake4"));
        traj4.atTime("Score5").onTrue(Commands.print("Score5"));

        prepRoutine(
            traj1,
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            followTrajectoryWithAccuracy(traj5),
            followTrajectoryWithAccuracy(traj6)
        );
    }
}
