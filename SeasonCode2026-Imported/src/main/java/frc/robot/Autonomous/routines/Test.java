package frc.robot.Autonomous.routines;


import choreo.auto.AutoTrajectory;
import org.wpilib.command2.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;

public class Test extends AutoRoutineBase{
    public Test(){
        super("Test");
            //i hate pid
        AutoTrajectory traj1 = getTrajectory("Depot_2_Cycle_Testing",0);
        AutoTrajectory traj2 = getTrajectory("Depot_2_Cycle_Testing",1);


        prepRoutine(
            traj1,
            shootAllBalls(AutonConstants.RETURN_FROM_COLLECTING_SHOOTING_WAIT + AutonConstants.PRELOAD_SHOOTING_WAIT + AutonConstants.OUTPOST_SCORING_WAIT),
            Commands.print("done")
        );
    }
}
