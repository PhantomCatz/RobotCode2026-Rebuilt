package frc.robot.Autonomous.routines;


import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;

public class Test extends AutoRoutineBase{
    public Test(){
        super("Test");

        AutoTrajectory traj1 = getTrajectory("crashingInTheNameOfTestingPID",0);
        AutoTrajectory traj2 = getTrajectory("crashingInTheNameOfTestingPID",1);

        prepRoutine(
            traj1,
            Commands.waitSeconds(AutonConstants.DEPLOY_INTAKE_WAIT),
            CatzSuperstructure.Instance.deployIntake(),
            //"pray for me" - kendrick lamar
            followTrajectory(traj1),
            followTrajectory(traj2),
            Commands.print("Done")
        );
    }
}
