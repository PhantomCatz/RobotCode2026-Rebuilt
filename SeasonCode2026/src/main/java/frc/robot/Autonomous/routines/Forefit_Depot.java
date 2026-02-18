package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;

public class Forefit_Depot extends AutoRoutineBase{
    public Forefit_Depot(){
        super("Forefit_Outpost");

        AutoTrajectory traj1 = getTrajectory("Forefit_Depot",0);
        AutoTrajectory traj2 = getTrajectory("Forefit_Depot",1);
        AutoTrajectory traj3 = getTrajectory("Forefit_Depot",2);
        AutoTrajectory traj4 = getTrajectory("Forefit_Depot",3);
        AutoTrajectory traj5 = getTrajectory("Forefit_Depot",4);
        AutoTrajectory traj6 = getTrajectory("Forefit_Depot", 5);

        traj1.atTime("Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));


        prepRoutine(
            traj1,
            CatzSuperstructure.Instance.toggleIntakeDeploy(),
            shootAllBalls(AutonConstants.PRELOAD_SHOOTING_WAIT),
            followTrajectoryWithAccuracy(traj1),
            CatzSuperstructure.Instance.cmdHoardShoot(),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            followTrajectoryWithAccuracy(traj5),
            followTrajectoryWithAccuracy(traj6),
            CatzSuperstructure.Instance.cmdFullStop(),
            Commands.print("done")
        );
    }
}
