package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Autonomous.AutoRoutineBase;
import frc.robot.Autonomous.AutonConstants;
import frc.robot.CatzSubsystems.CatzSuperstructure;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;

public class Forefit_Outpost extends AutoRoutineBase{
    public Forefit_Outpost(){
        super("Forefit_Outpost");

        AutoTrajectory traj1 = getTrajectory("Forefit_Outpost",0);
        AutoTrajectory traj2 = getTrajectory("Forefit_Outpost",1);
        AutoTrajectory traj3 = getTrajectory("Forefit_Outpost",2);
        AutoTrajectory traj4 = getTrajectory("Forefit_Outpost",3);
        AutoTrajectory traj5 = getTrajectory("Forefit_Outpost",4);
        AutoTrajectory traj6 = getTrajectory("Forefit_Outpost",5);
        AutoTrajectory traj7 = getTrajectory("Forefit_Outpost",6);

        traj2.atTime("Intake2").onTrue(CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT));
        traj3.atTime("Hoard3").onTrue(CatzSuperstructure.Instance.cmdHoardShoot())
        prepRoutine(
            traj1,
            CatzSuperstructure.Instance.toggleIntakeDeploy(),
            followTrajectoryWithAccuracy(traj1),
            followTrajectoryWithAccuracy(traj2),
            followTrajectoryWithAccuracy(traj3),
            followTrajectoryWithAccuracy(traj4),
            followTrajectoryWithAccuracy(traj5),
            followTrajectoryWithAccuracy(traj6),
            followTrajectoryWithAccuracy(traj7),
            CatzSuperstructure.Instance.cmdFullStop(),
            Commands.print("done")
        );
    }
}
