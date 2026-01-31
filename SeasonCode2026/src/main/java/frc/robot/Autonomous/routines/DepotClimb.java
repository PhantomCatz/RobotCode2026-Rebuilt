package frc.robot.Autonomous.routines;

import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Autonomous.AutoRoutineBase;

public class DepotClimb extends AutoRoutineBase{
    public DepotClimb(){
        super("DepotClimb");

        AutoTrajectory trajpre = getTrajectory("DepotClimb",0);
        AutoTrajectory traj1 = getTrajectory("DepotClimb",1);
        AutoTrajectory traj2 = getTrajectory("DepotClimb",2);
        AutoTrajectory traj3 = getTrajectory("DepotClimb",3);
        AutoTrajectory traj4 = getTrajectory("DepotClimb",4);
        AutoTrajectory traj5 = getTrajectory("DepotClimb",5);

        prepRoutine(
            trajpre,
            followTrajectoryWithAccuracy(trajpre),
            new ParallelCommandGroup(
                // Commands.waitUntil(traj1.atTime("Score1")).andThen(CatzSuperstructure.Instance.Shoot()),
                // Commands.waitUntil(traj1.atTime("Intake2")).andThen(CatzSuperstructure.Instance.Intake()),
                // Commands.waitUntil(traj1.atTime("Score3")).andThen(CatzSuperstructure.Instance.Shoot()),
                // Commands.waitUntil(traj1.atTime("Climb5")).andThen(CatzSuperstructure.Instance.Climb())
                Commands.waitUntil(traj1.atTime("Score1")).andThen(Commands.print("ONE \n\n")),
                Commands.waitUntil(traj1.atTime("Intake2")).andThen(Commands.print("TWO \n\n")),
                Commands.waitUntil(traj1.atTime("Score3")).andThen(Commands.print("THREE \n\n")),
                Commands.waitUntil(traj1.atTime("Climb5")).andThen(Commands.print("FIVE \n\n"))

            )

            // // Commands.waitUntil(traj1.atTime("Score")).andThen(() -> CatzSuperstructure.Instance.applyShooterSetpoint()), // CatzSuperstructure.Instance.ScoreFuel(),
            // followTrajectoryWithAccuracy(traj1),
            // followTrajectoryWithAccuracy(traj2),
            // // Commands.waitUntil(traj1.atTime("Intake")).andThen(() -> CatzSuperstructure.Instance.Intake()),              // CatzSuperstructure.Instance.IntakeFuel(),
            // followTrajectoryWithAccuracy(traj3),
            // // CatzSuperstructure.Instance.ScoreFuel(),
            // followTrajectoryWithAccuracy(traj4),
            // followTrajectoryWithAccuracy(traj5)
            // // CatzSuperstructure.Instance.Climb(),

        );
    }
}
