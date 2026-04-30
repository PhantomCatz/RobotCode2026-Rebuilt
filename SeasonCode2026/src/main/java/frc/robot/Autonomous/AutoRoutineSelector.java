package frc.robot.Autonomous;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Autonomous.routines.*;

public class AutoRoutineSelector {
    public static final AutoRoutineSelector Instance = new AutoRoutineSelector();

    private AutoChooser autoSelector = new AutoChooser();

    private AutoRoutineSelector(){

        // autoSelector.addRoutine("Better_Outpost_2_Cycle_Outpost", () -> new Better_Outpost_2_Cycle_Outpost().getRoutine());
        // autoSelector.addRoutine("Decon_Depot_1_Cycle", () -> new Decon_Depot_1_Cycle().getRoutine());

        autoSelector.addRoutine("Center_Outpost_Depot_Climb", () -> new Center_Outpost_Depot_Climb().getRoutine());
        autoSelector.addRoutine("Center_Outpost_Depot_Climb_Decon", () -> new Center_Outpost_Depot_Climb_Decon().getRoutine());
        // autoSelector.addRoutine("Center_Outpost_Depot_Neutral", () -> new Center_Outpost_Depot_Neutral().getRoutine());

        // autoSelector.addRoutine("Decon_Depot_2_Cycle_Bump_Fast", () -> new Decon_Depot_2_Cycle_Bump_Fast().getRoutine());
        // autoSelector.addRoutine("Decon_Depot_2_Cycle", () -> new Decon_Depot_2_Cycle().getRoutine());
        // autoSelector.addRoutine("Decon_Outpost_2_Cycle_Climb", () -> new Decon_Outpost_2_Cycle_Climb().getRoutine());
        // autoSelector.addRoutine("Decon_Outpost_2_Cycle_Outpost", () -> new Decon_Outpost_2_Cycle_Outpost().getRoutine());
        // autoSelector.addRoutine("Decon_Outpost_Climb", () -> new Decon_Outpost_Climb().getRoutine());
        // autoSelector.addRoutine("Depot_1_Cycle_Climbb", () -> new Depot_1_Cycle_Climbb().getRoutine());
        // autoSelector.addRoutine("Depot_1_Cycle", () -> new Depot_1_Cycle().getRoutine());
        // autoSelector.addRoutine("Depot_2_Cycle_Bump_Full_Hopper", () -> new Depot_2_Cycle_Bump_Full_Hopper().getRoutine());
        autoSelector.addRoutine("Depot_2_Cycle_Bump", () -> new Depot_2_Cycle_Bump().getRoutine());
        // autoSelector.addRoutine("Depot_2_Cycle_Bump_Mg", () -> new Depot_2_Cycle_Bump_Mg().getRoutine());
        // autoSelector.addRoutine("Depot_2_Cycle", () -> new Depot_2_Cycle().getRoutine());
        // autoSelector.addRoutine("Depot_3_Cycle", () -> new Depot_3_Cycle().getRoutine());
        // autoSelector.addRoutine("Forefit_Depot", () -> new Forefit_Depot().getRoutine());
        // autoSelector.addRoutine("Forefit_Outpost", () -> new Forefit_Outpost().getRoutine());
        // autoSelector.addRoutine("MiddlePath", () -> new MiddlePath().getRoutine());
        // autoSelector.addRoutine("Outpost_1_Cycle_Outpost", () -> new Outpost_1_Cycle_Outpost().getRoutine());
        // autoSelector.addRoutine("Outpost_1_Cycle", () -> new Outpost_1_Cycle().getRoutine());
        autoSelector.addRoutine("Outpost_2_Cycle_Bump", () -> new Outpost_2_Cycle_Bump().getRoutine());
        // autoSelector.addRoutine("Outpost_2_Cycle_Bump_Mg", () -> new Outpost_2_Cycle_Bump_Mg().getRoutine());

        autoSelector.addRoutine("NZ_Hoard_Depot_Bump", () -> new NZ_Hoard_Depot_Bump().getRoutine());
        autoSelector.addRoutine("NZ_Hoard_Depot_Trench", () -> new NZ_Hoard_Depot_Trench().getRoutine());
        autoSelector.addRoutine("NZ_Hoard_Outpost_Bump", () -> new NZ_Hoard_Outpost_Bump().getRoutine());
        autoSelector.addRoutine("NZ_Hoard_Outpost_Trench", () -> new NZ_Hoard_Outpost_Trench().getRoutine());

        autoSelector.addRoutine("New_Swipe_Outpost_Bump", () -> new New_Swipe_Outpost_Bump().getRoutine());
        autoSelector.addRoutine("New_Swipe_Outpost_Trench", () -> new New_Swipe_Outpost_Trench().getRoutine());

        autoSelector.addRoutine("Swipe_Depot", () -> new Swipe_Depot().getRoutine());
        // autoSelector.addRoutine("Swipe_Depot_Outpost", () -> new Swipe_Depot_Outpost().getRoutine());
        autoSelector.addRoutine("Swipe_Outpost", () -> new Swipe_Outpost().getRoutine());
        // autoSelector.addRoutine("Swipe_Outpost_Depot", () -> new Swipe_Outpost_Depot().getRoutine());


        // autoSelector.addRoutine("Steal_Depot_Bump", () -> new Steal_Depot_Bump().getRoutine());
        // autoSelector.addRoutine("Steal_Depot_Trench", () -> new Steal_Depot_Trench().getRoutine());
        // autoSelector.addRoutine("Steal_Outpost_Bump", () -> new Steal_Outpost_Bump().getRoutine());
        // autoSelector.addRoutine("Steal_Outpost_Trench", () -> new Steal_Outpost_Trench().getRoutine());
        // autoSelector.addRoutine("Outpost_2_Cycle_Bump_Elim", () -> new Outpost_2_Cycle_Bump_Elim().getRoutine());
        // autoSelector.addRoutine("Outpost_2_Cycle_Bump_846", () -> new Outpost_2_Cycle_Bump_846().getRoutine());
        // autoSelector.addRoutine("Outpost_2_Cycle_Outpost", () -> new Outpost_2_Cycle_Outpost().getRoutine());
        // autoSelector.addRoutine("Outpost_2_Cycle", () -> new Outpost_2_Cycle().getRoutine());
        // autoSelector.addRoutine("Outpost_3_Cycle", () -> new Outpost_3_Cycle().getRoutine());
        // autoSelector.addRoutine("Outpost_Hammerhead_2_Cycle_Outpost", () -> new Outpost_Hammerhead_2_Cycle_Outpost().getRoutine());
        // autoSelector.addRoutine("Solo_Outpost_2_Cycle_Outpost", () -> new Solo_Outpost_2_Cycle_Outpost().getRoutine());

        autoSelector.addRoutine("Test", () -> new Test().getRoutine());

        // autoSelector.addRoutine("Half_Hoard_Climb_Depot", () -> new Half_Hoard_Climb_Depot().getRoutine());
        // autoSelector.addRoutine("Half_Hoard_Climb_Outpost", () -> new Half_Hoard_Climb_Outpost().getRoutine());
        // autoSelector.addRoutine("Half_Hoard_Cycle_Depot", () -> new Half_Hoard_Cycle_Depot().getRoutine());
        // autoSelector.addRoutine("Half_Hoard_Cycle_Outpost", () -> new Half_Hoard_Cycle_Outpost().getRoutine());

        SmartDashboard.putData("Auto Path Selection", autoSelector);
    }

    public Command getSelectedCommand(){
        return autoSelector.selectedCommandScheduler();
    }

    public AutoChooser getAutoChooser(){
        return autoSelector;
    }
}
