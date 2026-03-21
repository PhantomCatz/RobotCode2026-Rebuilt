package frc.robot.Autonomous;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Autonomous.routines.*;

public class AutoRoutineSelector {
    public static final AutoRoutineSelector Instance = new AutoRoutineSelector();

    private AutoChooser autoSelector = new AutoChooser();

    private AutoRoutineSelector(){
        autoSelector.addRoutine("Test", () -> new Test().getRoutine());

        autoSelector.addRoutine("Decon_Depot_1_Cycle", () -> new Decon_Depot_1_Cycle().getRoutine());
        autoSelector.addRoutine("Decon_Depot_2_Cycle", () -> new Decon_Depot_2_Cycle().getRoutine());
        // autoSelector.addRoutine("Decon_Outpost_2_Cycle_Climb", () -> new Decon_Outpost_2_Cycle_Climb().getRoutine());
        autoSelector.addRoutine("Decon_Outpost_2_Cycle_Outpost", () -> new Decon_Outpost_2_Cycle_Outpost().getRoutine());
        // autoSelector.addRoutine("Decon_Outpost_Climb", () -> new Decon_Outpost_Climb().getRoutine());
        autoSelector.addRoutine("Decon_PNZO", () -> new Decon_PNZO().getRoutine());
        autoSelector.addRoutine("Depot_1_Cycle", () -> new Depot_1_Cycle().getRoutine());
        autoSelector.addRoutine("Depot_2_Cycle", () -> new Depot_2_Cycle().getRoutine());
        autoSelector.addRoutine("Depot_3_Cycle", () -> new Depot_3_Cycle().getRoutine());
        autoSelector.addRoutine("Forefit_Depot", () -> new Forefit_Depot().getRoutine());
        autoSelector.addRoutine("Forefit_Outpost", () -> new Forefit_Outpost().getRoutine());
        autoSelector.addRoutine("MiddlePath", () -> new MiddlePath().getRoutine());
        autoSelector.addRoutine("Outpost_1_Cycle_Outpost", () -> new Outpost_1_Cycle_Outpost().getRoutine());
        autoSelector.addRoutine("Outpost_1_Cycle", () -> new Outpost_1_Cycle().getRoutine());
        autoSelector.addRoutine("Outpost_2_Cycle_Outpost", () -> new Outpost_2_Cycle_Outpost().getRoutine());
        autoSelector.addRoutine("Outpost_2_Cycle", () -> new Outpost_2_Cycle().getRoutine());
        autoSelector.addRoutine("Outpost_3_Cycle", () -> new Outpost_3_Cycle().getRoutine());
        autoSelector.addRoutine("PNZO", () -> new PNZO().getRoutine());




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
