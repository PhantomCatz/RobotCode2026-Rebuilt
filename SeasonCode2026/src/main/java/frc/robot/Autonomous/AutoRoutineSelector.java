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
