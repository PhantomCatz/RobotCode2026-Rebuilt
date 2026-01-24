package frc.robot.Autonomous;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Autonomous.routines.*;

public class AutoRoutineSelector {
    public static final AutoRoutineSelector Instance = new AutoRoutineSelector();

    private AutoChooser autoSelector = new AutoChooser();

    private AutoRoutineSelector(){

        autoSelector.addRoutine("TestPath", () -> new TestPath().getRoutine());
        autoSelector.addRoutine("R3IAS", () -> new R3IAS().getRoutine());
        autoSelector.addRoutine("R2IAS", () -> new R2IAS().getRoutine());
        autoSelector.addRoutine("R1IAS", () -> new R1IAS().getRoutine());
        autoSelector.addRoutine("PNZO", () -> new PNZO().getRoutine());
        autoSelector.addRoutine("PNZD", () -> new PNZD().getRoutine());
        autoSelector.addRoutine("Outpostclimb", () -> new OutpostClimb().getRoutine());
        autoSelector.addRoutine("Forefit_Outpost", () -> new Forefit_Outpost().getRoutine());
        autoSelector.addRoutine("Forefit_Depot", () -> new Forefit_Depot().getRoutine());
        autoSelector.addRoutine("DepotClimb", () -> new DepotClimb().getRoutine());

        SmartDashboard.putData("Auto Path Selection", autoSelector);
    }

    public Command getSelectedCommand(){
        return autoSelector.selectedCommandScheduler();
    }

    public AutoChooser getAutoChooser(){
        return autoSelector;
    }
}
