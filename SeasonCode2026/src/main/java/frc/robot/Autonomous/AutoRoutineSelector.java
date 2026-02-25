package frc.robot.Autonomous;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Autonomous.routines.Depot_Climb;
import frc.robot.Autonomous.routines.Forefit_Depot;
import frc.robot.Autonomous.routines.Forefit_Outpost;
import frc.robot.Autonomous.routines.Outpost_Climb;
import frc.robot.Autonomous.routines.PNZO;
import frc.robot.Autonomous.routines.R1_IAS;
import frc.robot.Autonomous.routines.R2_IAS;
import frc.robot.Autonomous.routines.R3_IAS;
import frc.robot.Autonomous.routines.Test;

public class AutoRoutineSelector {
    public static final AutoRoutineSelector Instance = new AutoRoutineSelector();

    private AutoChooser autoSelector = new AutoChooser();

    private AutoRoutineSelector(){
        autoSelector.addRoutine("Test", () -> new Test().getRoutine());

        autoSelector.addRoutine("R3_IAS", () -> new R3_IAS().getRoutine());
        autoSelector.addRoutine("R2_IAS", () -> new R2_IAS().getRoutine());
        autoSelector.addRoutine("R1_IAS", () -> new R1_IAS().getRoutine());
        autoSelector.addRoutine("PNZO", () -> new PNZO().getRoutine());
        autoSelector.addRoutine("Outpost_Climb", () -> new Outpost_Climb().getRoutine());
        autoSelector.addRoutine("Forefit_Outpost", () -> new Forefit_Outpost().getRoutine());
        autoSelector.addRoutine("Forefit_Depot", () -> new Forefit_Depot().getRoutine());
        autoSelector.addRoutine("Depot_Climb", () -> new Depot_Climb().getRoutine());

        SmartDashboard.putData("Auto Path Selection", autoSelector);
    }

    public Command getSelectedCommand(){
        return autoSelector.selectedCommandScheduler();
    }

    public AutoChooser getAutoChooser(){
        return autoSelector;
    }
}
