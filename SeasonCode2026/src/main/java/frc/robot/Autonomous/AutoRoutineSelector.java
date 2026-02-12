package frc.robot.Autonomous;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Autonomous.routines.R2IAS;
import frc.robot.Autonomous.routines.Test;
import frc.robot.Autonomous.routines.Testy;

public class AutoRoutineSelector {
    public static final AutoRoutineSelector Instance = new AutoRoutineSelector();

    private AutoChooser autoSelector = new AutoChooser();

    private AutoRoutineSelector(){
        autoSelector.addRoutine("R2IAS", () -> new R2IAS().getRoutine());
        autoSelector.addRoutine("Test", () -> new Test().getRoutine());
        autoSelector.addRoutine("Testy", () -> new Testy().getRoutine());

        SmartDashboard.putData("Auto Path Selection", autoSelector);
    }

    public Command getSelectedCommand(){
        return autoSelector.selectedCommandScheduler();
    }

    public AutoChooser getAutoChooser(){
        return autoSelector;
    }
}
