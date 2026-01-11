package frc.robot.CatzSubsystems.CatzIntake;

import frc.robot.CatzAbstractions.Bases.GenericMotorSubsystem;

public class CatzIntake extends GenericMotorSubsystem {

    public static final CatzIntake Instance = new CatzIntake();

    private CatzIntake() {
        super(getIOInstance(IntakeConstants.getIOConfig()), "CatzIntake");
    }
}
