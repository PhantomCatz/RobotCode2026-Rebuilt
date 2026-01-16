package frc.robot.CatzSubsystems.CatzIntake;

import org.littletonrobotics.junction.Logger;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.GenericMotorSubsystem;

public class CatzIntake extends GenericMotorSubsystem<IntakeIO, IntakeIO.IntakeIOInputs>{

    private static final IntakeIO io = getIOInstance();
    private static final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public static final CatzIntake Instance = new CatzIntake();

    private CatzIntake() {
        super(io, inputs, "CatzIntake");
    }

    static IntakeIO getIOInstance() {
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Intake Configured for Real");
                return new IntakeIOTalonFX(IntakeConstants.getIOConfig());
            case SIM:
                System.out.println("Intake Configured for Simulation");
                return new IntakeIOSim();
                default:
                System.out.println("Intake Unconfigured");
                return new IntakeIOSim();
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        Logger.processInputs(name, inputs);
    }
}
