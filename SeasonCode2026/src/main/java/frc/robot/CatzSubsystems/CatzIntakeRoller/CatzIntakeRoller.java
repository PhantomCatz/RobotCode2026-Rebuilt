package frc.robot.CatzSubsystems.CatzIntakeRoller;

import org.littletonrobotics.junction.Logger;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.GenericMotorSubsystem;

public class CatzIntakeRoller extends GenericMotorSubsystem<IntakeRollerIO, IntakeRollerIO.IntakeRollerIOInputs>{

    private static final IntakeRollerIO io = getIOInstance();
    private static final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

    public static final CatzIntakeRoller Instance = new CatzIntakeRoller();

    public enum IntakeState{
        ON,
        OFF,
        AUTO; //object detection mode
    }

    private CatzIntakeRoller() {
        super(io, inputs, "CatzIntakeRoller");
    }

    private static IntakeRollerIO getIOInstance() {
        if (CatzConstants.IntakeOn == false) {
            System.out.println("Intake Disabled by CatzConstants");
            return new IntakeRollerIOSim();
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Intake Configured for Real");
                return new IntakeRollerIOTalonFX(IntakeRollerConstants.getIOConfig());
            case SIM:
                System.out.println("Intake Configured for Simulation");
                return new IntakeRollerIOSim();
                default:
                System.out.println("Intake Unconfigured");
                return new IntakeRollerIOSim();
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        Logger.processInputs(name, inputs);
    }
}
