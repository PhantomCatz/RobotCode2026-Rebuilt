package frc.robot.CatzSubsystems.CatzIntakeRoller;


import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.GenericMotorSubsystem;
import frc.robot.Utilities.Setpoint;

public class CatzIntakeRoller extends GenericMotorSubsystem<IntakeRollerIO, IntakeRollerIO.IntakeRollerIOInputs>{

    private static final IntakeRollerIO io = getIOInstance();
    private static final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

    public static final CatzIntakeRoller Instance = new CatzIntakeRoller();

    public IntakeState state = IntakeState.OFF;

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
            System.out.println("Intake Roller Disabled by CatzConstants");
            return new IntakeRollerIOSim();
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Intake Roller Configured for Real");
                return new IntakeRollerIOTalonFX(IntakeRollerConstants.getIOConfig());
            case SIM:
                System.out.println("Intake Roller Configured for Simulation");
                return new IntakeRollerIOSim();
                default:
                System.out.println("Intake Roller Unconfigured");
                return new IntakeRollerIOSim();
        }
    }

    public Setpoint toggleIntake(){
        if(state == IntakeState.OFF){
            state = IntakeState.ON;
            return IntakeRollerConstants.ON_SETPOINT;
        }else{
            state = IntakeState.OFF;
            return IntakeRollerConstants.OFF_SETPOINT;
        }
    }
}
