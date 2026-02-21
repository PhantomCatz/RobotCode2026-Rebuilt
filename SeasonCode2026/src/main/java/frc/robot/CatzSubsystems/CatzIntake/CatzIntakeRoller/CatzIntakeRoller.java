package frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller;


import edu.wpi.first.units.measure.Angle;
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
            System.out.println("Intake Disabled by CatzConstants");
            return new IntakeRollerIOSim(IntakeRollerConstants.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Intake Roller Configured for Real");
                return new IntakeRollerIOTalonFX(IntakeRollerConstants.getIOConfig());
            case SIM:
                System.out.println("Intake Configured for Simulation");
                return new IntakeRollerIOSim(IntakeRollerConstants.gains);
                default:
                System.out.println("Intake Unconfigured");
                return new IntakeRollerIOSim(IntakeRollerConstants.gains);
        }
    }

    public Setpoint toggleIntake(){
        if(state == IntakeState.OFF){
            state = IntakeState.ON;
            return IntakeRollerConstants.H_SETPOINT;
        }else{
            state = IntakeState.OFF;
            return IntakeRollerConstants.OFF_SETPOINT;
        }
    }

    public void setCurrentPosition(Angle of) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCurrentPosition'");
    }
}
