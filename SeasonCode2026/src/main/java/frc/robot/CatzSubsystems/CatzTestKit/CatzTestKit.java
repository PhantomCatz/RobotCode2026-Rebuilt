package frc.robot.CatzSubsystems.CatzTestKit;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.GenericMotorSubsystem;
import frc.robot.Utilities.Setpoint;

public class CatzTestKit extends GenericMotorSubsystem<TestKitIO, TestKitIO.TestKitIOInputs>{

    private static final TestKitIO io = getIOInstance();
    private static final TestKitIOInputsAutoLogged inputs = new TestKitIOInputsAutoLogged();

    public static final CatzTestKit Instance = new CatzTestKit();

    public IntakeState state = IntakeState.OFF;

    public enum IntakeState{
        ON,
        OFF,
        AUTO; //object detection mode
    }

    private CatzTestKit() {
        super(io, inputs, "CatzTestKit");
    }

    private static TestKitIO getIOInstance() {
        if (CatzConstants.IntakeOn == false) {
            System.out.println("Intake Disabled by CatzConstants");
            return new TestKitIOSim(TestKitConstants.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Intake Roller Configured for Real");
                return new TestKitIOTalonFX(TestKitConstants.getIOConfig());
            case SIM:
                System.out.println("Intake Configured for Simulation");
                return new TestKitIOSim(TestKitConstants.gains);
                default:
                System.out.println("Intake Unconfigured");
                return new TestKitIOSim(TestKitConstants.gains);
        }
    }

    public Setpoint toggleIntake(){
        if(state == IntakeState.OFF){
            state = IntakeState.ON;
            return TestKitConstants.ON_SETPOINT;
        }else{
            state = IntakeState.OFF;
            return TestKitConstants.OFF_SETPOINT;
        }
    }
}
