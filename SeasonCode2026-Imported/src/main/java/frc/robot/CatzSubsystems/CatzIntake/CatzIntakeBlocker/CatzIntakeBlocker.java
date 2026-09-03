package frc.robot.CatzSubsystems.CatzIntake.CatzIntakeBlocker;



import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzIntakeBlocker extends ServoMotorSubsystem<IntakeBlockerIO, IntakeBlockerIO.IntakeBlockerIOInputs>{

    private static final IntakeBlockerIO io = getIOInstance();
    private static final IntakeBlockerIOInputsAutoLogged inputs = new IntakeBlockerIOInputsAutoLogged();

    public static final CatzIntakeBlocker Instance = new CatzIntakeBlocker();

    public enum IntakeState{
        ON,
        OFF;
    }

    private CatzIntakeBlocker() {
        super(io, inputs, "CatzIntakeBlocker", IntakeBlockerConstants.BLOCKER_THRESHOLD);
        setCurrentPosition(IntakeBlockerConstants.HOME_POSITION);
    }

    double prevP = 0.0;
    double prevV = 0.0;
    @Override
    public void periodic(){
        super.periodic();

        double newP = IntakeBlockerConstants.kP.get();
        double newV = IntakeBlockerConstants.kV.get();
        if(newP != prevP || newV != prevV){
            prevV = newV;
            prevP = newP;
            setGainsPV(newP, newV);
        }
    }

    private static IntakeBlockerIO getIOInstance() {
        if (CatzConstants.BlockerOn == false) {
            System.out.println("Intake Blocker Disabled by CatzConstants");
            return new IntakeBlockerIOSim(IntakeBlockerConstants.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Intake Blocker Configured for Real");
                return new IntakeBlockerIOTalonFX(IntakeBlockerConstants.getIOConfig());
            case SIM:
                System.out.println("Intake Blocker Configured for Simulation");
                return new IntakeBlockerIOSim(IntakeBlockerConstants.gains);
                default:
                System.out.println("Intake Blocker Unconfigured");
                return new IntakeBlockerIOSim(IntakeBlockerConstants.gains);
        }
    }
}
