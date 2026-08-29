package frc.robot.CatzSubsystems.CatzPivotArm;



import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzPivotArm extends ServoMotorSubsystem<PivotArmIO, PivotArmIO.PivotArmIOInputs>{

    private static final PivotArmIO io = getIOInstance();
    private static final PivotArmIOInputsAutoLogged inputs = new PivotArmIOInputsAutoLogged();

    public static final CatzPivotArm Instance = new CatzPivotArm();

    public enum IntakeState{
        ON,
        OFF,
    }

    private CatzPivotArm() {
        super(io, inputs, "CatzPivotArm", PivotArmConstants.DEPLOY_THRESHOLD);
        setCurrentPosition(PivotArmConstants.HOME_POSITION);
    }

    double prevP = 0.0;
    double prevV = 0.0;
    @Override
    public void periodic(){
        super.periodic();

        double newP = PivotArmConstants.kP.get();
        double newV = PivotArmConstants.kV.get();
        if(newP != prevP || newV != prevV){
            prevV = newV;
            prevP = newP;
            setGainsPV(newP, newV);
        }
    }

    private static PivotArmIO getIOInstance() {
        if (CatzConstants.IntakeOn == false) {
            System.out.println("Intake Deploy Disabled by CatzConstants");
            return new PivotArmIOSim(PivotArmConstants.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Intake Deploy Configured for Real");
                return new PivotArmIOTalonFX(PivotArmConstants.getIOConfig());
            case SIM:
                System.out.println("Intake Deploy Configured for Simulation");
                return new PivotArmIOSim(PivotArmConstants.gains);
                default:
                System.out.println("Intake Deploy Unconfigured");
                return new PivotArmIOSim(PivotArmConstants.gains);
        }
    }
}
