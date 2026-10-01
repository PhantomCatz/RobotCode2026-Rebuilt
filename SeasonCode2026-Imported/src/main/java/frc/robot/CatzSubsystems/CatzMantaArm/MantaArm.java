package frc.robot.CatzSubsystems.CatzMantaArm;



import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class MantaArm extends ServoMotorSubsystem<MantaArmIO, MantaArmIO.MantaArmIOInputs>{

    private static final MantaArmIO io = getIOInstance();
    private static final MantaArmIOInputsAutoLogged inputs = new MantaArmIOInputsAutoLogged();

    public static final MantaArm Instance = new MantaArm();

    public enum IntakeState{
        ON,
        OFF,
        AUTO; //object detection mode
    }

    private MantaArm() {
        super(io, inputs, "MantaArm", MantaArmConstants.DEPLOY_THRESHOLD);
        setCurrentPosition(MantaArmConstants.HOME_POSITION);
    }

    double prevP = 0.0;
    double prevV = 0.0;
    @Override
    public void periodic(){
        super.periodic();

        double newP = MantaArmConstants.kP.get();
        double newV = MantaArmConstants.kV.get();
        if(newP != prevP || newV != prevV){
            prevV = newV;
            prevP = newP;
            setGainsPV(newP, newV);
        }
    }

    private static MantaArmIO getIOInstance() {
        if (CatzConstants.IntakeOn == false) {
            System.out.println("Intake Deploy Disabled by CatzConstants");
            return new MantaArmIOSim(MantaArmConstants.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Intake Deploy Configured for Real");
                return new MantaArmIOTalonFX(MantaArmConstants.getIOConfig());
            case SIM:
                System.out.println("Intake Deploy Configured for Simulation");
                return new MantaArmIOSim(MantaArmConstants.gains);
                default:
                System.out.println("Intake Deploy Unconfigured");
                return new MantaArmIOSim(MantaArmConstants.gains);
        }
    }
}
