package frc.robot.CatzSubsystems.CatzMantaPivot;



import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzMantaPivot extends ServoMotorSubsystem<MantaPivotIO, MantaPivotIO.MantaPivotIOInputs>{

    private static final MantaPivotIO io = getIOInstance();
    private static final MantaPivotIOInputsAutoLogged inputs = new MantaPivotIOInputsAutoLogged();

    public static final CatzMantaPivot Instance = new CatzMantaPivot();

    public enum IntakeState{
        ON,
        OFF,
        AUTO; //object detection mode
    }

    private CatzMantaPivot() {
        super(io, inputs, "CatzMantaPivot", MantaPivotConstants.DEPLOY_THRESHOLD);
        setCurrentPosition(MantaPivotConstants.HOME_POSITION);
    }

    double prevP = 0.0;
    double prevV = 0.0;
    @Override
    public void periodic(){
        super.periodic();

        double newP = MantaPivotConstants.kP.get();
        double newV = MantaPivotConstants.kV.get();
        if(newP != prevP || newV != prevV){
            prevV = newV;
            prevP = newP;
            setGainsPV(newP, newV);
        }
    }

    private static MantaPivotIO getIOInstance() {
        if (CatzConstants.IntakeOn == false) {
            System.out.println("Intake Deploy Disabled by CatzConstants");
            return new MantaPivotIOSim(MantaPivotConstants.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Intake Deploy Configured for Real");
                return new MantaPivotIOTalonFX(MantaPivotConstants.getIOConfig());
            case SIM:
                System.out.println("Intake Deploy Configured for Simulation");
                return new MantaPivotIOSim(MantaPivotConstants.gains);
                default:
                System.out.println("Intake Deploy Unconfigured");
                return new MantaPivotIOSim(MantaPivotConstants.gains);
        }
    }
}
