package frc.robot.CatzSubsystems.CatzIntake.CatzIntakeDeploy;



import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzIntakeDeploy extends ServoMotorSubsystem<IntakeDeployIO, IntakeDeployIO.IntakeDeployIOInputs>{

    private static final IntakeDeployIO io = getIOInstance();
    private static final IntakeDeployIOInputsAutoLogged inputs = new IntakeDeployIOInputsAutoLogged();

    public static final CatzIntakeDeploy Instance = new CatzIntakeDeploy();

    public enum IntakeState{
        ON,
        OFF,
        AUTO; //object detection mode
    }

    private CatzIntakeDeploy() {
        super(io, inputs, "CatzIntakeDeploy", IntakeDeployConstants.DEPLOY_THRESHOLD);
        setCurrentPosition(IntakeDeployConstants.HOME_POSITION);
    }

    double p = 0.0;
    double d = 0.0;
    double s = 0.0;
    double v = 0.0;
    @Override
    public void periodic(){
        super.periodic();

        // if(IntakeDeployConstants.kP.get() != p || IntakeDeployConstants.kD.get() != d || IntakeDeployConstants.kS.get() != s || IntakeDeployConstants.kV.get() != v){
        //     setPDSVGGains(IntakeDeployConstants.kP.get(), IntakeDeployConstants.kD.get(), IntakeDeployConstants.kS.get(), IntakeDeployConstants.kV.get(), 0.0);
        //     p = IntakeDeployConstants.kP.get();
        //     d = IntakeDeployConstants.kD.get();
        //     s = IntakeDeployConstants.kS.get();
        //     v = IntakeDeployConstants.kV.get();
        // }
    }

    private static IntakeDeployIO getIOInstance() {
        if (CatzConstants.IntakeOn == false) {
            System.out.println("Intake Deploy Disabled by CatzConstants");
            return new IntakeDeployIOSim(IntakeDeployConstants.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Intake Deploy Configured for Real");
                return new IntakeDeployIOTalonFX(IntakeDeployConstants.getIOConfig());
            case SIM:
                System.out.println("Intake Deploy Configured for Simulation");
                return new IntakeDeployIOSim(IntakeDeployConstants.gains);
                default:
                System.out.println("Intake Deploy Unconfigured");
                return new IntakeDeployIOSim(IntakeDeployConstants.gains);
        }
    }
}
