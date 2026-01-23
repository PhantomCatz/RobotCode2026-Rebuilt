package frc.robot.CatzSubsystems.CatzIntakeDeploy;


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

    private static IntakeDeployIO getIOInstance() {
        if (CatzConstants.IntakeOn == false) {
            System.out.println("Intake Deploy Disabled by CatzConstants");
            return new IntakeDeployIOSim();
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Intake Deploy Configured for Real");
                return new IntakeDeployIOTalonFX(IntakeDeployConstants.getIOConfig());
            case SIM:
                System.out.println("Intake Deploy Configured for Simulation");
                return new IntakeDeployIOSim();
                default:
                System.out.println("Intake Deploy Unconfigured");
                return new IntakeDeployIOSim();
        }
    }
}
