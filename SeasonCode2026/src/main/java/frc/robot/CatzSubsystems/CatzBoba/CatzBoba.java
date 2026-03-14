package frc.robot.CatzSubsystems.CatzBoba;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeDeploy.IntakeDeployConstants;

public class CatzBoba extends ServoMotorSubsystem<BobaIO, BobaIO.BobaIOInputs> {

    private static final BobaIO io = getIOInstance();
    private static final BobaIOInputsAutoLogged inputs = new BobaIOInputsAutoLogged();

    private static BobaIO getIOInstance() {
        if (CatzConstants.BobaOn == false) {
            System.out.println("Boba Disabled by CatzConstants");
            return new BobaIOSim(BobaConstants.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Boba Configured for Real");
                return new BobaIOTalonFX(BobaConstants.getIOConfig());
            case SIM:
                System.out.println("Boba Configured for Simulation");
                return new BobaIOSim(BobaConstants.gains);
                default:
                System.out.println("Boba Unconfigured");
                return new BobaIOSim(BobaConstants.gains);
        }
    }

    double p = 0.0;
    double d = 0.0;
    double s = 0.0;
    double v = 0.0;
    @Override
    public void periodic(){
        super.periodic();

        if(IntakeDeployConstants.kP.get() != p || IntakeDeployConstants.kD.get() != d || IntakeDeployConstants.kS.get() != s || IntakeDeployConstants.kV.get() != v){
            setPDSVGGains(IntakeDeployConstants.kP.get(), IntakeDeployConstants.kD.get(), IntakeDeployConstants.kS.get(), IntakeDeployConstants.kV.get(), 0.0);
            p = IntakeDeployConstants.kP.get();
            d = IntakeDeployConstants.kD.get();
            s = IntakeDeployConstants.kS.get();
            v = IntakeDeployConstants.kV.get();
        }
    }

    public static final CatzBoba Instance = new CatzBoba();

    private CatzBoba() {
        super(io, inputs, "CatzBoba", BobaConstants.BOBA_THRESHOLD);
    }



}
