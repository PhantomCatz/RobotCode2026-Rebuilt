package frc.robot.CatzSubsystems.CatzShooter.CatzHood;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzHood extends ServoMotorSubsystem<HoodIO, HoodIO.HoodIOInputs>{

    private static final HoodIO io = getIOInstance();
    private static final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private static HoodIO getIOInstance() {
        if (CatzConstants.HoodOn == false) {
            System.out.println("Hood Disabled by CatzConstants");
            return new HoodIOSim(HoodConstants.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Hood Configured for Real");
                return new HoodIOTalonFX(HoodConstants.getIOConfig(), true);
            case SIM:
                System.out.println("Hood Configured for Simulation");
                return new HoodIOSim(HoodConstants.gains);
                default:
                System.out.println("Hood Unconfigured");
                return new HoodIOSim(HoodConstants.gains);
        }
    }

    private CatzHood() {
        super(io, inputs, "CatzHood", HoodConstants.HOOD_THRESHOLD);

        setCurrentPosition(HoodConstants.HOOD_ZERO_POS);
    }

    double p = 0.0;
    double d = 0.0;
    double s = 0.0;
    double v = 0.0;
    double g = 0.0;
    @Override
    public void periodic(){
        super.periodic();

        if(HoodConstants.kP.get() != p || HoodConstants.kD.get() != d || HoodConstants.kS.get() != s || HoodConstants.kV.get() != v || HoodConstants.kG.get() != g){
            setPDSVGGains(HoodConstants.kP.get(), HoodConstants.kD.get(), HoodConstants.kS.get(), HoodConstants.kV.get(), HoodConstants.kG.get());
            p = HoodConstants.kP.get();
            d = HoodConstants.kD.get();
            s = HoodConstants.kS.get();
            v = HoodConstants.kV.get();
            g = HoodConstants.kG.get();
        }
    }

    public static final CatzHood Instance = new CatzHood();

    public double currentVelocity = inputs.velocityRPS;

    // public double getStatorCurrent() {
    //     return inputs.statorCurrentAmps;
    // }

    public edu.wpi.first.units.measure.AngularVelocity getVelocity() {
        return edu.wpi.first.units.Units.RotationsPerSecond.of(inputs.velocityRPS);
    }
}
