package frc.robot.CatzSubsystems.CatzHood;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzHood extends ServoMotorSubsystem<HoodIO, HoodIO.HoodIOInputs>{

    private static final HoodIO io = getIOInstance();
    private static final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private static HoodIO getIOInstance() {
        if (CatzConstants.HoodOn == false) {
            System.out.println("Hood Disabled by CatzConstants");
            return new HoodIOSim();
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Hood Configured for Real");
                return new HoodIOTalonFX(HoodConstants.getIOConfig());
            case SIM:
                System.out.println("Hood Configured for Simulation");
                return new HoodIOSim();
                default:
                System.out.println("Hood Unconfigured");
                return new HoodIOSim();
        }
    }

    private CatzHood() {
        super(io, inputs, "CatzHood", HoodConstants.HOOD_THRESHOLD);

        setCurrentPosition(HoodConstants.HOOD_ZERO_POS);
    }

    public static final CatzHood Instance = new CatzHood();



}
