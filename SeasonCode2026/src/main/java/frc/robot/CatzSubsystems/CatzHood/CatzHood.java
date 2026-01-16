package frc.robot.CatzSubsystems.CatzHood;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzHood extends ServoMotorSubsystem<HoodIO, HoodIO.HoodIOInputs>{

    private static final HoodIO io = getIOInstance();
    private static final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public static HoodIO getIOInstance() {
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Indexer Configured for Real");
                return new HoodIOTalonFX(HoodConstants.getIOConfig());
            case SIM:
                System.out.println("Indexer Configured for Simulation");
                return new HoodIOSim();
                default:
                System.out.println("Indexer Unconfigured");
                return new HoodIOSim();
        }
    }

    private CatzHood() {
        super(io, inputs, "CatzHood", HoodConstants.HOOD_THRESHOLD);

        setCurrentPosition(HoodConstants.HOOD_ZERO_POS);
    }

    public static final CatzHood Instance = new CatzHood();

}
