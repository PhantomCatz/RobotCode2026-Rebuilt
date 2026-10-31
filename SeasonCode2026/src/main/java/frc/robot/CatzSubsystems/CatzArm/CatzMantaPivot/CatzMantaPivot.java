package frc.robot.CatzSubsystems.CatzArm.CatzMantaPivot;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzMantaPivot extends ServoMotorSubsystem<MantaPivotIO, MantaPivotIO.MantaPivotIOInputs>{

    private static final MantaPivotIO io = getIOInstance();
    private static final MantaPivotIOInputsAutoLogged inputs = new MantaPivotIOInputsAutoLogged();

    public static final CatzMantaPivot Instance = new CatzMantaPivot();

    private CatzMantaPivot() {
        super(io, inputs, "CatzMantaPivot", MantaPivotConstants.DEPLOY_THRESHOLD);
        setCurrentPosition(MantaPivotConstants.STOW_POSITION);
    }

    double prevP = 0.0;
    double prevI = 0.0;
    double prevD = 0.0;
    double prevS = 0.0;
    double prevV = 0.0;
    double prevA = 0.0;
    double prevG = 0.0;

    @Override
    public void periodic(){
        super.periodic();

        // Update PID from LoggedTunableNumbers
        double newP = MantaPivotConstants.kP.get();
        double newI = MantaPivotConstants.kI.get();
        double newD = MantaPivotConstants.kD.get();
        double newS = MantaPivotConstants.kS.get();
        double newV = MantaPivotConstants.kV.get();
        double newA = MantaPivotConstants.kA.get();
        double newG = MantaPivotConstants.kG.get();

        if(newP != prevP || newI != prevI || newD != prevD || newS != prevS || newV != prevV || newA != prevA || newG != prevG){
            prevP = newP;
            prevI = newI;
            prevD = newD;
            prevS = newS;
            prevV = newV;
            prevA = newA;
            prevG = newG;
            setGainsPIDSVAG(newP, newI, newD, newS, newV, newA, newG);
        }
    }

    private static MantaPivotIO getIOInstance() {
        if (CatzConstants.MantaPivotOn == false) {
            System.out.println("Pivot Disabled by CatzConstants");
            return new MantaPivotIOSim(MantaPivotConstants.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Pivot Configured for Real");
                return new MantaPivotIOTalonFX(MantaPivotConstants.getIOConfig());
            case SIM:
                System.out.println("Pivot Configured for Simulation");
                return new MantaPivotIOSim(MantaPivotConstants.gains);
                default:
                System.out.println("Pivot Unconfigured");
                return new MantaPivotIOSim(MantaPivotConstants.gains);
        }
    }
}
