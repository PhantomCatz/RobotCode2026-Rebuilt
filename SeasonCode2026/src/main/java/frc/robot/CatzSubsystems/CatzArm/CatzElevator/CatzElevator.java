package frc.robot.CatzSubsystems.CatzArm.CatzElevator;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzElevator extends ServoMotorSubsystem<ElevatorIO, ElevatorIO.ElevatorIOInputs>{

    private static final ElevatorIO io = getIOInstance();
    private static final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public static final CatzElevator Instance = new CatzElevator();

    public enum ArmState {
        STOW,
        HOLD,
        L1,
        L2,
        L3,
        L4,
        ALGAE
    }

    private CatzElevator() {
        super(io, inputs, "CatzElevator", ElevatorConstants.ELEVATOR_THRESHOLD);
        setCurrentPosition(ElevatorConstants.STOW_POSITION);
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
        double newP = ElevatorConstants.kP.get();
        double newI = ElevatorConstants.kI.get();
        double newD = ElevatorConstants.kD.get();
        double newS = ElevatorConstants.kS.get();
        double newV = ElevatorConstants.kV.get();
        double newA = ElevatorConstants.kA.get();
        double newG = ElevatorConstants.kG.get();

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

    private static ElevatorIO getIOInstance() {
        if (CatzConstants.ElevatorOn == false) {
            System.out.println("Elevator Disabled by CatzConstants");
            return new ElevatorIOSim(ElevatorConstants.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Elevator Configured for Real");
                return new ElevatorIOTalonFX(ElevatorConstants.getIOConfig());
            case SIM:
                System.out.println("Elevator Configured for Simulation");
                return new ElevatorIOSim(ElevatorConstants.gains);
                default:
                System.out.println("Elevator Unconfigured");
                return new ElevatorIOSim(ElevatorConstants.gains);
        }
    }
}
