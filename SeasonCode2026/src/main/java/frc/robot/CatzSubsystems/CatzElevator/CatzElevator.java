package frc.robot.CatzSubsystems.CatzElevator;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzElevator extends ServoMotorSubsystem<ElevatorIO, ElevatorIO.ElevatorIOInputs>{

    private static final ElevatorIO io = getIOInstance();
    private static final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public static final CatzElevator Instance = new CatzElevator();

    private CatzElevator() {
        super(io, inputs, "CatzElevator", ElevatorConstants.ELEVATOR_THRESHOLD);
        setCurrentPosition(ElevatorConstants.STOW_POSITION);
    }

    double prevP = 0.0;
    double prevV = 0.0;
    @Override
    public void periodic(){
        super.periodic();

        double newP = ElevatorConstants.kP.get();
        double newV = ElevatorConstants.kV.get();
        if(newP != prevP || newV != prevV){
            prevV = newV;
            prevP = newP;
            setGainsPV(newP, newV);
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
