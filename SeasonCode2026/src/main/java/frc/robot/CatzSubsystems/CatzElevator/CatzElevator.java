package frc.robot.CatzSubsystems.CatzElevator;

import edu.wpi.first.units.Units;
import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzElevator extends ServoMotorSubsystem<ElevatorIO, ElevatorIO.ElevatorIOInputs> {
    private static final ElevatorIO io = getIOInstance();
    private static final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputAutoLogged();

    private static ElevatorIO getIOInstance() {
        if (CatzConstants.ElevatorOn == false) {
            System.out.println("Elevator Disabled by CatzConstants");
            return new ElevatorIOSim(ElevatorConstants.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("ELevator Configuted for REAL");
                return new ElevatorIOTalonFX(ElevatorConstants.getIOConfig());
            case SIM:
                System.out.println("Elevator Configured for Simulation");
                return new ElevatorIOSim(ElevatorConstants.gains);
            default:
                System.out.println("Elevator Unconfigured");
                return new ElevatorIOSim(ElevatorConstants.gains);
        }
    }
    
    public static final CatzElevator Instance = new CatzElevator();
    
    private CatzElevator() {
        super(io, inputs, "CatzElevator", ElevatorConstants.converter.toAngle(ElevatorConstants.CLIMB_THRESHOLD));
        
        setCurrentPosition(Units.Rotations.of(0.0));
    }

}
