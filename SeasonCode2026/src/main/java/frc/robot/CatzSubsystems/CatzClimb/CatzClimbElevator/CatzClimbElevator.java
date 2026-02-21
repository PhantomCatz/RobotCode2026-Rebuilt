package frc.robot.CatzSubsystems.CatzClimb.CatzClimbElevator;


import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzClimbElevator extends ServoMotorSubsystem<ClimbIOElevator, ClimbIOElevator.ClimbTallIOInputs> {

    private static final ClimbIOElevator io = getIOInstance();
    private static final ClimbElevatorIOInputsAutoLogged inputs = new ClimbElevatorIOInputsAutoLogged();

    private static ClimbIOElevator getIOInstance() {
        if (CatzConstants.ClimbOn == false) {
            System.out.println("Climb Disabled by CatzConstants");
            return new ClimbIOTallElevator(ClimbConstantsElevator.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Climb Configured for Real");
                return new ClimbIOTalonFXElevator(ClimbConstantsElevator.getIOConfig());
            case SIM:
                System.out.println("Climb Configured for Simulation");
                return new ClimbIOTallElevator(ClimbConstantsElevator.gains);
                default:
                System.out.println("Climb Unconfigured");
                return new ClimbIOTallElevator(ClimbConstantsElevator.gains);
        }
    }

    public static final CatzClimbElevator Instance = new CatzClimbElevator();

    private CatzClimbElevator() {
        super(io, inputs, "CatzClimb", ClimbConstantsElevator.converter.toAngle(ClimbConstantsElevator.CLIMB_THRESHOLD));
    }

}
