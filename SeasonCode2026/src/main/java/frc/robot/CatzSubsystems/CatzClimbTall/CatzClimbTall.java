package frc.robot.CatzSubsystems.CatzClimbTall;


import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzClimbTall extends ServoMotorSubsystem<ClimbIOTall, ClimbIOTall.ClimbTallIOInputs> {

    private static final ClimbIOTall io = getIOInstance();
    private static final ClimbTallIOInputsAutoLogged inputs = new ClimbTallIOInputsAutoLogged();

    private static ClimbIOTall getIOInstance() {
        if (CatzConstants.ClimbOn == false) {
            System.out.println("Climb Disabled by CatzConstants");
            return new ClimbIOTallSim(ClimbConstantsTall.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Climb Configured for Real");
                return new ClimbIOTalonFXTall(ClimbConstantsTall.getIOConfig());
            case SIM:
                System.out.println("Climb Configured for Simulation");
                return new ClimbIOTallSim(ClimbConstantsTall.gains);
                default:
                System.out.println("Climb Unconfigured");
                return new ClimbIOTallSim(ClimbConstantsTall.gains);
        }
    }

    public static final CatzClimbTall Instance = new CatzClimbTall();

    private CatzClimbTall() {
        super(io, inputs, "CatzClimb", ClimbConstantsTall.converter.toAngle(ClimbConstantsTall.CLIMB_THRESHOLD));
    }

}
