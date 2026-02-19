package frc.robot.CatzSubsystems.CatzClimb.CatzClimbShort;


import org.littletonrobotics.junction.Logger;

import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzClimbShort extends ServoMotorSubsystem<ClimbIO, ClimbIO.ClimbIOInputs> {

    private static final ClimbIO io = getIOInstance();
    private static final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private static ClimbIO getIOInstance() {
        if (CatzConstants.ClimbOn == false) {
            System.out.println("Climb Disabled by CatzConstants");
            return new ClimbIOSim(ClimbConstantsShort.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Climb Configured for Real");
                return new ClimbIOTalonFX(ClimbConstantsShort.getIOConfig());
            case SIM:
                System.out.println("Climb Configured for Simulation");
                return new ClimbIOSim(ClimbConstantsShort.gains);
                default:
                System.out.println("Climb Unconfigured");
                return new ClimbIOSim(ClimbConstantsShort.gains);
        }
    }

    public static final CatzClimbShort Instance = new CatzClimbShort();

    private CatzClimbShort() {
        super(io, inputs, "CatzClimb", ClimbConstantsShort.converter.toAngle(ClimbConstantsShort.CLIMB_THRESHOLD));
    }

    @Override
    public void periodic() {
        super.periodic();

        Logger.recordOutput("Climb/TargetPosition", setpoint.baseUnits);
    }
}
