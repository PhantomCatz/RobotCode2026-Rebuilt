package frc.robot.CatzSubsystems.CatzClimb;

import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;

public class CatzClimb extends ServoMotorSubsystem {

    public static final CatzClimb Instance = new CatzClimb();

    private CatzClimb() {
        super(getIOInstance(ClimbConstants.getIOConfig()), "CatzClimb", ClimbConstants.converter.toAngle(ClimbConstants.CLIMB_THRESHOLD));
    }
}
