package frc.robot.CatzSubsystems.CatzClimb.CatzClimbClaw;

import frc.robot.CatzAbstractions.io.GenericSparkmaxIOReal;

public class ClimbIOSparkmaxClaw extends GenericSparkmaxIOReal<ClimbIOClaw.ClimbClawIOInputs> implements ClimbIOClaw{
    public ClimbIOSparkmaxClaw( MotorIOSparkMaxConfig config){
        super(config);
    }
}
