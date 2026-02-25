package frc.robot.CatzSubsystems.CatzShooter.CatzHood;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class HoodIOTalonFX extends GenericTalonFXIOReal<HoodIO.HoodIOInputs> implements HoodIO{
    public HoodIOTalonFX(MotorIOTalonFXConfig config, boolean requiresFastUpdate){
        super(config, requiresFastUpdate);
    }

    @Override
    public void setMotionMagicSetpoint(double target) {
        double feedforward = HoodConstants.hoodGravityFF.get() * Math.cos(CatzHood.Instance.getLatencyCompensatedPosition() * 2 * Math.PI - Math.toRadians(HoodConstants.hoodPhaseShift.get()));
        setControl(new MotionMagicVoltage(target).withFeedForward(feedforward));
    }
}
