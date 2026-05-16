package frc.robot.CatzSubsystems.CatzShooter.CatzHood;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.units.Units;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class HoodIOTalonFX extends GenericTalonFXIOReal<HoodIO.HoodIOInputs> implements HoodIO{
    public HoodIOTalonFX(MotorIOTalonFXConfig config, boolean requiresFastUpdate){
        super(config, requiresFastUpdate);
    }
    final double DISABLE_FF = Units.Degrees.of(15.0).in(Units.Rotations);
    @Override
    public void setMotionMagicSetpoint(double target) {
        double feedforward;
        if(target == HoodConstants.HOOD_ZERO_POS.in(Units.Rotations)){
            feedforward = 0.0;
        }else{
            feedforward = HoodConstants.HOOD_GRAVITY_FF * Math.cos(CatzHood.Instance.getLatencyCompensatedPosition() * 2 * Math.PI - HoodConstants.HOOD_GRAVITY_FF_PHASE_SHIFT);
        }
        setControl(new MotionMagicVoltage(target).withFeedForward(feedforward));
    }
}
