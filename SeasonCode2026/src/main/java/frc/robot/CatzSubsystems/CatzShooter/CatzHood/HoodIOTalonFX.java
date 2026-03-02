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
            System.out.println("HOOD FF DISABLED!!!!");
        }else{
            feedforward = HoodConstants.hoodGravityFF.get() * Math.cos(CatzHood.Instance.getLatencyCompensatedPosition() * 2 * Math.PI - Math.toRadians(HoodConstants.hoodPhaseShift.get()));
        }
        setControl(new MotionMagicVoltage(target).withFeedForward(feedforward));
    }
}
