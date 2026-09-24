package frc.robot.CatzSubsystems.MantaPivot;


import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class MantaPivotIOTalonFX extends GenericTalonFXIOReal<MantaPivotIO.MantaPivotIOInputs> implements MantaPivotIO{
    public MantaPivotIOTalonFX(MotorIOTalonFXConfig config){
        super(config, true);
    }

    @Override
    public void setMotionMagicSetpoint(double target){
        double feedforward;
        if(MantaPivot.Instance.getLatencyCompensatedPosition() > 0.28){
            feedforward = 0.0;
        }else{
            feedforward = -MantaPivotConstants.GRAVITY_FEEDFORWARD* Math.sin(MantaPivot.Instance.getLatencyCompensatedPosition() * 2 * Math.PI);
        }
        // Logger.recordOutput("Intake Deploy Setpoint", target);
        setControl(new MotionMagicVoltage(target).withFeedForward(feedforward));
    }

    @Override
    public void setVoltageSetpoint(double target){
        double feedforward = -MantaPivotConstants.GRAVITY_FEEDFORWARD * Math.sin(MantaPivot.Instance.getLatencyCompensatedPosition() * 2 * Math.PI);
        setControl(new VoltageOut(target + feedforward));
    }
}
