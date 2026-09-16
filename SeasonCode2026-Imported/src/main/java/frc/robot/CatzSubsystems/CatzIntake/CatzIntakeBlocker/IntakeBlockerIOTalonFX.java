package frc.robot.CatzSubsystems.CatzIntake.CatzIntakeBlocker;


import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class IntakeBlockerIOTalonFX extends GenericTalonFXIOReal<IntakeBlockerIO.IntakeBlockerIOInputs> implements IntakeBlockerIO{
    public IntakeBlockerIOTalonFX(MotorIOTalonFXConfig config){
        super(config, true);
    }

    @Override
    public void setMotionMagicSetpoint(double target){
        double feedforward;
        if(CatzIntakeBlocker.Instance.getLatencyCompensatedPosition() > 0.28){
            feedforward = 0.0;
        }else{
            feedforward = -IntakeBlockerConstants.GRAVITY_FEEDFORWARD* Math.sin(CatzIntakeBlocker.Instance.getLatencyCompensatedPosition() * 2 * Math.PI);
        }
        // Logger.recordOutput("Intake Blocker Setpoint", target);
        setControl(new MotionMagicVoltage(target).withFeedForward(feedforward));
    }

    @Override
    public void setVoltageSetpoint(double target){
        double feedforward = -IntakeBlockerConstants.GRAVITY_FEEDFORWARD * Math.sin(CatzIntakeBlocker.Instance.getLatencyCompensatedPosition() * 2 * Math.PI);
        setControl(new VoltageOut(target + feedforward));
    }
}
