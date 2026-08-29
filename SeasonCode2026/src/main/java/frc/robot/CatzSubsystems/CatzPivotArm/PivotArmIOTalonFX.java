package frc.robot.CatzSubsystems.CatzPivotArm;


import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class PivotArmIOTalonFX extends GenericTalonFXIOReal<PivotArmIO.PivotArmIOInputs> implements PivotArmIO{
    public PivotArmIOTalonFX(MotorIOTalonFXConfig config){
        super(config, true);
    }

    @Override
    public void setMotionMagicSetpoint(double target){
        double feedforward;
        if(CatzPivotArm.Instance.getLatencyCompensatedPosition() > 0.28){
            feedforward = 0.0;
        }else{
            feedforward = -PivotArmConstants.GRAVITY_FEEDFORWARD* Math.sin(CatzPivotArm.Instance.getLatencyCompensatedPosition() * 2 * Math.PI);
        }
        // Logger.recordOutput("Intake Deploy Setpoint", target);
        setControl(new MotionMagicVoltage(target).withFeedForward(feedforward));
    }

    @Override
    public void setVoltageSetpoint(double target){
        double feedforward = -PivotArmConstants.GRAVITY_FEEDFORWARD * Math.sin(CatzPivotArm.Instance.getLatencyCompensatedPosition() * 2 * Math.PI);
        setControl(new VoltageOut(target + feedforward));
    }
}
