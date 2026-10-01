package frc.robot.CatzSubsystems.CatzIntake.CatzMantaArm;


import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class MantaArmIOTalonFX extends GenericTalonFXIOReal<MantaArmIO.MantaArmIOInputs> implements MantaArmIO{
    public MantaArmIOTalonFX(MotorIOTalonFXConfig config){
        super(config, true);
    }

    @Override
    public void setMotionMagicSetpoint(double target){
        double feedforward;
        if(MantaArm.Instance.getLatencyCompensatedPosition() > 0.28){
            feedforward = 0.0;
        }else{
            feedforward = -MantaArmConstants.GRAVITY_FEEDFORWARD* Math.sin(MantaArm.Instance.getLatencyCompensatedPosition() * 2 * Math.PI);
        }
        // Logger.recordOutput("Intake Deploy Setpoint", target);
        setControl(new MotionMagicVoltage(target).withFeedForward(feedforward));
    }

    @Override
    public void setVoltageSetpoint(double target){
        double feedforward = -MantaArmConstants.GRAVITY_FEEDFORWARD * Math.sin(CatzMantaArm.Instance.getLatencyCompensatedPosition() * 2 * Math.PI);
        setControl(new VoltageOut(target + feedforward));
    }
}
