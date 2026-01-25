package frc.robot.CatzSubsystems.CatzIntakeDeploy;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;

public class IntakeDeployIOTalonFX extends GenericTalonFXIOReal<IntakeDeployIO.IntakeDeployIOInputs> implements IntakeDeployIO{
    public IntakeDeployIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }

    @Override
    public void setMotionMagicSetpoint(double target){
        double feedforward = -IntakeDeployConstants.kG.get() * Math.sin(CatzIntakeDeploy.Instance.getPosition() * 2 * Math.PI);

        System.out.println("ff: " + feedforward);
        setControl(new MotionMagicVoltage(target).withFeedForward(feedforward));
    }

    @Override
    public void setVoltageSetpoint(double target){
        double feedforward = -IntakeDeployConstants.GRAVITY_FEEDFORWARD * Math.sin(CatzIntakeDeploy.Instance.getPosition() * 2 * Math.PI);
        setControl(new VoltageOut(target + feedforward));
    }
}
