package frc.robot.CatzSubsystems.CatzTurret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;

public class TurretIOTalonFX extends GenericTalonFXIOReal<TurretIO.TurretIOInputs> implements TurretIO{

    public TurretIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }

    private final double TO_ROT = 1.0 / (2*Math.PI);
    @Override
    public void setMotionMagicSetpoint(double targetRot){
        double robotAngularVelocity = CatzRobotTracker.Instance.getRobotChassisSpeeds().omegaRadiansPerSecond * TO_ROT;
        double feedforward = TurretConstants.ROBOT_OMEGA_FEEDFORWARD * robotAngularVelocity;

        leaderTalon.setControl(new MotionMagicVoltage(targetRot).withFeedForward(feedforward));
    }
}
