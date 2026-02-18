package frc.robot.CatzSubsystems.CatzShooter.CatzTurret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;

public class TurretIOTalonFX extends GenericTalonFXIOReal<TurretIO.TurretIOInputs> implements TurretIO{

    public TurretIOTalonFX(MotorIOTalonFXConfig config, boolean requiresFastUpdate){
        super(config, requiresFastUpdate);
    }

    private final double TO_ROT = 1.0 / (2*Math.PI);
    @Override
    public void setMotionMagicSetpoint(double targetRot){
        double curAngularVel = CatzRobotTracker.Instance.getRobotChassisSpeeds().omegaRadiansPerSecond;
        double velFeedforward = -TurretConstants.omegaFF.get() * curAngularVel;
        leaderTalon.setControl(new MotionMagicVoltage(targetRot).withFeedForward(velFeedforward));
    }
}
