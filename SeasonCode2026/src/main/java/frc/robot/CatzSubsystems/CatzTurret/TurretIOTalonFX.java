package frc.robot.CatzSubsystems.CatzTurret;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.CatzAbstractions.io.GenericTalonFXIOReal;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;

public class TurretIOTalonFX extends GenericTalonFXIOReal<TurretIO.TurretIOInputs> implements TurretIO{
    private PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    public TurretIOTalonFX(MotorIOTalonFXConfig config){
        super(config);
    }

    @Override
    public void setPositionSetpoint(double targetRot){
        double targetRads = targetRot * 2*Math.PI;
        double robotAngularVelocity = CatzRobotTracker.Instance.getRobotChassisSpeeds().omegaRadiansPerSecond;
        double feedforward = TurretConstants.kV.get() * robotAngularVelocity;

        leaderTalon.setControl(
            positionTorqueCurrentRequest.withPosition(Angle.ofBaseUnits(targetRads, Units.Radians))
                                        .withFeedForward(feedforward)
        );
    }
}
