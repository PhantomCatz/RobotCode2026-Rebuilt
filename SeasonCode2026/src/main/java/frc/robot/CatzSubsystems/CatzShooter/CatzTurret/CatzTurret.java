package frc.robot.CatzSubsystems.CatzShooter.CatzTurret;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.CatzConstants;
import frc.robot.FieldConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.TurretIO.TurretIOInputs;
import frc.robot.Utilities.Setpoint;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;

public class CatzTurret extends ServoMotorSubsystem<TurretIO, TurretIO.TurretIOInputs> {

    /**
     * Angle history of the turret in radians
     */
    private final TimeInterpolatableBuffer<Double> angleHistory = TimeInterpolatableBuffer.createDoubleBuffer(1.5);

    private static final TurretIOInputs inputs = new TurretIOInputsAutoLogged();

    private static final TurretIO io = getIOInstance();

    public enum ShooterState {
        HOME,
        TRACKING,
        MANUAL;
    }

    public ShooterState state = ShooterState.HOME;

    private CatzTurret() {
        super(io, inputs, "CatzTurret", TurretConstants.TURRET_THRESHOLD);

        setCurrentPosition(Units.Rotations.of(getCANCoderAbsPos()));
        // setCurrentPosition(Units.Rotations.of(0.0));

    }

    public static final CatzTurret Instance = new CatzTurret();

    double p = 0.0;
    double d = 0.0;
    double s = 0.0;
    double v = 0.0;

    @Override
    public void periodic() {
        super.periodic();

        // if (TurretConstants.kP.get() != p || TurretConstants.kD.get() != d || TurretConstants.kS.get() != s
        //         || TurretConstants.kV.get() != v) {
        //     setPDSVGGains(TurretConstants.kP.get(), TurretConstants.kD.get(), TurretConstants.kS.get(),
        //             TurretConstants.kV.get(), 0.0);
        //     p = TurretConstants.kP.get();
        //     d = TurretConstants.kD.get();
        //     s = TurretConstants.kS.get();
        //     v = TurretConstants.kV.get();
        // }

        Pose2d turretPose = new Pose2d(CatzTurret.Instance.getFieldToTurret(),
                Rotation2d.fromRotations(CatzTurret.Instance.getLatencyCompensatedPosition())
                        .plus(CatzRobotTracker.Instance.getEstimatedPose().getRotation())
                        .plus(TurretConstants.TURRET_ROTATION_OFFSET));
        Logger.recordOutput("Shooter Location", turretPose);

        double distFromHub = FieldConstants.getHubLocation().getDistance(turretPose.getTranslation());
        Logger.recordOutput("Distance from Hub", distFromHub);
        // Logger.recordOutput("Distance from Close Corner", AimCalculations.getCornerHoardingTarget(true).getDistance(getFieldToTurret()));
        // Logger.recordOutput("Distance from Far Corner", AimCalculations.getCornerHoardingTarget(false).getDistance(getFieldToTurret()));

        Logger.recordOutput("CANCoder Absolute Position", getCANCoderAbsPos());

        angleHistory.addSample(Timer.getFPGATimestamp(), getLatencyCompensatedPosition() * 2 * Math.PI);
    }

    /**
     * Calculates the best turret angle setpoint to reach a target rotation
     * while respecting physical limits and minimizing movement
     */
    public Setpoint calculateWrappedSetpoint(Angle target) {
        double targetRads = target.in(Units.Radians) + Math.toRadians(-0.0); // TODO REMOVE

        targetRads = MathUtil.angleModulus(targetRads);

        // TODO remove after the turret is fixed
        targetRads = Math.min(targetRads, TurretConstants.TURRET_MAX.in(Units.Radian));
        targetRads = Math.max(targetRads, TurretConstants.TURRET_MIN.in(Units.Radian));

        return Setpoint.withMotionMagicSetpoint(Units.Radians.of(targetRads));
    }

    public Translation2d getFieldToTurret() {
        Pose2d fieldToRobot = CatzRobotTracker.Instance.getEstimatedPose();
        return fieldToRobot.getTranslation().plus(TurretConstants.TURRET_OFFSET.rotateBy(fieldToRobot.getRotation()));
    }

    public Translation2d getFieldToTurret(Pose2d predictedRobotPose){
        return predictedRobotPose.getTranslation().plus(TurretConstants.TURRET_OFFSET.rotateBy(predictedRobotPose.getRotation()));
    }

    public double getAngleAtTime(double time) {
        return angleHistory.getSample(time).orElse(getLatencyCompensatedPosition() * 2 * Math.PI);
    }

    /**
     *
     * @return The absolute position of the CANCoder accounting for the gear ratio.
     * Note that the CANCoder's range is only [-1,1] rotations, and so the absolute encoder range of the turret is only
     * applicable for +-1 / gear_ratio rotations. If the turret reads the absolute position outside of this range, then
     * it will not be truly "absolute".
     */
    public double getCANCoderAbsPos(){
        return TurretConstants.TURRET_CANCODER.getAbsolutePosition().getValueAsDouble() * TurretConstants.CANCODER_RATIO;
    }

    private static TurretIO getIOInstance() {
        if (CatzConstants.TurretOn == false) {
            System.out.println("Turret Disabled by CatzConstants");
            return new TurretIOSim(TurretConstants.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Turret Configured for Real");
                return new TurretIOTalonFX(TurretConstants.getIOConfig(), true);
            case SIM:
                System.out.println("Turret Configured for Simulation");
                return new TurretIOSim(TurretConstants.gains);
            default:
                System.out.println("Turret Unconfigured");
                return new TurretIOSim(TurretConstants.gains);
        }
    }
}
