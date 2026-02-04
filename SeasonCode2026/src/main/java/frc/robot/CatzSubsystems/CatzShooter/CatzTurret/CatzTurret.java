package frc.robot.CatzSubsystems.CatzShooter.CatzTurret;


import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.CatzConstants;
import frc.robot.CatzAbstractions.Bases.ServoMotorSubsystem;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.TurretIO.TurretIOInputs;
import frc.robot.Utilities.Setpoint;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;

public class CatzTurret extends ServoMotorSubsystem<TurretIO, TurretIO.TurretIOInputs>{

    /**
     * Angle history of the turret in radians
     */
    private final TimeInterpolatableBuffer<Double> angleHistory = TimeInterpolatableBuffer.createDoubleBuffer(1.5);

    private static final TurretIOInputs inputs = new TurretIOInputsAutoLogged();

    private static final TurretIO io = getIOInstance();

    public enum ShooterState{
        HOME,
        TRACKING,
        MANUAL;
    }

    public ShooterState state = ShooterState.HOME;

    private CatzTurret(){
        super(io, inputs, "CatzTurret", TurretConstants.TURRET_THRESHOLD);

        MagnetSensorConfigs magConfig = new MagnetSensorConfigs();
        magConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        TurretConstants.TURRET_CANCODER.getConfigurator().apply(magConfig);

        double CAN_ABS_POS = TurretConstants.TURRET_CANCODER.getAbsolutePosition().getValueAsDouble() * TurretConstants.CANCODER_RATIO - TurretConstants.CANCODER_OFFSET;
        setCurrentPosition(Units.Rotations.of(CAN_ABS_POS));
    }

    public static final CatzTurret Instance = new CatzTurret();


    double p = 0.0;
    double d = 0.0;
    double s = 0.0;
    double v = 0.0;
    @Override
    public void periodic(){
        super.periodic();

        if(TurretConstants.kP.get() != p || TurretConstants.kD.get() != d || TurretConstants.kS.get() != s || TurretConstants.kV.get() != v){
            setPDSVGGains(TurretConstants.kP.get(), TurretConstants.kD.get(), TurretConstants.kS.get(), TurretConstants.kV.get(), 0.0);
            p = TurretConstants.kP.get();
            d = TurretConstants.kD.get();
            s = TurretConstants.kS.get();
            v = TurretConstants.kV.get();
        }

        Logger.recordOutput("Turret/ Commanded Setpoint", setpoint.baseUnits / (2*Math.PI));
        Logger.recordOutput("Turret/ CANCoder Absolute Rotations", TurretConstants.TURRET_CANCODER.getPosition().getValueAsDouble() * TurretConstants.CANCODER_RATIO);
        angleHistory.addSample(Timer.getFPGATimestamp(), getLatencyCompensatedPosition() * 2 * Math.PI);
    }

    /**
     * Calculates the best turret angle setpoint to reach a target rotation
     * while respecting physical limits and minimizing movement
     * Returns null when no valid setpoint can be found.
     */
    public Setpoint calculateWrappedSetpoint(Angle target){
        double targetRads = target.in(Units.Radians);

        targetRads = MathUtil.angleModulus(targetRads);

        return Setpoint.withMotionMagicSetpoint(Units.Radians.of(targetRads));
    }

    public Translation2d getFieldToTurret(){
        Pose2d fieldToRobot = CatzRobotTracker.Instance.getEstimatedPose();
        return fieldToRobot.getTranslation().plus(TurretConstants.TURRET_OFFSET.rotateBy(fieldToRobot.getRotation()));
    }

    public double getAngleAtTime(double time){
        return angleHistory.getSample(time).orElse(getLatencyCompensatedPosition()*2*Math.PI);
    }

    private static TurretIO getIOInstance(){
        if (CatzConstants.TurretOn == false) {
            System.out.println("Turret Disabled by CatzConstants");
            return new TurretIOSim(TurretConstants.gains);
        }
        switch (CatzConstants.hardwareMode) {
            case REAL:
                System.out.println("Turret Configured for Real");
                return new TurretIOTalonFX(TurretConstants.getIOConfig());
            case SIM:
                System.out.println("Turret Configured for Simulation");
                return new TurretIOSim(TurretConstants.gains);
                default:
                System.out.println("Turret Unconfigured");
                return new TurretIOSim(TurretConstants.gains);
        }
    }
}
