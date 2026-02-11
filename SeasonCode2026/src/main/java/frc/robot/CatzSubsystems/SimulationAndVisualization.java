package frc.robot.CatzSubsystems;

import static edu.wpi.first.units.Units.Foot;
import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;

public class SimulationAndVisualization {

    private static Pose3d[] simMechanismPoses = {new Pose3d(), new Pose3d(), new Pose3d()};

    public static final Translation2d superstructureOrigin2d = new Translation2d(0.95, 0.4);
    public static final Translation3d superstructureOrigin3d = new Translation3d(superstructureOrigin2d.getX(), 0.0, superstructureOrigin2d.getY());

    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(
            Units.Foot.of(8.0).in(Meters),
            Units.Foot.of(8.0).in(Meters),
            new Color8Bit(Color.kDarkGray)
    );
    private final LoggedMechanismLigament2d turretRotation;

    private final String name;

    public SimulationAndVisualization(String name) {
        this.name = name;
        LoggedMechanismRoot2d root = mechanism.getRoot(
                name + " Root",
                superstructureOrigin2d.getX(),
                superstructureOrigin2d.getY()
        );
        turretRotation = root.append(
                new LoggedMechanismLigament2d(
                        name + " Pivot",
                        Units.Inch.of(10.0).in(Foot),
                        0.0,
                        8.0,
                        new Color8Bit(Color.kFirstRed))
        );
    }

    /**
     * Updates the simulation visualization and handles projectile shooting.
     * * @param intakeAngle Current angle of the intake
     * @param hoodAngle   Current angle of the shooter hood (controls vertical launch angle)
     * @param turretAngle Current angle of the turret (controls horizontal launch direction)
     * @param shooterRPM  Current RPM of the shooter flywheel (determines launch speed)
     * @param isShooting  True if the robot is currently shooting a projectile
     */
    public void update(Rotation2d intakeAngle, Rotation2d hoodAngle, Rotation2d turretAngle, double shooterRPM, boolean isShooting) {
        if (Robot.isSimulation()) {
            Logger.recordOutput("Mechanism2d/" + name, mechanism);
            turretRotation.setAngle(turretAngle);

            // Handle Projectile Shooting
            if (isShooting) {
                // Fetch current robot state from Drivetrain Simulation
                // Note: Ensure CatzDrivetrain exposes these methods or equivalent access to sim state
                Pose2d robotPose = CatzDrivetrain.driveSimulationInstance.getSimulatedDriveTrainPose();

                // We assume there is a method to get field relative speeds for accurate physics
                // If this specific method doesn't exist, use driveSimulationInstance.getChassisSpeeds()
                // and rotate by robot heading if necessary.
                ChassisSpeeds chassisSpeeds = CatzDrivetrain.driveSimulationInstance.getDriveTrainSimulatedChassisSpeedsFieldRelative();

                if (chassisSpeeds == null) {
                    chassisSpeeds = new ChassisSpeeds(); // Fallback to zero if not available
                }

                RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
                    // Specify the position of the chassis when the note is launched
                    robotPose.getTranslation(),
                    // Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
                    new Translation2d(0.2, 0),
                    // Specify the field-relative speed of the chassis
                    chassisSpeeds,
                    // The shooter facing direction (Robot Rotation + Turret Rotation)
                    turretAngle,
                    // Initial height of the flying note
                    Units.Meter.of(0.45),
                    // The launch speed (proportional to RPM, approx 16 m/s at 6000 RPM)
                    Units.MetersPerSecond.of((shooterRPM / 6000.0) * 20.0),
                    // The angle at which the note is launched (Hood Angle)
                    Units.Radians.of(hoodAngle.getRadians())
                );

                // Configure Visualization Callbacks (Optional but recommended by docs)
                fuelOnFly.withProjectileTrajectoryDisplayCallBack(
                    (pose3ds) -> Logger.recordOutput("Flywheel/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
                    (pose3ds) -> Logger.recordOutput("Flywheel/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new))
                );

                // Register the projectile to the simulation arena
                SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
            }

            updateSimulation();
        }

        Logger.recordOutput("Visualization/FinalComponentPoses", simMechanismPoses);
    }

    public void resetSimulation() {
        if (CatzConstants.hardwareMode == CatzConstants.RobotHardwareMode.REAL) return;

        CatzRobotTracker.Instance.resetPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (CatzConstants.hardwareMode == CatzConstants.RobotHardwareMode.REAL) return;

        SimulatedArena.getInstance().simulationPeriodic();
        // Publish to telemetry using AdvantageKit
        Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        Logger.recordOutput("FieldSimulation/RobotPosition", CatzDrivetrain.driveSimulationInstance.getSimulatedDriveTrainPose());
    }

    // TODO Possible depreciated code
    public static Pose3d getSimPose(int index) {
        return simMechanismPoses[index];
    }

    public static void setSimPose(int index, Pose3d pose) {
        simMechanismPoses[index] = pose;
    }
}
