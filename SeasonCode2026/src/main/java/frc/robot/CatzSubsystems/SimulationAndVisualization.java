package frc.robot.CatzSubsystems;


import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzShooter.CatzTurret.TurretConstants;

public class SimulationAndVisualization {

    // Constants
    private static final double TURRET_X_OFFSET = 0.2; // Meters from robot center
    private static final double TURRET_Y_OFFSET = 0.0;
    private static final double INTAKE_X_OFFSET = 0.4;
    private static final double TURRET_HEIGHT_OFF_GROUND = 0.5; // For 3D poses

    private static final double TURRET_LENGTH = 0.3;
    private static final double HOOD_LENGTH = 0.15;
    private static final double INTAKE_LENGTH = 0.25;

    //  Mechanism2d Components
    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismRoot2d turretRoot;
    private final LoggedMechanismLigament2d turretLigament;
    private final LoggedMechanismLigament2d hoodLigament;

    private final LoggedMechanismRoot2d intakeRoot;
    private final LoggedMechanismLigament2d intakeLigament;

    private final String name;

    private double lastShotTime = 0.0;
    private final double TIME_BETWEEN_SHOTS = 0.2; // 5 balls per second

    // Index 0: Turret, 1: Hood, 2: Intake
    private Pose3d[] componentPoses = { new Pose3d(), new Pose3d(), new Pose3d() };

    public SimulationAndVisualization(String name) {
        this.name = name;

        // 1. Initialize Canvas (6m x 6m)
        mechanism = new LoggedMechanism2d(6.0, 6.0, new Color8Bit(Color.kDarkGray));

        // 2. Build Turret Hierarchy (Root -> Turret -> Hood)
        // The root is where the turret is mounted on the robot chassis
        turretRoot = mechanism.getRoot(name + "TurretRoot", 3.0 + TURRET_X_OFFSET, 3.0 + TURRET_Y_OFFSET);

        turretLigament = turretRoot.append(
            new LoggedMechanismLigament2d("Turret", TURRET_LENGTH, 0, 6, new Color8Bit(Color.kBlue))
        );

        // Append Hood to Turret (so it rotates WITH the turret)
        hoodLigament = turretLigament.append(
            new LoggedMechanismLigament2d("Hood", HOOD_LENGTH, 0, 4, new Color8Bit(Color.kYellow))
        );

        // 3. Build Intake Hierarchy (Root -> Intake)
        intakeRoot = mechanism.getRoot(name + "IntakeRoot", 3.0 + INTAKE_X_OFFSET, 3.0);

        intakeLigament = intakeRoot.append(
            new LoggedMechanismLigament2d("Intake", INTAKE_LENGTH, 0, 6, new Color8Bit(Color.kOrange))
        );
    }

    /**
     * Updates the simulation visualization and handles projectile shooting.
     * * @param intakeAngle Current angle of the intake
     * @param hoodAngle   Current angle of the shooter hood (controls vertical launch angle)
     * @param turretAngle Current angle of the turret (controls horizontal launch direction)
     * @param shooterRPM  Current RPM of the shooter flywheel (determines launch speed)
     * @param isShooting  True if the robot is currently shooting a projectile
     * @param isIntaking  True if the robot is currently intaking a projectile
     */
    public void update(Rotation2d intakeAngle, Rotation2d hoodAngle, Rotation2d turretAngle, double shooterRPM, boolean isShooting) {
        double currentTime = Logger.getTimestamp();

        if (Robot.isSimulation()) {
        //    isShooting = DriverStation.isEnabled();

            // Handle Projectile Shooting
            if (isShooting && (currentTime - lastShotTime > TIME_BETWEEN_SHOTS)) {
                // Fetch current robot state from Drivetrain Simulation
                // Note: Ensure CatzDrivetrain exposes these methods or equivalent access to sim state
                Pose2d robotPose = CatzDrivetrain.driveSimulationInstance.getActualPoseInSimulationWorld();
                ChassisSpeeds chassisSpeeds = CatzDrivetrain.driveSimulationInstance.getActualSpeedsFieldRelative();

                if (chassisSpeeds == null) {
                    chassisSpeeds = new ChassisSpeeds(); // Fallback to zero if not available
                }

                RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
                    // Specify the position of the chassis when the gp is launched
                    robotPose.getTranslation(),
                    // Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
                    TurretConstants.TURRET_OFFSET,
                    // Specify the field-relative speed of the chassis
                    chassisSpeeds,
                    // The shooter facing direction (Robot Rotation + Turret Rotation)
                    turretAngle,
                    // Initial height of the flying GP
                    Units.Meter.of(0.45),
                    // The launch speed (proportional to RPM, approx 16 m/s at 6000 RPM)
                    Units.MetersPerSecond.of(8.0),
                    // The angle at which the note is launched (Hood Angle)
                    Units.Radians.of(3.14/4)
                );

                // Configure Visualization Callbacks (Optional but recommended by docs)
                fuelOnFly.withProjectileTrajectoryDisplayCallBack(
                    (pose3ds) -> Logger.recordOutput("Flywheel/FuelProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
                    (pose3ds) -> Logger.recordOutput("Flywheel/FuelProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new))
                );

                // Register the projectile to the simulation arena
                SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);

                lastShotTime = currentTime; // Reset timer
            }

            updateSimulation();
        }

        //------------------------------------------------
        // Update mechanism 2d
        //------------------------------------------------
        turretLigament.setAngle(turretAngle);

        // If the hood angle is relative to the turret in code, use it directly.
        // If hoodAngle is absolute (field relative), subtract turretAngle.
        // Assuming relative here:
        hoodLigament.setAngle(hoodAngle);

        intakeLigament.setAngle(intakeAngle);

        // Log the 2D mechanism
        Logger.recordOutput("Mechanism2d/" + name, mechanism);

        // --- Update 3D Poses (For AdvantageScope 3D View) ---
        update3DPoses(intakeAngle, hoodAngle, turretAngle);
        Logger.recordOutput("Mechanism3d/" + name + "/Poses", componentPoses);
    }

    /**
     * Calculates the 3D pose of components relative to the robot center.
     * This allows you to visualize the moving parts on the 3D field in AdvantageScope.
     */
    private void update3DPoses(Rotation2d intakeAngle, Rotation2d hoodAngle, Rotation2d turretAngle) {
        // 1. Turret Pose (Robot Center -> Translate to mount -> Rotate)
        Transform3d turretTransform = new Transform3d(
            new Translation3d(TURRET_X_OFFSET, TURRET_Y_OFFSET, TURRET_HEIGHT_OFF_GROUND),
            new Rotation3d(0, 0, turretAngle.getRadians())
        );
        componentPoses[0] = new Pose3d().plus(turretTransform);

        // 2. Hood Pose (Turret Pose -> Translate up -> Rotate Hood)
        // This assumes the hood pivots at the end of the turret structure or slightly above it
        Transform3d hoodTransform = turretTransform.plus(new Transform3d(
            new Translation3d(0, 0, 0.1), // Slightly above turret center
            new Rotation3d(0, -hoodAngle.getRadians(), 0) // Pitch rotation
        ));
        componentPoses[1] = new Pose3d().plus(hoodTransform);

        // 3. Intake Pose
        Transform3d intakeTransform = new Transform3d(
            new Translation3d(INTAKE_X_OFFSET, 0, 0.2),
            new Rotation3d(0, -intakeAngle.getRadians(), 0) // Pitch rotation
        );
        componentPoses[2] = new Pose3d().plus(intakeTransform);
    }

    public void resetSimulationArenaAndPose() {
        if (CatzConstants.hardwareMode == CatzConstants.RobotHardwareMode.REAL) return;

        CatzRobotTracker.Instance.resetPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (CatzConstants.hardwareMode == CatzConstants.RobotHardwareMode.REAL) return;

        SimulatedArena.getInstance().simulationPeriodic();
        CatzDrivetrain.driveSimulationInstance.periodic();

        // Publish to telemetry using AdvantageKit
        Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        Logger.recordOutput("FieldSimulation/RobotPosition", CatzDrivetrain.driveSimulationInstance.getActualPoseInSimulationWorld());
    }
}
