package frc.robot.CatzSubsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SubsystemVisualizer {
    // --- Constants (Tune these to match your CAD) ---
    private static final double TURRET_X_OFFSET = 0.2; // Meters from robot center
    private static final double TURRET_Y_OFFSET = 0.0;
    private static final double INTAKE_X_OFFSET = 0.4;
    private static final double TURRET_HEIGHT_OFF_GROUND = 0.5; // For 3D poses

    private static final double TURRET_LENGTH = 0.3;
    private static final double HOOD_LENGTH = 0.15;
    private static final double INTAKE_LENGTH = 0.25;

    // --- Mechanism2d Components ---
    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismRoot2d turretRoot;
    private final LoggedMechanismLigament2d turretLigament;
    private final LoggedMechanismLigament2d hoodLigament;

    private final LoggedMechanismRoot2d intakeRoot;
    private final LoggedMechanismLigament2d intakeLigament;

    private final String name;

    // --- 3D Components ---
    // Index 0: Turret, 1: Hood, 2: Intake
    private Pose3d[] componentPoses = { new Pose3d(), new Pose3d(), new Pose3d() };

    public SubsystemVisualizer(String name) {
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

    public void update(Rotation2d intakeAngle, Rotation2d hoodAngle, Rotation2d turretAngle) {

        // --- Update Mechanism2d (The "Stick Figure") ---
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
}
