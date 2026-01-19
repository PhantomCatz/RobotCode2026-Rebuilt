package frc.robot.CatzSubsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;

public class SubystemVisualizer {
        private static Pose3d[] simMechanismPoses = {new Pose3d(), new Pose3d(), new Pose3d()};


        public static final Translation2d superstructureOrigin2d = new Translation2d(0.95, 0.4);
        public static final Translation3d superstructureOrigin3d = new Translation3d(superstructureOrigin2d.getX(), 0.0, superstructureOrigin2d.getY());

        private final LoggedMechanism2d mechanism = new LoggedMechanism2d(
                                                                Units.feetToMeters(8.0),
                                                                Units.feetToMeters(8.0),
                                                                new Color8Bit(Color.kDarkGray)
                                                                );
        private final LoggedMechanismLigament2d turretRotation;




        private final String name;
        public SubystemVisualizer(String name) {
                this.name = name;
                LoggedMechanismRoot2d root = mechanism.getRoot(
                                                        name + " Root",
                                                        superstructureOrigin2d.getX(),
                                                        superstructureOrigin2d.getY()
                );
                turretRotation = root.append(
                                                new LoggedMechanismLigament2d(
                                                                name + " Pivot",
                                                                Units.inchesToMeters(10.0),
                                                                0.0,
                                                                8.0,
                                                                new Color8Bit(Color.kFirstRed))
                );
        }



        // inatek angle shooter hood angle turret angle
        public void update(Rotation2d intakeAngle, Rotation2d hoodAngle, Rotation2d turretAngle) {
                if (Robot.isSimulation()) {
                        Logger.recordOutput("Mechanism2d/" + name, mechanism);
                        turretRotation.setAngle(turretAngle);
                }

                // Max of top of carriage or starting height


                Logger.recordOutput("Visualization/FinalComponentPoses", simMechanismPoses);
        }


        // TODO Possible depreciated code
        public static Pose3d getSimPose(int index) {
                return simMechanismPoses[index];
        }

        public static void setSimPose(int index, Pose3d pose) {
                simMechanismPoses[index] = pose;
        }
}
