package frc.robot.CatzSubsystems;

import java.util.Set;


import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CatzConstants;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.Autonomous.autoSequence.DepotCornerSwipe;
import frc.robot.Autonomous.autoSequence.DepotMiddleSwipe;
import frc.robot.Autonomous.autoSequence.OppositeDepotCornerSwipe;
import frc.robot.Autonomous.autoSequence.OppositeDepotMiddleSwipe;
import frc.robot.Autonomous.autoSequence.OppositeTowerSwipe;
import frc.robot.Autonomous.autoSequence.TowerSwipe;
import frc.robot.CatzSubsystems.CatzArm.CatzElevator.CatzElevator;
import frc.robot.CatzSubsystems.CatzArm.CatzElevator.CatzElevator.ArmState;
import frc.robot.CatzSubsystems.CatzArm.CatzElevator.ElevatorConstants;
import frc.robot.CatzSubsystems.CatzArm.CatzMantaPivot.CatzMantaPivot;
import frc.robot.CatzSubsystems.CatzArm.CatzMantaPivot.MantaPivotConstants;
import frc.robot.CatzSubsystems.CatzClimb.CatzClimb;
import frc.robot.CatzSubsystems.CatzClimb.ClimbConstants;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeDeploy.CatzIntakeDeploy;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeDeploy.IntakeDeployConstants;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.CatzIntakeRoller;
import frc.robot.CatzSubsystems.CatzIntake.CatzIntakeRoller.IntakeRollerConstants;
import frc.robot.Commands.DriveAndRobotOrientationCmds.PIDDriveCmd;
import frc.robot.Utilities.Setpoint;

public class CatzSuperstructure {
    public static final CatzSuperstructure Instance = new CatzSuperstructure();

    public boolean isClimbMode = false;
    public boolean isDefenseMode = false;
    private boolean isScoring = false;
    private boolean isHoarding = false;

    private boolean initialShootReady = false;

    private boolean climbManual = false;
    private boolean hoodManual = false;
    private boolean turretManual = false;
    private boolean deployManual = false;

    private final SubsystemVisualizer visualizer;

    private ArmState armState = ArmState.STOW;
    private final Command[] scoreCommands = {armL1(), armL2(), armL3(), armL4()};
    private int scoreLevel = 0;

    private final TowerSwipe outpostSwipeRoutine;
    private final OppositeTowerSwipe outpostOppositeSwipeRoutine;
    private final DepotMiddleSwipe depotMiddleSwipeRoutine;
    private final OppositeDepotMiddleSwipe depotOppositeMiddleSwipeRoutine;
    private final DepotCornerSwipe depotCornerSwipeRoutine;
    private final OppositeDepotCornerSwipe depotOppositeCornerSwipeRoutine;

    private CatzSuperstructure() {
        this.visualizer = new SubsystemVisualizer("SuperstructureViz");

        CatzConstants.autoFactory = new AutoFactory(
                                                  CatzRobotTracker.getInstance()::getEstimatedPose,
                                                  CatzRobotTracker.getInstance()::resetPose,
                                                  CatzDrivetrain.getInstance()::followChoreoTrajectoryExecute,
                                                  true,
                                                  CatzDrivetrain.getInstance()
                                                ); //it is apparently a good idea to initialize these variables not statically because there can be race conditions

        outpostSwipeRoutine = new TowerSwipe();
        outpostOppositeSwipeRoutine = new OppositeTowerSwipe();
        depotMiddleSwipeRoutine = new DepotMiddleSwipe();
        depotOppositeMiddleSwipeRoutine = new OppositeDepotMiddleSwipe();
        depotCornerSwipeRoutine = new DepotCornerSwipe();
        depotOppositeCornerSwipeRoutine = new OppositeDepotCornerSwipe();
    }

    private double getRumbleStrength() {
        Pose2d currentPose = CatzRobotTracker.Instance.getEstimatedPose();
        if (currentPose.getY() > FieldConstants.BOTTOM_TRENCH_MAX_Y
                && currentPose.getY() < FieldConstants.TOP_TRENCH_MIN_Y) {
            return 0.0;
        }
        double distFromTrench = Math.min(Math.abs(currentPose.getX() - FieldConstants.LEFT_TRENCH_X),
                Math.abs(currentPose.getX() - FieldConstants.RIGHT_TRENCH_X));
        if (distFromTrench > FieldConstants.MIN_RUMBLE_DIST)
            return 0.0;
        return 1.0 - distFromTrench / FieldConstants.MIN_RUMBLE_DIST;
    }

    public boolean isIntakeOn = false;

    // --------------------------------------------------------------------------
    // Public Command States
    // --------------------------------------------------------------------------


    public Command armStow() {
        return CatzElevator.Instance.setpointCommand(ElevatorConstants.STOW)
        .alongWith(CatzMantaPivot.Instance.setpointCommand(MantaPivotConstants.STOW))
        .beforeStarting(() -> armState = ArmState.STOW);
    }
    public Command armHold() {
        return CatzElevator.Instance.setpointCommand(ElevatorConstants.HOLD)
        .alongWith(CatzMantaPivot.Instance.setpointCommand(MantaPivotConstants.HOLD))
        .beforeStarting(() -> armState = ArmState.HOLD);
    }
    public Command armL1() {
        return CatzElevator.Instance.setpointCommand(ElevatorConstants.L1)
        .alongWith(CatzMantaPivot.Instance.setpointCommand(MantaPivotConstants.L1))
        .beforeStarting(() -> armState = ArmState.L1);
    }
    public Command armL2() {
        return CatzElevator.Instance.setpointCommand(ElevatorConstants.L2)
        .alongWith(CatzMantaPivot.Instance.setpointCommand(MantaPivotConstants.L2))
        .beforeStarting(() -> armState = ArmState.L2);
    }
    public Command armL3() {
        return CatzElevator.Instance.setpointCommand(ElevatorConstants.L3)
        .alongWith(CatzMantaPivot.Instance.setpointCommand(MantaPivotConstants.L3))
        .beforeStarting(() -> armState = ArmState.L3);
    }
    public Command armL4() {
        return CatzElevator.Instance.setpointCommand(ElevatorConstants.L4)
        .alongWith(CatzMantaPivot.Instance.setpointCommand(MantaPivotConstants.L4))
        .beforeStarting(() -> armState = ArmState.L4);
    }
    public Command armAlgae() {
        return CatzElevator.Instance.setpointCommand(ElevatorConstants.ALGAE)
        .alongWith(CatzMantaPivot.Instance.setpointCommand(MantaPivotConstants.ALGAE))
        .beforeStarting(() -> armState = ArmState.ALGAE);
    }
    public Command armLX() {
        return scoreCommands[scoreLevel];
    }

    /* --- INTAKE --- */
    public Angle intakeSetpoint = IntakeDeployConstants.DEPLOY_POSITION;
    public boolean isIntakeDeployed = true;

    public Command toggleIntakeDeploy() {
        return Commands.runOnce(() -> {
            if (isIntakeDeployed) {
                intakeSetpoint = IntakeDeployConstants.STOW_POSITION;
            } else {
                intakeSetpoint = IntakeDeployConstants.DEPLOY_POSITION;
            }
            isIntakeDeployed = !isIntakeDeployed;
        });
    }

    public Command upIntake() {
        return Commands.runOnce(() -> intakeSetpoint = IntakeDeployConstants.UP_POSITION);
    }

    public Command deployIntake() {
        return Commands.runOnce(() -> {
            intakeSetpoint = IntakeDeployConstants.DEPLOY_POSITION;
            isIntakeDeployed = true;
        });
    }

    public Command stowIntake() {
        return Commands.runOnce(() -> {
            intakeSetpoint = IntakeDeployConstants.STOW_POSITION;
            isIntakeDeployed = false;

        });
    }

    public Command jiggleIntakeCommand() {
            return Commands.run(() -> {
            double time = Timer.getFPGATimestamp();
            double angleRot = Math.sin(time * IntakeDeployConstants.JIGGLE_FREQUENCY * (2 * Math.PI)) > 0
                    ? IntakeDeployConstants.UP_POSITION.in(Units.Rotations)
                    : IntakeDeployConstants.DEPLOY_POSITION.in(Units.Rotations);

            CatzIntakeRoller.Instance.applySetpoint(IntakeRollerConstants.JIGGLE_SETPOINT);
            intakeSetpoint = Units.Rotations.of(angleRot);

        }, CatzIntakeRoller.Instance);
    }

    public Command toggleIntakeRollers() {
        return Commands.runOnce(() -> {
            if (isIntakeOn) {
                isIntakeOn = false;
                CatzIntakeRoller.Instance.applySetpoint(IntakeRollerConstants.OFF_SETPOINT);

                // if(!isScoring){
                //     CatzDrivetrain.getInstance().setNormalConfig();
                // }
                RobotContainer.rumbleDrv(0.0);

            } else {
                isIntakeOn = true;
                // CatzIntakeRoller.Instance.applySetpoint(IntakeRollerConstants.getOnSetpoint());
                CatzIntakeRoller.Instance.applySetpoint(IntakeRollerConstants.ON_SETPOINT);

                // if(!isScoring){
                    // CatzDrivetrain.getInstance().setIntakeMoveConfig();
                // }
                RobotContainer.rumbleDrv(0.05);
            }
        }, CatzIntakeRoller.Instance);
    }

    public Command intakeON() {
        return CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.ON_SETPOINT)
                .alongWith(Commands.runOnce(() -> isIntakeOn = true));
    }

    public Command intakeOFF() {
        return CatzIntakeRoller.Instance.setpointCommand(IntakeRollerConstants.OFF_SETPOINT)
                .alongWith(Commands.runOnce(() -> isIntakeOn = false));
    }

    public boolean getIsScoring() {
        return isScoring;
    }

    public boolean getIsHoarding() {
        return isHoarding;
    }

    /* --- COMMANDS FOR TESTING --- */

    public Command alignToBackUpClimb() {
        return Commands.defer(() -> {
            Translation2d currentTranslation = CatzRobotTracker.Instance.getEstimatedPose().getTranslation();
            return new PIDDriveCmd(FieldConstants.getClimbAwayPosition(currentTranslation), true);
        }, Set.of(CatzDrivetrain.getInstance()));//.onlyIf(() -> isClimbMode || DriverStation.isAutonomous());
    }

    public Command alignToCloseClimb() {
        return Commands.defer(() -> {
            Translation2d currentTranslation = CatzRobotTracker.Instance.getEstimatedPose().getTranslation();
            return new PIDDriveCmd(1.5, FieldConstants.getClimbClosePosition(currentTranslation), true);
        }, Set.of(CatzDrivetrain.getInstance()));//.onlyIf(() -> isClimbMode || DriverStation.isAutonomous());
    }

    public Command autoClimbCommand() {
        return Commands.sequence(
                        cmdClimbReach(),
                        deployIntake(),
                        alignToBackUpClimb().withTimeout(2.0),
                        alignToCloseClimb().withTimeout(2.0),
                        stowIntake(),
                        cmdClimbStow()).beforeStarting(() -> isClimbMode = true);
    }

    public Command autoClimbLowerCommand() {
        return Commands.sequence(
                cmdClimbReach(),
                deployIntake()
                // alignToBackUpClimb()
            ).andThen(() -> isClimbMode = false);
    }

    public Command cmdClimbReach() {
        return CatzClimb.Instance.setpointCommand(ClimbConstants.REACH_SETPOINT);
    }

    public Command cmdClimbStow() {
        return CatzClimb.Instance.setpointCommand(ClimbConstants.STOW_SETPOINT);
    }

    public Command toggleManualExtendClimb() {
        return Commands.runOnce(() -> {


            CatzMantaPivot.Instance.followSetpointCommand(() -> {
                double input = -(RobotContainer.xboxAux.getLeftY()) * 12;
                if (Math.abs(input) < 0.84)
                    return Setpoint.withVoltageSetpoint(0.0);

                return Setpoint.withVoltageSetpoint(input);
            }).schedule();
            // if (climbManual == false) {

            // } else {
            //     // CatzClimb.Instance.setpointCommand(ClimbConstants.STOW_SETPOINT).schedule();
            //     climbManual = false;
            // }
        });
    }

    public Command toggleManualMantaPivot() {
        return Commands.runOnce(() -> {
            disableManuals(CatzMantaPivot.Instance);

            CatzMantaPivot.Instance.followSetpointCommand(() -> {
                double input = -(RobotContainer.xboxAux.getLeftY()) * 12;
                if (Math.abs(input) < 0.84)
                    return Setpoint.withVoltageSetpoint(0.0);

                return Setpoint.withVoltageSetpoint(input);
            }).schedule();
        });
    }

    public Command toggleManualElevator() {
    return CatzElevator.Instance.followSetpointCommand(() -> {
                double input = -(RobotContainer.xboxAux.getLeftY()) * 12;
                if (Math.abs(input) < 0.84)
                    return Setpoint.withVoltageSetpoint(0.0);

                return Setpoint.withVoltageSetpoint(input);
        }).beforeStarting(() -> disableManuals(CatzElevator.Instance));
    }

    public Command toggleManualDeploy() {
        return Commands.runOnce(() -> {

            if (deployManual == false) {
                disableManuals(CatzIntakeDeploy.Instance);
                deployManual = true;

                CatzIntakeDeploy.Instance.followSetpointCommand(() -> {
                    double input = -(RobotContainer.xboxAux.getLeftY()) * 12;
                    if (Math.abs(input) < 0.84)
                        return Setpoint.withVoltageSetpoint(0.0);

                    return Setpoint.withVoltageSetpoint(input);
                }).schedule();

            } else {
                CatzIntakeDeploy.Instance.setpointCommand(IntakeDeployConstants.STOW).schedule();
                deployManual = false;
            }
        });
    }

    private void disableManuals(Object excludedSubsystem) {
        // Reset flags
        climbManual = false;
        hoodManual = false;
        turretManual = false;
        deployManual = false;

        // Only schedule stow if it's NOT the one we are about to manually control
        if (excludedSubsystem != CatzClimb.Instance) {
            CatzClimb.Instance.setpointCommand(ClimbConstants.STOW_SETPOINT).schedule();
        }
        if (excludedSubsystem != CatzIntakeDeploy.Instance) {
            CatzIntakeDeploy.Instance.setpointCommand(IntakeDeployConstants.STOW).schedule();
        }
    }

    public boolean canResetPose = false;

    public Command resetClimbPose() {
        return CatzClimb.Instance.setCurrentPositionCommand(Units.Rotations.of(0.0));//.onlyIf(()->canResetPose);
    }

    public Command resetDeployPose() {
        return CatzIntakeDeploy.Instance.setCurrentPositionCommand(IntakeDeployConstants.STOW_POSITION).onlyIf(()->canResetPose);
    }

    public Command enableClimbSoftLimit() {
        return Commands.runOnce(() -> {
            CatzClimb.Instance.setSoftLimitsEnabled(true, true);
        });
    }

    public Command disableClimbSoftLimit() {
        return Commands.runOnce(() -> {
            CatzClimb.Instance.setSoftLimitsEnabled(false, false);
        });
    }

    public Command toggleDefenseMode() {
        return Commands.either(
            Commands.runOnce(() -> CatzDrivetrain.getInstance().setNormalConfig()).finallyDo(() -> isDefenseMode = false),
            Commands.runOnce(() -> CatzDrivetrain.getInstance().setDefenseConfig()).finallyDo(() -> isDefenseMode = true).alongWith(CatzSuperstructure.Instance.stowIntake()),
            () -> isDefenseMode
        );
    }

    public Command incrementScorePosition() {
        return Commands.runOnce(() -> {
            scoreLevel++;
            scoreLevel %= 4;
        });
    }

    public Command decrementScorePosition() {
        return Commands.runOnce(() -> {
            scoreLevel--;
            scoreLevel %= 4;
        });
    }

    public Command TowerSwipePosition() {
    return Commands.defer(() -> {
        Translation2d currentTranslation = CatzRobotTracker.Instance.getEstimatedPose().getTranslation();

        boolean isOpponentSide = false;
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue){
            if(currentTranslation.getX() > FieldConstants.fieldXHalf){
                isOpponentSide = true;
            }
        }else{
            if(currentTranslation.getX() < FieldConstants.fieldXHalf){
                isOpponentSide = true;
            }
        }

        return new PIDDriveCmd(FieldConstants.getTowerSwipePosition(currentTranslation, isOpponentSide), false, 0.1, 20.0)
        .alongWith(deployIntake());
    }, Set.of(CatzDrivetrain.getInstance()));
  }

    public Command swipe() {
        return Commands.defer(() -> {
            Translation2d currentTranslation = CatzRobotTracker.Instance.getEstimatedPose().getTranslation();
            boolean flipAlliance = false;

            if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue){
                if(currentTranslation.getX() > FieldConstants.fieldXHalf){
                    flipAlliance = true;
                }
            }else{
                if(currentTranslation.getX() < FieldConstants.fieldXHalf){
                    flipAlliance = true;
                }
            }

            if (flipAlliance) {
                // Pass true because we are on the opponent side
                switch(FieldConstants.getCloserSwipe(currentTranslation, true)) {
                    case(1): return outpostOppositeSwipeRun();
                    case(2): return depotOppositeMiddleSwipeRun();
                    case(3): return depotOppositeCornerSwipeRun();
                    default: return Commands.none().andThen(Commands.print("none!!!!"));
                }
            }
            // Pass false because we are on our home side
            switch(FieldConstants.getCloserSwipe(currentTranslation, false)) {
                case(1): return outpostSwipeRun();
                case(2): return depotMiddleSwipeRun();
                case(3): return depotCornerSwipeRun();
                default: return Commands.none().andThen(Commands.print("none!!!!"));
            }

        }, Set.of(CatzDrivetrain.getInstance(), CatzIntakeDeploy.Instance, CatzIntakeRoller.Instance));
    }

    public Command outpostSwipeRun() {
        return Commands.print("okay!!1").andThen(outpostSwipeRoutine.getPathCommand());
    }

    public Command depotMiddleSwipeRun() {
        return Commands.print("okay!!2").andThen(depotMiddleSwipeRoutine.getPathCommand());
    }

    public Command depotCornerSwipeRun() {
        return Commands.print("okay!!3").andThen(depotCornerSwipeRoutine.getPathCommand());
    }

    public Command outpostOppositeSwipeRun(){
        return outpostOppositeSwipeRoutine.getPathCommand();
    }

    public Command depotOppositeMiddleSwipeRun(){
        return depotOppositeMiddleSwipeRoutine.getPathCommand();
    }

    public Command depotOppositeCornerSwipeRun(){
        return depotOppositeCornerSwipeRoutine.getPathCommand();
    }
}
