package frc.robot.CatzSubsystems.CatzArm;

import static frc.robot.CatzSubsystems.CatzArm.ArmConstants.ARM_GEAR_REDUCTION;
import static frc.robot.CatzSubsystems.CatzArm.ArmConstants.ARM_INITIAL_DEGREES;
import static frc.robot.CatzSubsystems.CatzArm.ArmConstants.ARM_MOTOR_ID;
import static frc.robot.CatzSubsystems.CatzArm.ArmConstants.CURRENT_LIMIT;
import static frc.robot.CatzSubsystems.CatzArm.ArmConstants.motionMagicParameters;
import static frc.robot.CatzSubsystems.CatzArm.ArmConstants.slot0_gains;
import static frc.robot.CatzSubsystems.CatzArm.ArmConstants.slot1_gains;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ArmIOReal implements ArmIO{
    private TalonFX armMotor = new TalonFX(ARM_MOTOR_ID);

    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0).withUpdateFreqHz(0.0);
    private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final StatusSignal<Angle> armPosition;
    private final StatusSignal<AngularVelocity> armVelocity;
    private final StatusSignal<Voltage> armAppliedVolts;
    private final StatusSignal<Current> armSupplyCurrent;
    private final StatusSignal<Current> armTorqueCurrent;
    private final StatusSignal<Temperature> armTempCelsius;

    public ArmIOReal() {
        armPosition = armMotor.getPosition();
        armVelocity = armMotor.getVelocity();
        armAppliedVolts = armMotor.getMotorVoltage();
        armSupplyCurrent = armMotor.getSupplyCurrent();
        armTorqueCurrent = armMotor.getTorqueCurrent();
        armTempCelsius = armMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            armPosition,
            armVelocity,
            armAppliedVolts,
            armSupplyCurrent,
            armTorqueCurrent,
            armTempCelsius);

        config.Slot0.kP = slot0_gains.kP();
        config.Slot0.kI = slot0_gains.kI();
        config.Slot0.kD = slot0_gains.kD();
        config.Slot0.kS = slot0_gains.kS();
        config.Slot0.kV = slot0_gains.kV();
        config.Slot0.kA = slot0_gains.kA();


        config.Slot1.kP = slot1_gains.kP();
        config.Slot1.kI = slot1_gains.kI();
        config.Slot1.kD = slot1_gains.kD();
        config.Slot1.kS = slot1_gains.kS();
        config.Slot1.kV = slot1_gains.kV();
        config.Slot1.kA = slot1_gains.kA();

        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        config.MotionMagic.MotionMagicCruiseVelocity = motionMagicParameters.mmCruiseVelocity();
        config.MotionMagic.MotionMagicAcceleration = motionMagicParameters.mmAcceleration();
        config.MotionMagic.MotionMagicJerk = motionMagicParameters.mmJerk();

        armMotor.getConfigurator().apply(config, 1.0);

        armMotor.setPosition(Units.degreesToRotations(ARM_INITIAL_DEGREES));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.isArmMotorConnected =
            BaseStatusSignal.refreshAll(
                    armPosition,
                    armVelocity,
                    armAppliedVolts,
                    armSupplyCurrent,
                    armTorqueCurrent,
                    armTempCelsius)
                .isOK();
        inputs.positionDegreesFinalShaft = (armPosition.getValueAsDouble() / ARM_GEAR_REDUCTION) * 360.0;
        inputs.velocityRPM               = (armVelocity.getValueAsDouble() * 60.0) / ARM_GEAR_REDUCTION * 360.0;
        inputs.appliedVolts              = armAppliedVolts.getValueAsDouble();
        inputs.supplyCurrentAmps         = armSupplyCurrent.getValueAsDouble();
        inputs.torqueCurrentAmps         = armTorqueCurrent.getValueAsDouble();
        inputs.tempCelsius               = armTempCelsius.getValueAsDouble();
    }

    @Override
    public void resetPosition(double pos) {// Set the motor position in mechanism rotations
        armMotor.setPosition((pos / 360.0) * ARM_GEAR_REDUCTION);
    }

    @Override
    public void runSetpointUp(double targetDegrees, double feedforwardVolts) {
        double setpointRotations = ((targetDegrees / 360.0) * ARM_GEAR_REDUCTION);
        armMotor.setControl(positionControl.withPosition(setpointRotations)
                                           .withFeedForward(feedforwardVolts));
    }

    @Override
    public void runSetpointDown(double targetDegrees, double feedforwardVolts) {
        double setpointRotations = ((targetDegrees / 360.0) * ARM_GEAR_REDUCTION);
        armMotor.setControl(positionControl.withPosition(setpointRotations)
                                           .withFeedForward(feedforwardVolts));
    }

    @Override
    public void setGainsSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        System.out.println("kP: " + kP + " kI: " + kI + " kD: " + kD);
        armMotor.getConfigurator().apply(config);
    }

    @Override
    public void setGainsSlot1(double kP, double kI, double kD, double kS, double kV, double kA) {
        config.Slot1.kP = kP;
        config.Slot1.kI = kI;
        config.Slot1.kD = kD;
        config.Slot1.kS = kS;
        config.Slot1.kV = kV;
        config.Slot1.kA = kA;
        System.out.println("kP: " + kP + " kI: " + kI + " kD: " + kD);
        armMotor.getConfigurator().apply(config);
    }

    @Override
    public void runCharacterizationMotor(double input) {
        armMotor.setControl(voltageControl.withOutput(input));
    }

    @Override
    public void setPercentOutput(double percentOutput) {
        armMotor.setControl(new DutyCycleOut(percentOutput));
    }
}
