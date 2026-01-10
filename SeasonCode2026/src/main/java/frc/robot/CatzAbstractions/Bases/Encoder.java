package frc.robot.CatzAbstractions.Bases;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CatzAbstractions.io.EncoderIO;
import frc.robot.CatzAbstractions.io.EncoderIOInputsAutoLogged;

public class Encoder {

    private final String name;
    private final EncoderIO io;
    private final EncoderIOInputsAutoLogged inputs = new EncoderIOInputsAutoLogged();

    /**
     * @param io The IO implementation (Real, Sim, or Replay)
     * @param name The name for logging (e.g., "Arm/AbsEncoder")
     */
    public Encoder(EncoderIO io, String name) {
        this.io = io;
        this.name = name;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------------------------
    //  Data Getters
    //-----------------------------------------------------------------------------------------------------------------------------------------------------------
    /** @return The absolute position as an Angle object. */
    public Angle getPosition() {
        return Units.Rotations.of(inputs.absolutePositionRotations);
    }

    /** @return The position in raw rotations (primitive double). */
    public double getPositionRotations() {
        return inputs.absolutePositionRotations;
    }

    /** @return The velocity as an AngularVelocity object. */
    public AngularVelocity getVelocity() {
        return Units.RotationsPerSecond.of(inputs.velocityRPS);
    }

    public boolean isConnected() {
        return inputs.isConnected;
    }

    public void setPosition(Angle position) {
        io.setPosition(position.in(Units.Rotations));
    }

    //-------------------------------------------------------------------------------------------------------------------------------------------------------------
    //  Command Factories
    //-------------------------------------------------------------------------------------------------------------------------------------------------------------
    /**
     * Waits until the encoder reads a value greater than the target.
     */
    public Command waitUntilGreaterThan(Angle target) {
        return Commands.waitUntil(() -> getPosition().gt(target));
    }

    /**
     * Waits until the encoder reads a value less than the target.
     */
    public Command waitUntilLessThan(Angle target) {
        return Commands.waitUntil(() -> getPosition().lt(target));
    }

    /**
     * Waits until the encoder is within a tolerance of the target angle.
     */
    public Command waitUntilInRange(Angle target, Angle tolerance) {
        return Commands.waitUntil(() ->
            Math.abs(getPosition().minus(target).in(Units.Degrees)) < tolerance.in(Units.Degrees)
        );
    }
}
