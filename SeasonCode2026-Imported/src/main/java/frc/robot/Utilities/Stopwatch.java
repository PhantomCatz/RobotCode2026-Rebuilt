package frc.robot.Utilities;

import org.wpilib.units.Units;
import org.wpilib.units.measure.Time;
import org.wpilib.system.Timer;

public class Stopwatch {

	private double startTime = Double.POSITIVE_INFINITY;

	public void start() {
		startTime = Timer.getTimestamp();
	}

	public void startIfNotRunning() {
		if (Double.isInfinite(startTime)) {
			start();
		}
	}

	public Time getTime() {
		if (Double.isInfinite(startTime)) {
			return Units.Seconds.of(0.0);
		}
		return Units.Seconds.of(Timer.getTimestamp() - startTime);
	}

	public double getTimeAsDouble() {
		if (Double.isInfinite(startTime)) {
			return 0.0;
		}
		return Timer.getTimestamp() - startTime;
	}

	public void reset() {
		startTime = Double.POSITIVE_INFINITY;
	}

	public void resetAndStart() {
		startTime = Double.POSITIVE_INFINITY;
		start();
	}
}
