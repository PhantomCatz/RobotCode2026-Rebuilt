package frc.robot.Utilities;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.wpilib.hardware.bus.CANPort;

import com.ctre.phoenix6.BaseStatusSignal;

public class CANBusManager {

    private static final Map<CANPort, List<BaseStatusSignal>> signals = new EnumMap<>(CANPort.class);
    private static final Map<CANPort, BaseStatusSignal[]> cached = new EnumMap<>(CANPort.class);

    public CANBusManager() {}

    public static void register(CANPort canBus, BaseStatusSignal... newSignals) {
        List<BaseStatusSignal> list = signals.computeIfAbsent(canBus, key -> new ArrayList<>());
        Collections.addAll(list, newSignals);
        cached.put(canBus, list.toArray(new BaseStatusSignal[0]));
    }

    public static void refreshAll() {
        for(var entry : cached.entrySet()) {
            var result = BaseStatusSignal.refreshAll(entry.getValue());

            Logger.recordOutput("CAN/" + entry.getKey().name() + "/OK", result.isOK());
        }
    }


}
