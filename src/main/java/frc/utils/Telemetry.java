package frc.utils;

public class Telemetry {
    private static Telemetry instance;

    private Telemetry() {
        
    }

    public static Telemetry getInstance() {
        if (instance == null) instance = new Telemetry();
        return instance;
    }


}
