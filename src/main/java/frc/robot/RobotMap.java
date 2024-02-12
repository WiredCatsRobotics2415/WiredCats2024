package frc.robot;

public class RobotMap {
    public static final String CANBUS_NAME = "rio";

    public static class Climber { 
        public static final int CLIMBER_MASTER = 1; // Drive motor - FR
        public static final int CLIMBER_FOLLOWER = 4; // Drive motor - FL
    }

    public static class Intake {
        public static final int INTAKE_MOTOR = 11;
        public static final int TOP_IR = 1; 
        public static final int BOTTOM_IR = 2; 
    }
}
