package frc.robot;

public class RobotMap {
    public static final String CANBUS_NAME = "rio";

    public static class Climber { 
        public static final int CLIMBER_MASTER = 0; // Drive motor - FR
        public static final int CLIMBER_FOLLOWER = 0; // Drive motor - FL
    }

    public static class Intake {
        public static final int INTAKE_MOTOR = 18;
        public static final int TOP_IR = 1; 
        public static final int BOTTOM_IR = 2; 
    }

    public static class Arm {
        public static final int LEFT_MOTOR_PORT = 32;
        public static final int RIGHT_MOTOR_PORT = 14;
        public static final int ANALOG_POT_PORT = 0; //TO CHANGE
    }

    public static class Flywheel {
        public static final int LEFT_FLYWHEEL = 0; 
        public static final int RIGHT_FLYWHEEL = 0; 
    }
}
