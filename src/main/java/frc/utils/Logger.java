package frc.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Logger {

    /**
     * The fallthrough log level selection (ie. if warning is selected, warnings, info and debug are displayed)
     */
    public enum LogLevel {
        /**
        * Only used when getting a feature to work
        */
        DEBUG,
        /**
        * Printing settings or computed values
        */
        INFO,
        /**
        * Warning the user of potential error
        */
        WARNING,
        /**
        * Alerting the user of function-impairing errors
        */
        CRITICAL
    }

    private static SendableChooser<LogLevel> selectedLogLevel; 
    static {
        selectedLogLevel = new SendableChooser<LogLevel>();
        selectedLogLevel.addOption("Debug", LogLevel.DEBUG);
        selectedLogLevel.setDefaultOption("Info", LogLevel.INFO);
        selectedLogLevel.addOption("Warning", LogLevel.WARNING);
        selectedLogLevel.addOption("Critical", LogLevel.CRITICAL);
        SmartDashboard.putData("Log Level", selectedLogLevel);
    }

    private static double lastTime = 0.0;

    //A simple method to return the time since startup of the robot, along with the time since the last log, as a log prefix
    private static String getTimeString() {
        double seconds = Timer.getFPGATimestamp();
        String timeString = String.format("[%d:%02d +%.2f] ", (int) seconds / 60, (int) seconds % 60, seconds-lastTime);
        lastTime = seconds;
        return timeString;
    }

    /**
     * Log an object(s) to the console.
     * Note that the default log option is info, so if you log a debug message before the user can change the log level, <b>they will not see it</b>
     * @param level: The severity level of your message
     * @param objects: The objects to print. If multiple, they will be seperated by a '|' symbol
     */
    public static void log(LogLevel level, Object... objects) {
        if (level.ordinal() >= selectedLogLevel.getSelected().ordinal()) {
            if (objects.length == 1) {
                System.out.println(getTimeString() + objects[0]);
                return;
            }
            String toPrint = "";
            for (int i = 0; i < objects.length-1; i++) {
                toPrint = toPrint.concat(objects[i].toString() + " | ");
            }
            toPrint = toPrint.concat(objects[objects.length-1].toString());
            System.out.println(getTimeString() + toPrint);
        }
    }
}