package frc.utils;

import edu.wpi.first.wpilibj.Preferences;

/**
 * Wrapper around the Preferences class
 * Note that default values in ensureExistance MUST be the correct type (ie. for double,
 * must be 1.0d instead of 1)
 */
public class RobotPreferences {
    public static enum PrefTypes {
        BOOL,
        DOUBLE,
        FLOAT,
        LONG,
        INT,
        STRING
    }

    private static void ensureExistance(String key, PrefTypes type, Object defaultValue) {
        if (!Preferences.containsKey(key)) {
            switch (type) {
                case BOOL:
                    Preferences.initBoolean(key, (boolean)defaultValue);
                    break;
                case DOUBLE:
                    Preferences.initDouble(key, (double)defaultValue);
                    break;
                case FLOAT:
                    Preferences.initFloat(key, (float)defaultValue);
                    break;
                case INT:
                    Preferences.initInt(key, (int)defaultValue);
                    break;
                case LONG:
                    Preferences.initLong(key, (long)defaultValue);
                    break;
                case STRING:
                    Preferences.initString(key, (String)defaultValue);
                    break;
                default:
                    break;
            }
        }
    }

    /**
    * @return true if preference is set to curve, false if preference is slew
    */
    public static boolean getInputFilter() {
        String key = "Input Filter Type (True for curve, False for slew): ";
        ensureExistance(key, PrefTypes.BOOL, true);
        return Preferences.getBoolean(key, true);
    }

    /**
    * @return integer>0 if IF is true, otherwise 1
    */
    public static double getCurvePower() {
        String key = "Curve Power (Only if IF is True): ";
        ensureExistance(key, PrefTypes.DOUBLE, 3.0d);
        return Preferences.getDouble(key, 3.0d);
    }

    /**
    * @return double>0 if IF is false, otherwise 1
    */
    public static double getSlewRateLimit() {
        String key = "Slew limit (Only if IF is False): ";
        ensureExistance(key, PrefTypes.DOUBLE, 1.0d);
        return Preferences.getDouble(key, 1.0d);
    }

    /**
     * @return true if should use limelight
     */
    public static boolean shouldUseLimelight() {
        String key = "Use Limelight";
        ensureExistance(key, PrefTypes.BOOL, true);
        return Preferences.getBoolean(key, true);
    }
}
