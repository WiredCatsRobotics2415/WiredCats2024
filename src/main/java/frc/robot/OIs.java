package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drive;
import frc.utils.RobotPreferences;

public class OIs {
    public static abstract class OI {
        static SendableChooser<Integer> oiChooser;
        static {
            oiChooser = new SendableChooser<Integer>();
            oiChooser.setDefaultOption("Gulikit Controller", 0);
            SmartDashboard.putData("OI", oiChooser);
        }

        /**
         * A convinience record to store two doubles without making a whole class
         */
        public static record TwoDControllerInput(double x, double y) {};

        //PROPERTIES
        /**
         * The deadband of all controller axes, in raw controller input values [-1, 1]
         */
        public double DEADBAND;

        /**
         * The binds map of an OI
         */
        public Map<String, Trigger> binds = new HashMap<String, Trigger>();

        //JOYSTICKS
        /**
         * Get appropriately scaled translation values, in raw controller units [-1, 1]
         */
        public abstract TwoDControllerInput getXY();

        /**
         * Get appropriately scaled rotation values, in raw controller units [-1, 1]
         */
        public abstract double getRotation();

        //UTILS
        /**
         * Retrieves input preferences from RobotPreferences.
         * Should be called from teleopInit()
         */
        public abstract void setPreferences();
    }
    
    public static class GulikitController extends OI {
        CommandXboxController controller;
        CommandJoystick numpad; 

        private boolean isCurve;
        private double curve;
        private double slewRate;

        private SlewRateLimiter xLimiter;
        private SlewRateLimiter yLimiter;

        public final double DEADBAND = 0.05;

        public void setPreferences() {
            if (RobotPreferences.getInputFilter()) {
                //Curve
                isCurve = true;
                curve = RobotPreferences.getCurvePower();
                if (curve < 1) curve = 1; //Note: must include because fractional/negative powers will yield uncontrolable results
            } else {
                //Slew
                isCurve = false;
                slewRate = RobotPreferences.getSlewRateLimit();
                if (slewRate < 0) slewRate = 1.0d; //Note: must include because negative rates will yield uncontrolable results
                xLimiter = new SlewRateLimiter(slewRate);
                yLimiter = new SlewRateLimiter(slewRate);
            }
        }

        public GulikitController() {
            controller = new CommandXboxController(0);
            numpad = new CommandJoystick(1);

            binds.put("PigeonReset", controller.button(7)); //Minus
            binds.put("TargetHotspot", controller.button(8)); //Plus
            binds.put("Amp", numpad.button(4));

            configClimberControls();
            configArmControls();
            configIntakeControls();
            configFlywheelControls();
        }

        public void configClimberControls() {
            binds.put("LeftClimberDown", controller.leftTrigger());
            binds.put("LeftClimberUp", controller.button(5)); //Left bumper
            binds.put("RightClimberDown", controller.rightTrigger()); 
            binds.put("RightClimberUp", controller.button(6)); //Right bumper
            binds.put("ClimberMode1", numpad.button(1)); 
            binds.put("ClimberMode2", numpad.button(2));
        }

        public void configArmControls() {
            binds.put("LowerArm", controller.button(6)); //Right bumper
            binds.put("RaiseArm", controller.button(5)); //left bumper
        }

        public void configIntakeControls() {
            binds.put("Intake", controller.button(2)); //A
            binds.put("ManualOuttake", controller.leftTrigger()); 
        }

        public void configFlywheelControls() {
            binds.put("SpinUp", numpad.button(3));
            binds.put("Shoot", controller.button(1)); 
        }

        private double deadbandCompensation(double r) {
            return (r - DEADBAND)/(1 - DEADBAND);
        }

        private double minimumPowerCompensation(double r) {
            return r * (1 - Drive.MinimumDrivePower) + Drive.MinimumDrivePower;
        }

        public TwoDControllerInput getXY() {
            double x = MathUtil.applyDeadband(controller.getRawAxis(1), DEADBAND);
            double y = MathUtil.applyDeadband(controller.getRawAxis(0), DEADBAND);
            double newX, newY = 0.0d;
            if (isCurve) {
                double angle = Math.atan2(y, x);
                double magInital = Math.sqrt(x*x + y*y);
                double magCurved = Math.pow(deadbandCompensation(magInital), curve);
                double powerCompensated = minimumPowerCompensation(magCurved);
                newX = Math.cos(angle) * powerCompensated;
                newY = Math.sin(angle) * powerCompensated;
            } else {
                newX = xLimiter.calculate(x);
                newY = yLimiter.calculate(y);
            }
            if (Double.isNaN(newX)) newX = 0.0d;
            if (Double.isNaN(newY)) newY = 0.0d;
            return new TwoDControllerInput(newX, newY);
        }

        public double getRotation() {
            double deadbandCompensated = deadbandCompensation(
                MathUtil.applyDeadband(controller.getRawAxis(4), DEADBAND));
            if (isCurve) {
                return Math.pow(minimumPowerCompensation(deadbandCompensated), curve);
            } else {
                return minimumPowerCompensation(deadbandCompensated);
            }
        }
    }
}
