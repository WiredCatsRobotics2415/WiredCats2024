package frc.robot;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.utils.Logger;
import frc.utils.Logger.LogLevel;
import frc.utils.RobotPreferences;

public class OIs {
    public static abstract class OI {
        static SendableChooser<Integer> oiChooser;
        static {
            oiChooser = new SendableChooser<Integer>();
            oiChooser.setDefaultOption("Gulikit Controller", 0);
            SmartDashboard.putData("OI", oiChooser);
        }

        //RECORDS
        public static record TwoDControllerInput(double x, double y) {};

        //PROPERTIES
        public double DEADBAND;

        public Map<String, JoystickButton> binds;

        //JOYSTICKS
        public abstract TwoDControllerInput getXY();

        public abstract double getRotation();

        //UTILS
        public abstract void setPreferences();
    }
    
    public static class GulikitController extends OI {
        XboxController controller;

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
                Logger.log(LogLevel.INFO, "OI: Using Curve at " + curve + " power.");
            } else {
                //Slew
                isCurve = false;
                slewRate = RobotPreferences.getSlewRateLimit();
                if (slewRate < 0) slewRate = 1.0d; //Note: must include because negative rates will yield uncontrolable results
                xLimiter = new SlewRateLimiter(slewRate);
                yLimiter = new SlewRateLimiter(slewRate);
                Logger.log(LogLevel.INFO, "OI: Using Slew at " + slewRate + " rate.");
            }
        }

        public GulikitController() {
            controller = new XboxController(0);
            binds = Map.of(
                "navX Reset", new JoystickButton(controller, 7) //Minus
            );
        }

        public TwoDControllerInput getXY() {
            double x = MathUtil.applyDeadband(controller.getRawAxis(1), DEADBAND);
            double y = MathUtil.applyDeadband(controller.getRawAxis(0), DEADBAND);
            double newX, newY;
            if (isCurve) {
                double angle = Math.atan(y/x);
                double magInital = Math.sqrt(x*x + y*y);
                double magCurved = Math.pow(magInital, curve);
                newX = Math.cos(angle) * magCurved;
                newY = Math.sin(angle) * magCurved;
            } else {
                newX = xLimiter.calculate(x);
                newY = yLimiter.calculate(y);
            }
            return new TwoDControllerInput(newX, newY);
        }

        public double getRotation() { 
            if (isCurve) {
                return Math.pow(MathUtil.applyDeadband(controller.getRawAxis(4), DEADBAND), curve);
            } else {
                return MathUtil.applyDeadband(controller.getRawAxis(4), DEADBAND);
            }
        }
    }
}
