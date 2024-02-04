package frc.robot;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

        public Map<String, Trigger> binds;

        //JOYSTICKS
        public abstract TwoDControllerInput getXY();

        public abstract double getRotation();

        //UTILS
        public abstract void setPreferences();
    }
    
    public static class GulikitController extends OI {
        CommandXboxController controller;

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
            binds = Map.of(
                "navX Reset", controller.button(7) //Minus
            );
        }

        public TwoDControllerInput getXY() {
            double x = MathUtil.applyDeadband(controller.getRawAxis(1), DEADBAND);
            double y = MathUtil.applyDeadband(controller.getRawAxis(0), DEADBAND);
            double newX, newY = 0.0d;
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
            if (x < 0) newX*=-1;
            if (Double.isNaN(newX)) newX = 0.0d;
            if (Double.isNaN(newY)) newY = 0.0d;
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
