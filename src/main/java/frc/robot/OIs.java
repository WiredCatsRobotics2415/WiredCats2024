package frc.robot;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
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
            binds = Map.of(
                "PigeonReset", controller.button(7), // Minus
                "FixAll", controller.button(8), // Plus
                "Intake", controller.button(2), // A
                "ReleaseClimber", controller.button(3), // Y
                "RetractClimber", controller.button(1), // B 
                "FlywheelOn", numpad.button(7, Robot.getEventLoop()), // 7 
                "FlywheelOff", numpad.button(8, Robot.getEventLoop()), // 8
                "ManualOuttake", controller.button(4) // X
            );
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
