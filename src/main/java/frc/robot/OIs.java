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
import frc.robot.Constants.DriverControl;
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

            binds.put("PigeonReset", controller.button(7, Robot.buttonEventLoop)); //Minus
            // binds.put("TargetHotspot", controller.button(8, Robot.buttonEventLoop)); //Plus (in use by finger reverse)

            // Climber
            binds.put("LeftClimberDown", controller.leftTrigger());
            binds.put("LeftClimberUp", controller.button(5, Robot.buttonEventLoop)); //Left bumper
            binds.put("RightClimberDown", controller.axisLessThan(3, 0.5)); //right trigger, 1 at rest and 0 when pressed
            binds.put("RightClimberUp", controller.button(6, Robot.buttonEventLoop)); //Right bumper
            binds.put("ClimberMode1", numpad.button(1, Robot.buttonEventLoop));
            binds.put("ClimberMode2", numpad.button(2, Robot.buttonEventLoop));
            
            // Arm
            binds.put("LowerArm", controller.button(5, Robot.buttonEventLoop)); //left bumper
            binds.put("RaiseArm", controller.button(6, Robot.buttonEventLoop)); //right bumper

            // Intake 
            binds.put("Intake", controller.button(2, Robot.buttonEventLoop)); //A
            binds.put("ManualOuttake", controller.leftTrigger());
            binds.put("Manual Intake", numpad.button(8, Robot.buttonEventLoop));
            
            // Flywheel (testing, using controller)
            // binds.put("SpinUp", controller.button(3, Robot.buttonEventLoop)); // Y
            // binds.put("SpinOff", controller.button(4, Robot.buttonEventLoop)); // X

            // Flywheel (competition controls, using numpad)
            //binds.put("SpinUpToShoot", numpad.button(7, Robot.buttonEventLoop));
            binds.put("SpinOff", numpad.button(8, Robot.buttonEventLoop));
            binds.put("SpinUpToAmp", numpad.button(9, Robot.buttonEventLoop));

            binds.put("Shoot", controller.axisLessThan(3, 0.5)); //right trigger, 1 at rest and 0 when pressed
            binds.put("ReverseFinger", controller.button(8, Robot.buttonEventLoop));

            binds.put("ShuttleRotate", controller.button(3, Robot.buttonEventLoop)); //Y

            // Presets 
            binds.put("Amp", numpad.button(4, Robot.buttonEventLoop));
            binds.put("ArmDrivePreset", numpad.button(5, Robot.buttonEventLoop));
            binds.put("ArmIntakePosition", numpad.button(6, Robot.buttonEventLoop));
            binds.put("ShootClose", numpad.button(7, Robot.buttonEventLoop)); // Subwoofer

            binds.put("AutoIntake", controller.button(1, Robot.buttonEventLoop)); //B
            binds.put("FixAll", numpad.button(1, Robot.buttonEventLoop)); 
            binds.put("ArmAngle", numpad.button(2, Robot.buttonEventLoop)); 

            binds.put("FixArm", numpad.button(3, Robot.buttonEventLoop));
        }

        private double deadbandCompensation(double r) {
            return (r - DEADBAND)/(1 - DEADBAND);
        }

        private double minimumPowerCompensation(double r) {
            return r * (1 - DriverControl.MinimumDrivePower) + DriverControl.MinimumDrivePower;
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
