package frc.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Flywheel extends SubsystemBase {
    private enum TestType {
        LOCK,
        RATIO
    }

    private TestType currentMode = TestType.LOCK;

    public TalonFX right;
    public TalonFX left;

    private DutyCycleOut rightOut = new DutyCycleOut(0);
    private DutyCycleOut leftOut = new DutyCycleOut(0);

    private final SendableChooser<TestType> testChooser = new SendableChooser<TestType>();
    private double leftSpeedRatio = 0.67d;
    private double rightSetRPM = 0d;

    private boolean shouldSpinUp = false;

    private static Flywheel instance;

    public Flywheel() {
        right = new TalonFX(RobotMap.Flywheel.RIGHT_FLYWHEEL);
        left = new TalonFX(RobotMap.Flywheel.LEFT_FLYWHEEL);

        testChooser.setDefaultOption(TestType.LOCK.toString(), TestType.LOCK);
        testChooser.addOption(TestType.RATIO.toString(), TestType.RATIO);

        SmartDashboard.setDefaultNumber("Left ratio", leftSpeedRatio);
        SmartDashboard.setDefaultNumber("Set speed", rightSetRPM);

        configMotors();
    }

    public static Flywheel getInstance() {
        if (instance == null) {
            instance = new Flywheel();
        }
        return instance;
    }

    private void configMotors() {
        TalonFXConfigurator rightCfg = right.getConfigurator();
        rightCfg.apply(Constants.Flywheel.RIGHT_PID);
        rightCfg.apply(Constants.Flywheel.COAST_CONFIG);
        right.setInverted(true);

        TalonFXConfigurator leftCfg = left.getConfigurator();
        leftCfg.apply(Constants.Flywheel.LEFT_PID);
        leftCfg.apply(Constants.Flywheel.COAST_CONFIG);
        left.setInverted(false);
    }

    /**
     * Changes the test mode, between Lock and Ratio, from the SendableChooser.
     * Intended to be called in teleopInit.
     */
    public void teleopInit() {
        currentMode = testChooser.getSelected();

        if (currentMode.equals(TestType.LOCK)) {
            //left.setControl(new StaticBrake());
        }
    }

    /**
     * @return Command that toggles whether or not the flywheel should spin up to its goal RPM.
     * If already spinned up, spin down to 0.
     * Otherwise, spin up to the goal.
     */
    public Command toggleSpinedUp() {
        return runOnce(() -> {
            System.out.println("Flywheel toggle " + (!shouldSpinUp ? "on" : "off"));
            shouldSpinUp = !shouldSpinUp;
        });
    }

    /**
     * @return true if the current speed of the left shooter motor is within + or - GOAL_TOLERANCE_RPM
     */
    public boolean withinGoal() {
        double currentValue = Constants.Flywheel.falconToRPM(left.getRotorVelocity().getValue());
        if (shouldSpinUp) {
            double goalValue = Constants.Flywheel.rpmToFalcon(rightSetRPM);
            return currentValue < (goalValue + Constants.Flywheel.GOAL_TOLERANCE_RPM) ||
                currentValue > (goalValue - Constants.Flywheel.GOAL_TOLERANCE_RPM);
        } else {
            return currentValue < Constants.Flywheel.GOAL_TOLERANCE_RPM ||
                currentValue > -Constants.Flywheel.GOAL_TOLERANCE_RPM;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(testChooser);
        leftSpeedRatio = SmartDashboard.getNumber("Recessive ratio", leftSpeedRatio);
        rightSetRPM = SmartDashboard.getNumber("Set speed", rightSetRPM);

        if (shouldSpinUp) {
            right.setControl(rightOut.withOutput(0.8));
            left.setControl(leftOut.withOutput(0.8));
        } else {
            right.setControl(rightOut.withOutput(Constants.Flywheel.rpmToFalcon(0.0d)));
            left.setControl(leftOut.withOutput(Constants.Flywheel.rpmToFalcon(0.0d)));
        }

        //SmartDashboard.putBoolean("Flywheel Within Goal", withinGoal());
    }
}
