package frc.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.generated.TunerConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.sim.PhysicsSim;
import frc.utils.Logger;
import frc.utils.Logger.LogLevel;

public class Flywheel extends SubsystemBase {
    private enum TestType {
        LOCK,
        RATIO
    }

    private double leftSpeedRatio = 1.0d;
    private double leftSetRPM = 0d;

    private TestType currentMode = TestType.LOCK;

    // Intialize flywheel motors
    private TalonFX left;
    private TalonFX right;
    // Initialize velocity feedforward
    private VelocityVoltage m_voltageVelocity;
    private final SendableChooser<TestType> testChooser = new SendableChooser<TestType>();

    private static Flywheel instance;

    private MechanismLigament2d leftGoal;
    private MechanismLigament2d rightGoal;

    private boolean shouldSpinUp = false;

    private Flywheel() {
        left = new TalonFX(RobotMap.Flywheel.LEFT_FLYWHEEL);
        right = new TalonFX(RobotMap.Flywheel.RIGHT_FLYWHEEL);
        BaseStatusSignal.setUpdateFrequencyForAll(50, left.getRotorVelocity(), right.getRotorVelocity());
        m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
        configMotors();
        configSmartDashboard();
        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(left, 0.001);
            PhysicsSim.getInstance().addTalonFX(right, 0.001);
        }
    }

    /**
     * Changes the test mode, between Lock and Ratio, from the SendableChooser.
     * Intended to be called in teleopInit.
     */
    public void teleopInit() {
        currentMode = testChooser.getSelected();

        if (currentMode.equals(TestType.LOCK)) {
            // left.setControl(new StaticBrake());
        }
    }

    public static Flywheel getInstance() {
        if (instance == null) {
            return new Flywheel();
        }
        return instance;
    }

    private void configSmartDashboard() {
        testChooser.setDefaultOption(TestType.LOCK.toString(), TestType.LOCK);
        testChooser.addOption(TestType.RATIO.toString(), TestType.RATIO);
        SmartDashboard.setDefaultNumber("Left:Right Ratio", leftSpeedRatio);
        SmartDashboard.setDefaultNumber("Set Speed (Left Motor - RPM)", leftSetRPM);
        SmartDashboard.setDefaultNumber("Current Speed (Left)", left.getRotorVelocity().getValueAsDouble());
        SmartDashboard.setDefaultNumber("Current Speed (Right)", right.getRotorVelocity().getValueAsDouble());

        Mechanism2d flywheelMech2d = new Mechanism2d(3, 3);

        MechanismRoot2d leftGoalRoot = flywheelMech2d.getRoot("leftGoal", 1.4, 0.25);
        leftGoal = leftGoalRoot.append(new MechanismLigament2d("leftGoal", 0.1, 0));
        MechanismRoot2d rightGoalRoot = flywheelMech2d.getRoot("rightGoal", 1.6, 0.25);
        rightGoal = rightGoalRoot.append(new MechanismLigament2d("rightGoal", 0.1, 0));

        Shuffleboard.getTab("Mechanism2d").add("Flywheel Mechanism", flywheelMech2d);
    }

    private void configMotors() {
        TalonFXConfigurator rightCfg = right.getConfigurator();
        rightCfg.apply(Constants.Flywheel.RIGHT_PID);
        rightCfg.apply(Constants.Flywheel.COAST_CONFIG);
        rightCfg.apply(Constants.Flywheel.CURRENT_LIMITS);
        right.setInverted(true);

        TalonFXConfigurator leftCfg = left.getConfigurator();
        leftCfg.apply(Constants.Flywheel.LEFT_PID);
        leftCfg.apply(Constants.Flywheel.COAST_CONFIG);
        leftCfg.apply(Constants.Flywheel.CURRENT_LIMITS);
        left.setInverted(false);
    }

    /**
     * @return Command that spins up each motor to the specified RPM.
     */
    public Command on(double leftSpeed, double rightSpeed) {
        return runOnce(
                () -> {
                    Logger.log(this, LogLevel.INFO, "Flywheel on", leftSpeed, rightSpeed);
                    left.setControl(m_voltageVelocity.withVelocity(Constants.Flywheel.rpmToRPS(leftSpeed)));
                    right.setControl(m_voltageVelocity.withVelocity(Constants.Flywheel.rpmToRPS(rightSpeed)));
                });
    }

    /**
     * @return Command that calls on with speeds set from SmartDashboard.
     */
    public Command onFromSmartDashboard() {
        return runOnce(
                () -> {
                    Logger.log(this, LogLevel.INFO, "Flywheel onFromSmartDashboard");
                    leftSpeedRatio = SmartDashboard.getNumber("Left:Right Ratio", leftSpeedRatio);
                    leftSetRPM = SmartDashboard.getNumber("Set Speed (Left Motor - RPM)", leftSetRPM);
                    on(leftSetRPM, leftSetRPM / leftSpeedRatio).schedule();
                });
    }

    /**
     * @return Command that sets both motor's RPM to 0.
     *         Note that the flywheels is in coast.
     */
    public Command off() {
        return runOnce(
                () -> {
                    Logger.log(this, LogLevel.INFO, "Flywheel off");
                    left.setControl(m_voltageVelocity.withVelocity(0));
                    right.setControl(m_voltageVelocity.withVelocity(0));
                });
    }

    /**
     * True if the current speed of the left shooter motor is within + or -
     * GOAL_TOLERANCE_RPM
     */
    public boolean withinSetGoal() {
        double currentValue = Constants.Flywheel.rpsToRPM(left.getRotorVelocity().getValue());
        if (shouldSpinUp) {
            double goalValue = Constants.Flywheel.rpmToRPS(leftSetRPM);
            return currentValue < (goalValue + Constants.Flywheel.GOAL_TOLERANCE_RPM) ||
                    currentValue > (goalValue - Constants.Flywheel.GOAL_TOLERANCE_RPM);
        } else {
            return currentValue < Constants.Flywheel.GOAL_TOLERANCE_RPM ||
                    currentValue > -Constants.Flywheel.GOAL_TOLERANCE_RPM;
        }
    }

    @Override
    public void periodic() {
        // Display current speed of both motors
        double leftSpeedRaw = left.getRotorVelocity().getValueAsDouble();
        double rightSpeedRaw = right.getRotorVelocity().getValueAsDouble();
        
        SmartDashboard.putNumber("Current Motor Speed (Left - RPM)", leftSpeedRaw);
        SmartDashboard.putNumber("Current Motor Speed (Right - RPM)", rightSpeedRaw);

        SmartDashboard.putNumber("Current Wheel Speed (Left - RPM)", Constants.Flywheel.rpsToRPM(leftSpeedRaw));
        SmartDashboard.putNumber("Current Wheel Speed (Right - RPM)", Constants.Flywheel.rpsToRPM(rightSpeedRaw));

        rightGoal.setColor(getColorForRPM(Constants.Flywheel.rpsToRPM(rightSpeedRaw)));
        leftGoal.setColor(getColorForRPM(Constants.Flywheel.rpsToRPM(leftSpeedRaw)));
    }

    private Color8Bit getColorForRPM(double rpm) {
        return new Color8Bit((int) ((rpm / 6380) * 255), 0, 0);
    }
}