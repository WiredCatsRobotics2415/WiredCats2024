package frc.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.Arm.EncoderOption;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.sim.PhysicsSim;

public class Arm extends SubsystemBase {
    private TalonFX leftMotor;
    private TalonFX rightMotor;
    private AnalogInput input;
    private ArmFeedforward ff =
            new ArmFeedforward(Constants.Arm.KS, Constants.Arm.KG, Constants.Arm.KV, Constants.Arm.KA);
    private ProfiledPIDController pid =
            new ProfiledPIDController(
                    Constants.Arm.KP,
                    0,
                    Constants.Arm.KD,
                    new TrapezoidProfile.Constraints(
                            Constants.Arm.VELO_MAX, Constants.Arm.ACCEL_MAX));

    private MechanismLigament2d goalLigament;
    private MechanismLigament2d positionLigament;

    private double goalInDegrees = Constants.Arm.MIN_DEGREES;
    private static Arm instance;

    public Arm() {
        input = new AnalogInput(RobotMap.Arm.ANALOG_POT_PORT);
        Constants.Arm.MAX_VOLT = input.getAverageVoltage();

        configureMotors();
        configureMechansim2dWidget();

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(leftMotor, 0.001);
            PhysicsSim.getInstance().addTalonFX(rightMotor, 0.001);

            Constants.Arm.MAX_VOLT = 3.3d; //Necesary in order to prevent division by 0
        }
    }

    public void configureMechansim2dWidget() {
        Mechanism2d armMechanism2d = new Mechanism2d(3, 3);

        MechanismRoot2d armGoal2d = armMechanism2d.getRoot("armGoal", 1.5, 0);
        goalLigament = armGoal2d.append(new MechanismLigament2d("armGoal", 0.75, 0));
        goalLigament.setColor(new Color8Bit(Color.kPaleGreen));

        MechanismRoot2d armPosition2d = armMechanism2d.getRoot("armPosition", 1.5, 0);
        positionLigament = armPosition2d.append(new MechanismLigament2d("armPosition", 0.75, 0));
        positionLigament.setColor(new Color8Bit(Color.kGreen));

        Shuffleboard.getTab("Mechanism2d").add("Arm Mechanism", armMechanism2d);
        SmartDashboard.setDefaultNumber("Arm Goal", getPotRotations());
    }

    /** Configures the Arm's motors. The right motor is inverted and follows the left motor. */
    public void configureMotors() {
        FeedbackConfigs feedbackConfigs =
                new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

        leftMotor = new TalonFX(RobotMap.Arm.LEFT_MOTOR_PORT);
        leftMotor.getConfigurator().apply(feedbackConfigs);
        leftMotor.setInverted(true);

        rightMotor = new TalonFX(RobotMap.Arm.RIGHT_MOTOR_PORT);
        rightMotor.setControl(new StrictFollower(leftMotor.getDeviceID()));
        rightMotor.setInverted(true);

        leftMotor.setNeutralMode(NeutralModeValue.Coast); // TODO: Change to Brake
        rightMotor.setNeutralMode(NeutralModeValue.Coast); // TODO: Change to Brake
    }

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    /**
     * Sets the left arm's motor to the desired voltage, calculated by the feedforward object and
     * PID subsystem.
     *
     * @param output the output of the ProfiledPIDController
     * @param setpoint the setpoint state of the ProfiledPIDController, for feedforward
     */
    private void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = ff.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output
        double voltOut = output + feedforward;
        leftMotor.setVoltage(voltOut);
        SmartDashboard.putNumber("Arm Volt out", voltOut);
    }

    /**
     * @return potentiometer value in rotations. Note that the potentiometer is flipped, so this
     *     code is esentially: 1 - pot value + offset
     */
    private double getPotRotations() {
        double measure =
                Constants.Arm.MAX_ANGLE
                        - ((input.getAverageVoltage() / Constants.Arm.MAX_VOLT)
                                * Constants.Arm.MAX_ANGLE);

        if (measure < 1) {
            measure = 0.0d;
        }

        return measure;
    }

    /**
     * @return measurement in rotations. Handles selection of encoder defined in Constants. If in
     *     simulation, use the analog port window to control the simulated position.
     */
    public double getMeasurement() {
        if (Robot.isSimulation()) {
            return getPotRotations();
        }

        double rotations = 0.0d;
        if (Constants.Arm.ENCODER_TO_USE.equals(EncoderOption.ANALOG_POT)) {
            rotations = getPotRotations();
        } else {
            rotations = Constants.Arm.falconToRotations(leftMotor.getPosition().getValue());
        }
        return rotations;
    }

    /**
     * Sets the goal of the arm in degrees. Does NOT account for the goal being a value outside of
     * the minimum or maximum degrees.
     *
     * @param goalInRotations
     */
    public void setGoal(double goalInDegrees) {
        this.goalInDegrees = goalInDegrees;
        pid.setGoal(new TrapezoidProfile.State(goalInDegrees, 0));
    }

    /**
     * @return A command to increase the arm's current goal by one degree. Does not go above max
     *     rotations defined in constants.
     */
    public Command increaseGoal() {
        return new RepeatCommand(
                new InstantCommand(
                        () -> {
                            if (goalInDegrees >= Constants.Arm.MAX_DEGREES) {
                                goalInDegrees = Constants.Arm.MAX_DEGREES;
                                return;
                            }
                            goalInDegrees += 1;
                            System.out.println("Goal increase: " + goalInDegrees);
                            this.setGoal(goalInDegrees);
                        }));
    }

    /**
     * @return A command to decrease the arm's current goal by one degree. Does not go below min
     *     rotations defined in constants.
     */
    public Command decreaseGoal() {
        return new RepeatCommand(
                new InstantCommand(
                        () -> {
                            if (goalInDegrees <= Constants.Arm.MIN_DEGREES) {
                                goalInDegrees = Constants.Arm.MIN_DEGREES;
                                return;
                            }
                            goalInDegrees -= (0.25 / 360.0d);
                            System.out.println("Goal decrease: " + goalInDegrees);
                            this.setGoal(goalInDegrees);
                        }));
    }

    @Override
    public void periodic() {
        double measurement = getMeasurement();
        SmartDashboard.putNumber("Arm Measurement", measurement);
        useOutput(pid.calculate(measurement), pid.getSetpoint());
        positionLigament.setAngle(measurement);
        goalLigament.setAngle(goalInDegrees);

        // control arm with smartdashboard
        double desiredAngle = SmartDashboard.getNumber("Arm Goal", 0.0d);
        if (desiredAngle != 0.0d) {
            setGoal(desiredAngle);
        }
    }
}
