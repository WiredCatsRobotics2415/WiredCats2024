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
import edu.wpi.first.wpilibj.DigitalInput;
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
import frc.commands.ShootingPresets;
import frc.robot.Constants;
import frc.robot.Constants.Arm.EncoderOption;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.sim.PhysicsSim;
import frc.utils.Logger;
import frc.utils.Logger.LogLevel;

public class Arm extends SubsystemBase {
    private TalonFX leftMotor;
    private TalonFX rightMotor;
    private AnalogInput input;
    private DigitalInput limitSwitch;
  
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

    private static boolean isCoasting = false;

    public Arm() {
        input = new AnalogInput(RobotMap.Arm.ANALOG_POT_PORT);
        // Constants.Arm.MAX_VOLT = input.getAverageVoltage();
        limitSwitch = new DigitalInput(RobotMap.Arm.LIMITSWITCH_PORT);

        configureMotors();
        configureMechansim2dWidget();

        setGoal(getMeasurement());

        SmartDashboard.putData("Coast", coast());

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(leftMotor, 0.001);
            PhysicsSim.getInstance().addTalonFX(rightMotor, 0.001);

            Constants.Arm.MAX_VOLT = 3.3d; //Necesary in order to prevent division by 0
        }
    }

    private boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    public void configureMechansim2dWidget() {
        Mechanism2d armMechanism2d = new Mechanism2d(3, 3);

        MechanismRoot2d armGoal2d = armMechanism2d.getRoot("armGoal", 1.5, 0.75);
        goalLigament = armGoal2d.append(new MechanismLigament2d("armGoal", 0.75, 0));
        goalLigament.setColor(new Color8Bit(Color.kPaleGreen));

        MechanismRoot2d armPosition2d = armMechanism2d.getRoot("armPosition", 1.5, 0.75);
        positionLigament = armPosition2d.append(new MechanismLigament2d("armPosition", 0.75, 0));
        positionLigament.setColor(new Color8Bit(Color.kGreen));

        Shuffleboard.getTab("Mechanism2d").add("Arm Mechanism", armMechanism2d);
        // SmartDashboard.setDefaultNumber("Arm Goal", getPotRotations());
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

        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
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
        //if (!limitSwitch.get()) leftMotor.setVoltage(voltOut);
        if (!isCoasting) {
            leftMotor.setVoltage(voltOut);
        }
        SmartDashboard.putNumber("Arm Volt out", voltOut);
    }

    /**
     * @return potentiometer value in rotations. Note that the potentiometer is flipped, so this
     *     code is esentially: 1 - pot value + offset
     */
    private double getPotRotations() {
        double measure = Constants.Arm.MAX_ANGLE - (((input.getAverageVoltage() - Constants.Arm.MIN_VOLT) / (Constants.Arm.MAX_VOLT - Constants.Arm.MIN_VOLT))
                                * Constants.Arm.MAX_ANGLE);

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
                            if (goalInDegrees >= Constants.Arm.MAX_ANGLE) {
                                goalInDegrees = Constants.Arm.MAX_ANGLE;
                                return;
                            }
                            goalInDegrees += 0.5;
                            System.out.println("Goal increase: " + goalInDegrees);
                            this.setGoal(goalInDegrees);
                        }));
    }

    /**
     * @return a command that sets the motors to coast mode
     */
    public Command coast() {
        return runOnce(
            () -> {
                rightMotor.setNeutralMode(NeutralModeValue.Coast);
                leftMotor.setNeutralMode(NeutralModeValue.Coast);
                Logger.log(this, LogLevel.INFO, "Motors put into COAST mode");
            }   
        );
    }

    /**
    * Sets the motors to brake mode
    */
    public void brake() {
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        Logger.log(this, LogLevel.INFO, "Motors put into BRAKE mode");
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
                            goalInDegrees -= 0.5;
                            System.out.println("Goal decrease: " + goalInDegrees);
                            this.setGoal(goalInDegrees);
                        }));
    }

    public Command moveDown() {
        return new InstantCommand(
            () -> this.setGoal(0)
        );
    }

    public Command moveUp() {
        return new InstantCommand(
            () -> this.setGoal(ShootingPresets.Settings.field.bottom)
        );
    }

    /**
     * @return A command to set the arm's current goal to the calculated Arm Angle based on distance to subwoofer
     */
    public Command moveToShotAngle() {
        return new InstantCommand(
            () -> {
                System.out.println(Constants.ShooterCalculations.getCalculatedArmShooterAngle());
                //this.setGoal(Constants.ShooterCalculations.getCalculatedArmShooterAngle());
            });
    } 

    public boolean withinSetGoalTolerance() {
        return goalInDegrees > getMeasurement()-3 && goalInDegrees < getMeasurement()+3;
    }

    /**
     * When the arm is hitting the limit switch (on the left bottom hard stop),
     * set the maximum voltage of the potentiometer to the current voltage
    /* 
    */ 
    private void resetPotentiometerIfAtZero(){
      if (getLimitSwitch()) {
        double oldMaxVolt = Constants.Arm.MAX_VOLT; 
        Constants.Arm.MAX_VOLT = input.getAverageVoltage(); 
        double oldMinVolt = Constants.Arm.MIN_VOLT; 
        Constants.Arm.MIN_VOLT = (Constants.Arm.MAX_VOLT - oldMaxVolt) + oldMinVolt; 
      } 
    }

    /**
     * Sets the min and max volt back to what they started as on boot.
     * From Constants.Arm.MAX_VOLT_OG and MIN_VOLT_OG
     */
    public void resetPotentiometerAndArm() {
        Constants.Arm.MAX_VOLT = Constants.Arm.MAX_VOLT_OG;
        Constants.Arm.MIN_VOLT = Constants.Arm.MIN_VOLT_OG;
    }

    public InstantCommand changeCoast = new InstantCommand(() -> {
        if (isCoasting == false) {
            isCoasting = true;
        } else {
            isCoasting = false;
        }
    });

    @Override
    public void periodic() {
        double measurement = getMeasurement();
        SmartDashboard.putNumber("Arm Measurement", measurement);
        SmartDashboard.putNumber("Arm Voltage", input.getAverageVoltage()); 
        SmartDashboard.putData("coast", changeCoast);
        useOutput(pid.calculate(measurement), pid.getSetpoint());
        positionLigament.setAngle(measurement);
        goalLigament.setAngle(goalInDegrees);

        resetPotentiometerIfAtZero();
        SmartDashboard.putBoolean("Limit Switch", getLimitSwitch());

        SmartDashboard.putNumber("goalindegrees", goalInDegrees);

        // control arm with smartdashboard
        //double desiredAngle = SmartDashboard.getNumber("Arm Goal", getMeasurement());
        //setGoal(desiredAngle);

        //System.out.println(limitSwitch.get());
    }
}
