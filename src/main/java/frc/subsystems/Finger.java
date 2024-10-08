package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.utils.Logger;
import frc.utils.Logger.LogLevel;
import com.revrobotics.SparkMaxAlternateEncoder;


public class Finger extends SubsystemBase{
    private CANSparkMax motor; 
    private SparkPIDController pidController;
    //private RelativeEncoder relativeEncoder;
    private RelativeEncoder relativeEncoder;
    //private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
    //private static final int kCPR = 8192;
    private static Finger instance; 
    private double offset;
    
    private MechanismLigament2d goalLigament;
    private MechanismLigament2d positionLigament;

    public Finger() {
        motor = new CANSparkMax(RobotMap.Finger.FINGER_MOTOR, CANSparkMax.MotorType.kBrushless); // initialize motor
        configureMotor();
        configureMechansim2dWidget();
        relativeEncoder.setPosition(0);
        offset = 0; 
        pidController.setReference(0, ControlType.kPosition);
    }

    public static Finger getInstance() {
        if (instance == null) {
            instance = new Finger();
        }
        return instance;
    }

    public void configureMechansim2dWidget() {
        Mechanism2d fingerMechanism2d = new Mechanism2d(3, 3);

        MechanismRoot2d fingerGoal2d = fingerMechanism2d.getRoot("fingerGoal", 1.5, 1.5);
        goalLigament = fingerGoal2d.append(new MechanismLigament2d("fingerGoal", 0.5, 270));
        goalLigament.setColor(new Color8Bit(Color.kCadetBlue));

        MechanismRoot2d fingerPosition2d = fingerMechanism2d.getRoot("fingerPosition", 1.5, 1.5);
        positionLigament = fingerPosition2d.append(new MechanismLigament2d("fingerPosition", 0.5, 270));
        positionLigament.setColor(new Color8Bit(Color.kGreen));

        Shuffleboard.getTab("Mechanism2d").add("Finger Mechanism", fingerMechanism2d);
    }

    public void configureMotor() {
        motor.restoreFactoryDefaults();

        // motor.setInverted(true);
        motor.setSmartCurrentLimit(30);

        relativeEncoder = motor.getEncoder();
        //relativeEncoder = motor.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4096);
        //relativeEncoder = motor.getAlternateEncoder(kAltEncType, kCPR);
        pidController = motor.getPIDController();
        pidController.setFeedbackDevice(relativeEncoder);

        relativeEncoder.setPositionConversionFactor(1/Constants.Finger.FINGER_GEAR_RATIO);

        // set PID coefficients
        pidController.setFF(Constants.Finger.Ks);
        pidController.setP(Constants.Finger.Kp);
        pidController.setD(Constants.Finger.Kd);
        pidController.setOutputRange(-Constants.Finger.outputExtrema, Constants.Finger.outputExtrema);
        motor.setIdleMode(IdleMode.kBrake);
        //motor.setClosedLoopRampRate(0.5); //slows down finger
        // pidController.setPositionPIDWrappingEnabled(true);
        // pidController.setPositionPIDWrappingMaxInput(1);
        // pidController.setPositionPIDWrappingMinInput(-1);
    }

    /**
     * @param position rotation number to run the finger, relative to where it is
     * @return Command that runs finger a certain number of rotations. 
     */
    public Command run(double position) {
        return new InstantCommand(() -> {
            //double toMove = getPosition() + (position * Constants.Finger.FINGER_GEAR_RATIO);
            pidController.setReference(position, CANSparkMax.ControlType.kPosition);
            Logger.log(this, LogLevel.INFO, "Finger set to  " + getPosition() + position);
            goalLigament.setAngle((position*360)%360);
        });
    }

    /**
     * @return Command that moves the finger to interfere with the note path.
     * intended to be used to prevent note from contacting flywheels
     */
    public Command reverse() {
        return new InstantCommand(() -> {
            offset += 0.05; 
            run(getPosition() - 0.05d).schedule();
        }); 
    }

    /**
     * @return Command that fires note by running the finger 1 full rotation.
     */ 
    public Command fire() {
        return new InstantCommand(() -> {
            double old_offset = offset; 
            old_offset -= (1/120.0d)*(offset/0.05d);
            offset = 0;
            relativeEncoder.setPosition(0);
            Logger.log(this, LogLevel.INFO, old_offset);
            Intake intake = Intake.getInstance();
            run(Constants.Finger.DISTANCE + old_offset)
                 .andThen(new WaitCommand(0.5)
                 .andThen(run(0))).schedule();
            //run(1 + old_offset).schedule();
        });
    }

    /**
     * @return current position of the finger. 
     */
    public double getPosition() {
        //if (raw >= 0.99) return 1;
        //if (raw <= -0.99) return -1;
        return relativeEncoder.getPosition();
    }

    @Override
    public void periodic() {
        //System.out.println("Position: " + relativeEncoder.getPosition());
        /* 
       if(getPosition()  >= 1){
        relativeEncoder.setPosition(0);
        run(getPosition()).schedule();
        Logger.log(this, LogLevel.INFO, "go above 1");
       } else if(getPosition() <= -1){
        relativeEncoder.setPosition(0);
        run(getPosition()).schedule();
        Logger.log(this, LogLevel.INFO, "go below -1");
       }
       Logger.log(this, LogLevel.INFO, getPosition());
       */
      positionLigament.setAngle((relativeEncoder.getPosition()*360)%360);
    }
}
