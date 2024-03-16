package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.utils.Logger;
import frc.utils.Logger.LogLevel;

public class Finger extends SubsystemBase{
    private CANSparkMax motor; 
    private SparkPIDController pidController;
    private RelativeEncoder relativeEncoder;
    private static Finger instance; 
    private double offset; 

    public Finger() {
        motor = new CANSparkMax(RobotMap.Finger.FINGER_MOTOR, CANSparkMax.MotorType.kBrushless); // initialize motor
        configureMotor();
        relativeEncoder.setPosition(0);
        offset = 0; 
    }

    public static Finger getInstance() {
        if (instance == null) {
            instance = new Finger();
        }
        return instance;
    }

    public void configureMotor() {
        motor.restoreFactoryDefaults();

        // motor.setInverted(true);
        motor.setSmartCurrentLimit(30);

        relativeEncoder = motor.getEncoder();
        pidController = motor.getPIDController();

        relativeEncoder.setPositionConversionFactor(1/Constants.Finger.FINGER_GEAR_RATIO);

        // set PID coefficients
        //pidController.setFF(Constants.Finger.Ks);
        pidController.setP(Constants.Finger.Kp);
        pidController.setD(Constants.Finger.Kd);
        pidController.setOutputRange(-Constants.Finger.outputExtrema, Constants.Finger.outputExtrema);
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
            run(Constants.Finger.DISTANCE + old_offset).schedule();
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
    }
}
