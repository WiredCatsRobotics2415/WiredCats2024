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

    public Finger() {
        motor = new CANSparkMax(RobotMap.Finger.FINGER_MOTOR, CANSparkMax.MotorType.kBrushless); // initialize motor
        configureMotor();
        relativeEncoder.setPosition(0);
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

        // set PID coefficients
        //pidController.setFF(Constants.Finger.Ks);
        pidController.setP(Constants.Finger.Kp);
        pidController.setD(Constants.Finger.Kd);
        pidController.setOutputRange(-Constants.Finger.outputExtrema, Constants.Finger.outputExtrema);
    }

    /**
     * @param position rotation number to run the finger, relative to where it is
     * @return Command that runs finger a certain number of rotations. 
     */
    public Command run(double position) {
        return new InstantCommand(() -> {
            double toMove = getPosition() + (position * Constants.Finger.FINGER_GEAR_RATIO);
            pidController.setReference(toMove, CANSparkMax.ControlType.kPosition);
            Logger.log(this, LogLevel.INFO, "Finger set to  " + toMove);
        });
    }

    /**
     * @return Command that moves the finger to interfere with the note path.
     * intended to be used to prevent note from contacting flywheels
     */
    public Command reverse() {
        return run(-0.75d);
    }

    /**
     * @return Command that shoots a note that has been intook, then moves back to its start position
     * (explained in {@link shootPreloadedAndGoToStartPosition})
     */
    public Command shootInTeleOp() {
        return run(1.375)
        .andThen(new WaitCommand(0.25))
        .andThen(run(0.625d));
    }

    /**
     * @return Command that fires note by running the finger 1 full rotation.
     */ 
    public Command fire() {
        return new InstantCommand(() -> {
            pidController.setReference(getPosition() + (Constants.Finger.DISTANCE * Constants.Finger.FINGER_GEAR_RATIO), CANSparkMax.ControlType.kPosition);
            System.out.println("finger");
        });
    }

    /**
     * @return current position of the finger. 
     */
    public double getPosition() {
        return relativeEncoder.getPosition();
    }

    // @Override
    // public void periodic() {
    //     System.out.println(getPosition());
    // }
}
