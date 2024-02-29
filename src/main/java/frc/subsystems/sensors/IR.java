package frc.subsystems.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.RobotMap;

/*
 * This class is for the IR sensor
 */
public class IR {
    public AnalogInput rightIR;
    public AnalogInput leftIR;

    public IR() {
        leftIR = new AnalogInput(RobotMap.Intake.TOP_IR);
        rightIR = new AnalogInput(RobotMap.Intake.BOTTOM_IR);
    }
}
