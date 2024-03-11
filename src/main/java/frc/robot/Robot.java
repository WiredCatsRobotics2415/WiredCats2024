// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.sim.PhysicsSim;

public class Robot extends TimedRobot {
    private RobotContainer robot;
    private Command auto;
    private PowerDistribution pdp;

    /**
     * The BUTTON ONLY event loop for the robot. Gets polled during robotPeriodic. Intended for use
     * in {@link OIs}'s binds maps, and is cleared before {@link RobotContainer}'s
     * configureButtonBinds is called.
     */
    public static final EventLoop buttonEventLoop = new EventLoop();

    @Override
    public void robotInit() {
        pdp = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
        robot = RobotContainer.getInstance();
    }

    @Override
    public void teleopInit() {
        robot.teleopInit();
    }

    @Override
    public void autonomousInit() {
        auto = robot.getAutonomousCommand(); // Run an auto path
        // auto = robot.getPathfindingCommand(); // Run a pathfinding command
        if (auto != null) CommandScheduler.getInstance().schedule(auto);
    }

    @Override
    public void autonomousExit() {
        if (auto != null) auto.cancel();
    }

    @Override
    public void robotPeriodic() {
        buttonEventLoop.poll();
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Voltage", pdp.getVoltage());
    }

    @Override
    public void simulationPeriodic() {
        PhysicsSim.getInstance().run();
    }
}
