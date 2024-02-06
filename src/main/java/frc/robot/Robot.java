// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private RobotContainer robot;
  private Command auto;

  @Override
  public void robotInit() {
    robot = RobotContainer.getInstance();
  }

  @Override
  public void teleopInit() {
    robot.teleopInit();
  }

  @Override
  public void autonomousInit() {
    // auto = robot.getAutonomousCommand(); // Run an auto path
    auto = robot.getPathfindingCommand(); // Run a pathfinding command
    if (auto != null) CommandScheduler.getInstance().schedule(auto);
  }

  @Override
  public void autonomousExit() {
    if (auto != null) auto.cancel();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}