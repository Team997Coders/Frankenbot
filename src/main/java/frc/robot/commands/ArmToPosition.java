/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ArmToPosition extends Command {
  private double position, tolerance;

  public ArmToPosition(double position) {
    requires(Robot.arm);
    this.position = position;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("started armtoposition");
    tolerance = 200;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.arm.SetPosition(position);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(position - Robot.arm.getEncoder()) < tolerance);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.arm.setSpeed(0);
    System.out.println("finished armtoposition");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
