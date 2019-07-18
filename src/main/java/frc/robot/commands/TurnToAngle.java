/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TurnToAngle extends Command {
  double whereGo, left, right, postion, error, speedModifier;
  public TurnToAngle(double whereGo) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    speedModifier = .05;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    postion = Robot.driveTrain.GetAngle();
    error = whereGo - postion;
    left = speedModifier * error;
    right = -speedModifier * error;
    Robot.driveTrain.SetUnmodifiedSpeed(left, right);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Math.abs(error) <= 10){
      return true;
    } else {
    return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.SetSpeed(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
