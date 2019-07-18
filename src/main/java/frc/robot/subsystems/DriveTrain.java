/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ArcadeDrive;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private WPI_TalonSRX leftTalon, rightTalon;
  private WPI_VictorSPX frontLeftVictor, frontRightVictor, backLeftVictor, backRightVictor;
  private AHRS gyro;
  
  public DriveTrain(){
    leftTalon = new WPI_TalonSRX(RobotMap.leftTalon);
    rightTalon = new WPI_TalonSRX(RobotMap.rightTalon);
    frontLeftVictor = new WPI_VictorSPX(RobotMap.frontLeftVictor);
    frontRightVictor = new WPI_VictorSPX(RobotMap.frontRightVictor);
    backLeftVictor = new WPI_VictorSPX(RobotMap.backLeftVictor);
    backRightVictor = new WPI_VictorSPX(RobotMap.backRightVictor);

	gyro = new AHRS(RobotMap.gyro);

    frontLeftVictor.follow(leftTalon);
    backLeftVictor.follow(leftTalon);
    frontRightVictor.follow(rightTalon);
    backRightVictor.follow(rightTalon);

    rightTalon.setInverted(true);
    frontRightVictor.setInverted(true);
    backRightVictor.setInverted(true);

    leftTalon.setInverted(false);
    frontLeftVictor.setInverted(false);
    backLeftVictor.setInverted(false);

    /*
		 * CTRE encoder on each Talon on the drivetrain, mechanically connected to the front wheels (i.e. 1:1 ratio)
		 */
		leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		leftTalon.setSensorPhase(true);
		rightTalon.setSensorPhase(true);
		
		leftTalon.setNeutralMode(NeutralMode.Coast);
		rightTalon.setNeutralMode(NeutralMode.Coast);
		
		/* set the peak, nominal outputs */
		leftTalon.configNominalOutputForward(0, 10);
		leftTalon.configNominalOutputReverse(0, 10);
		leftTalon.configPeakOutputForward(1, 10);
		leftTalon.configPeakOutputReverse(-1, 10);
		
		leftTalon.enableCurrentLimit(true);
		leftTalon.configPeakCurrentLimit(40, 10);
		leftTalon.configPeakCurrentDuration(100, 10);
		leftTalon.configContinuousCurrentLimit(30, 10);
		
		rightTalon.configNominalOutputForward(0, 10);
		rightTalon.configNominalOutputReverse(0, 10);
		rightTalon.configPeakOutputForward(1, 10);
		rightTalon.configPeakOutputReverse(-1, 10);
		
		rightTalon.enableCurrentLimit(true);
		rightTalon.configPeakCurrentLimit(40, 10);
		rightTalon.configPeakCurrentDuration(100, 10);
		rightTalon.configContinuousCurrentLimit(30, 10);
		
		leftTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 40, 10);
		//leftTalon.configOpenloopRamp(0.25, 10);
		rightTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 40, 10);
		//rightTalon.configOpenloopRamp(0.25, 10);
		
		/* set closed loop gains in slot0 */
		leftTalon.config_kF(0, 0.1097, 10);
		leftTalon.config_kP(0, 0.113333, 10);
		leftTalon.config_kI(0, 0, 10);
		leftTalon.config_kD(0, 0, 10);		

		rightTalon.config_kF(0, 0.1097, 10);
		rightTalon.config_kP(0, 0.113333, 10);
		rightTalon.config_kI(0, 0, 10);
		rightTalon.config_kD(0, 0, 10);	
  }

  /**
   * Set motor speed with a percent modifier.
   */
  public void SetSpeed(double leftSpeed, double rightSpeed){
	double modifier = .5;
    leftTalon.set(ControlMode.PercentOutput, leftSpeed * modifier);
    rightTalon.set(ControlMode.PercentOutput, rightSpeed * modifier);
  }

  /**
   * Set motor speed without any percent modifiers. 1 means go at 100%.
   */
  public void SetUnmodifiedSpeed(double leftSpeed, double rightSpeed){
    leftTalon.set(ControlMode.PercentOutput, leftSpeed);
    rightTalon.set(ControlMode.PercentOutput, rightSpeed);
  }

  public double GetAngle(){
	return gyro.getAngle();
  }

  public int GetLeftEncoder(){
	  return leftTalon.getSelectedSensorPosition();
  }

  public int GetRightEncoder(){
	  return rightTalon.getSelectedSensorPosition();
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("left encoder ticks", GetLeftEncoder());
    SmartDashboard.putNumber("right encoder ticks", GetRightEncoder());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ArcadeDrive());
  }
}
