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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.MoveArmUp;
import frc.robot.commands.MoveArmDown;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
	private WPI_TalonSRX armTalon;
	private SensorCollection sensorCollection;
	
  public Arm(){

    armTalon = new WPI_TalonSRX(RobotMap.armTalon);

		//maximum forward ticks is about 31000 
		armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		armTalon.setSensorPhase(false);
		armTalon.setNeutralMode(NeutralMode.Coast);
		
		armTalon.configNominalOutputForward(0, 10);
		armTalon.configNominalOutputReverse(0, 10);
		armTalon.configPeakOutputForward(1, 10);
		armTalon.configPeakOutputReverse(-1, 10);
		
		armTalon.enableCurrentLimit(true);
		armTalon.configPeakCurrentLimit(40, 10);
		armTalon.configPeakCurrentDuration(100, 10);
    armTalon.configContinuousCurrentLimit(30, 10);
    
		armTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 40, 10);
		//armTalon.configOpenloopRamp(0.25, 10);
		
		/* set closed loop gains in slot0 */
		armTalon.config_kF(0, 0, 10);
		armTalon.config_kP(0, 0.23, 10);
		armTalon.config_kI(0, 0, 10);
		armTalon.config_kD(0, 0.05, 10);

		sensorCollection = new SensorCollection(armTalon);
  }

	/**
	 * unfinished method
	 */
  public void SetPosition(double position){
		armTalon.set(ControlMode.Position, position);
	}
	
	public void autoZero() {
		if (getRearLimitSwitch()) {
			armTalon.setSelectedSensorPosition(0);
		}
	}

  public void setSpeed(double armSpeed){
    armTalon.set(ControlMode.PercentOutput, armSpeed);
  }

	public void stopArm() {
		armTalon.set(ControlMode.PercentOutput, 0);
	}

	public int getEncoder() {
		return armTalon.getSelectedSensorPosition();
	}

	public boolean getFrontLimitSwitch() {
		return sensorCollection.isFwdLimitSwitchClosed();
	}

	public boolean getRearLimitSwitch() {
		return sensorCollection.isRevLimitSwitchClosed();
	}

	public void updateSmartDashboard() {
		SmartDashboard.putNumber("arm encoder ticks", getEncoder());
		SmartDashboard.putBoolean("forward limit switch", getFrontLimitSwitch());
		SmartDashboard.putBoolean("rear limit switch", getRearLimitSwitch());
	}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
