package frc.robot.Lib;

import frc.robot.Lib.Loop.Loopable;

public interface SubsystemFramework extends Loopable{
	public abstract void outputToSmartDashboard();
	public abstract void setupSensors();
}