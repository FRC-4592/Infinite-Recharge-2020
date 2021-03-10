package frc.robot.Util;

import edu.wpi.first.wpilibj.Solenoid;

public class doubleSolenoid{
	Solenoid push;
	Solenoid pull;
	
	public doubleSolenoid(int push, int pull){
		this.push = new Solenoid(push);
		this.pull = new Solenoid(pull);
	}
	
	public void open(){
		push.set(true);
		pull.set(false);
	}
	
	public void close(){
		push.set(false);
		pull.set(true);
	}
}