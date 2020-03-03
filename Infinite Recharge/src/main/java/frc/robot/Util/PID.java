package frc.robot.Util;

@SuppressWarnings("unused")
public class PID{
	private double Kp;
	private double Ki;
	private double Kd;
	private double error;
	private double error_sum;
	private double output;
	
	//Constructors
		//P Loop
			public PID(double Kp){
				this.Kp = Kp;
			}
		
		//PI Loop
			public PID(double Kp, double Ki){
				this.Kp = Kp;
				this.Ki = Ki;
			}
		
		//PID Loop
			public PID(double Kp, double Ki, double Kd){
				this.Kp = Kp;
				this.Ki = Ki;
				this.Kd = Kd;
			}

	//Caller Methods
		//P Loop
			public double getOutputP(double error){
				this.error = error;
				
				output = error * Kp;
				
				if(output > 1){
					return 1;
				}else if(output < -1){
					return -1;
				}
				
				return output;
			}
			
			public double getControlledOutputP(double error, double controlValue){
				this.error = error;
				
				output = error * Kp;
				
				if(output > controlValue){
					return controlValue;
				}else if(output < -controlValue){
					return -controlValue;
				}
				
				return output;
			}
			
		//PI Loop
			public double getOutputPI(double error){
				this.error = error;
				error_sum += error;
				
			//	//edited start
			//	if(error <= 50 && error >= -50){
			//		error = 0;
			//	}
				//edited end	
				output = (error * Kp) + (error_sum * Ki);
				
				if(output > 1){
					return 1;
				}else if(output < -1){
					return -1;
				}
				
				return output;
			}
	
	public void setError_Sum(double error_sum){
		this.error_sum = error_sum;
	}		
}