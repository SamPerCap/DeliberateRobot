import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Key;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.TouchAdapter;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Waypoint;
import lejos.utility.Delay;

public class DeliberateRobot {
	static EV3 brick = (EV3) BrickFinder.getDefault();
	static Wheel leftWheel = WheeledChassis.modelWheel(Motor.B, 56).offset(-53.2);
	static Wheel rigthWheel = WheeledChassis.modelWheel(Motor.C, 56).offset(53.2);
	static int speed = 250;
	static int acceleration = 100;
	static TouchAdapter rightSensor;
	static TouchAdapter leftSensor;
	static boolean done = false;
	static Waypoint actualWP;
	static Waypoint beforeMove;
	static Waypoint differenceWP;
	static PoseProvider PP;
	
	public static void main(String[] args) {
Button.ESCAPE.addKeyListener(new lejos.hardware.KeyListener() {
			
			@Override
			public void keyReleased(Key k) {
				// TODO Auto-generated method stub
				done = true;
			}
			
			@Override
			public void keyPressed(Key k) {
				// TODO Auto-generated method stub
				
			}
});
		// TODO Auto-generated method stub
		Chassis chassis = new WheeledChassis(new Wheel[] { leftWheel, rigthWheel }, WheeledChassis.TYPE_DIFFERENTIAL);
		MovePilot pilot = new MovePilot(chassis);
		pilot.setLinearAcceleration(acceleration);
		pilot.setLinearSpeed(speed);
		pilot.setAngularAcceleration(acceleration);
		pilot.setAngularSpeed(speed);
		Navigator navi = new Navigator(pilot);
		
		try(
				EV3TouchSensor rightTouchSensor = new EV3TouchSensor((brick.getPort("S3")));	
				EV3TouchSensor leftTouchSensor = new EV3TouchSensor((brick.getPort("S2")));
				){
			rightSensor = new TouchAdapter(rightTouchSensor);
			leftSensor = new TouchAdapter(leftTouchSensor);
			
		
		navi.addWaypoint(2000,0);
		navi.addWaypoint(2000,2000);
		navi.addWaypoint(0,2000);
		navi.addWaypoint(0,0);	
		while(!navi.pathCompleted() && !done) {
			if(rightSensor.isPressed() || leftSensor.isPressed()) {
				navi.stop();
				if(rightSensor.isPressed()) {
					beforeMove = new Waypoint(navi.getPoseProvider().getPose().getX(), navi.getPoseProvider().getPose().getY(),navi.getPoseProvider().getPose().getHeading());
					LCD.drawString("Right touch", 0, 0);
					pilot.backward();
					pilot.travel(300);
					pilot.rotate(-45);
					pilot.forward();
					pilot.travel(300);
					actualWP = new Waypoint(navi.getPoseProvider().getPose().getX(), navi.getPoseProvider().getPose().getY(),navi.getPoseProvider().getPose().getHeading());
					differenceWP = new Waypoint(
							((double) (beforeMove.getX() - actualWP.getX())),
							((double) (beforeMove.getY() - actualWP.getY())),
							(beforeMove.getHeading() - (-45)));
					PP.getPose().setLocation(((float) differenceWP.getX()), ((float)differenceWP.getY()));
					PP.getPose().setHeading((float) differenceWP.getHeading());
					navi.setPoseProvider(PP);
				}else if(leftSensor.isPressed()) {
					LCD.drawString("Left touch", 0, 0);
					pilot.backward();
					Delay.msDelay(500);
					pilot.rotate(45);
					pilot.forward();
					Delay.msDelay(500);
					
				}
			}else {
				LCD.drawString("Follow the path", 0, 0);
				LCD.drawString("(" + (int)navi.getWaypoint().getX() +
			"," + (int)navi.getWaypoint().getY() + ")", 0, 1);
			LCD.drawString("(" + (int) navi.getPoseProvider().getPose().getX()+
					"," + (int)navi.getPoseProvider().getPose().getY() + ")", 0, 2);
				navi.followPath();
			}
			
		}
		navi.stop();
	}
	}
}
