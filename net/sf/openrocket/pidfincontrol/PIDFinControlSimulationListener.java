package net.sf.openrocket.pidfincontrol;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.aerodynamics.FlightConditions;
import net.sf.openrocket.aerodynamics.AerodynamicForces;
import net.sf.openrocket.rocketcomponent.FinSet;
import net.sf.openrocket.rocketcomponent.RocketComponent;
import net.sf.openrocket.simulation.AccelerationData;
import net.sf.openrocket.simulation.FlightDataType;
import net.sf.openrocket.unit.UnitGroup;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.MathUtil;

/**
 * The simulation listener that is attached to the simulation.
 * It is instantiated when the simulation run is started and the
 * methods are called at each step of the simulation.
 */
public class PIDFinControlSimulationListener extends AbstractSimulationListener {
	
	private static final double accelerations[] = {
			11.46,
			58.68,
			114.24,
			36.99,
			27.84,
			22.33,
			20.25,
			19.85,
			20.36,
			-5.26,
			-10.83,
			-10.77,
			-10.85,
			-10.94,
			-10.83,
			-10.62
	};
	private static final double altitudes[] = {
			3.5,
			4.9,
			6.3,
			7.9,
			9.5,
			11.3,
			12.8,
			14.3,
			15.8,
			17.2,
			18.6,
			19.9,
			21.2,
			22.4,
			22.4,
			23.6,
			24.7,
			25.7,
			26.8,
			27.7,
			28.6,
			29.5,
			30.3,
			31.1,
			31.8,
			32.5,
			33.1,
			33.6,
			34.2,
			34.6,
			35.1,
			35.4,
			35.8,
			36.0,
			36.3,
			36.5,
			36.6,
			36.7,
			36.7};
	
	double pGain;
	double iGain;
	double dGain;
	private static final double scaler = 0.001;
		
	// Name of control fin set
	private static final String FIN_NAME = "Trapezoidal fin set";
	
	// Define custom flight data type
	private static final FlightDataType FIN_THICKNESS_TYPE = FlightDataType.getType("Control fin thickness", "\u03B1fc", UnitGroup.UNITS_LENGTH);
	private static final FlightDataType ERROR = FlightDataType.getType("PID Error", "\u03B1fc", UnitGroup.UNITS_ACCELERATION);
	private static final FlightDataType IDEAL_VALUE = FlightDataType.getType("Ideal Acceleration", "\u03B1fc", UnitGroup.UNITS_ACCELERATION);
	
	// Simulation time at which PID controller is activated
//	private static final double START_TIME = 0.4;
	private static final double START_TIME = 0.07;
	
	// Desired altitude
	private static final double SETPOINT = 36.0;
	
	// Maximum control fin angle (rad)
	private static final double MAX_FIN_THICKNESS = 0.5;
	private static final double MIN_FIN_THICKNESS = 0.3;
	
	private double prevTime = 0;
	private double errorIntegral = 0;
	private double lastError = 0;
	private double acc = 0;
	private double takeoffTime = 0;
	private boolean takeoff = false;
	private boolean startFins = false;
	private int timeStep = 0;
	private double lastThickness = 0;
	private double lastVelocity = 0;
	private double dt = 0.07;
	
	
	public PIDFinControlSimulationListener(double pGain, double iGain, double dGain) {
		super();
		this.pGain = pGain * scaler;
		this.iGain = iGain * scaler;
		this.dGain = dGain * scaler;
	}

//	@Override
//	public AccelerationData postAccelerationCalculation(SimulationStatus status, AccelerationData acceleration) throws SimulationException {
//		if (acceleration != null)
//		{
//			acc = acceleration.getLinearAccelerationWC().z;
//			if (acc > 30.0) {
//				takeoff = true;
//				takeoffTime = status.getSimulationTime();
//			}
//		}
//		return null;
//	}
	
	
	
//	@Override
//	public void startSimulation(SimulationStatus status) throws SimulationException {
//		
//	}

	@Override
	public void postStep(SimulationStatus status) throws SimulationException {
		// Activate PID controller only after a specific time
		if (timeStep >= accelerations.length) return;
		double time = status.getSimulationTime();
		if (time < START_TIME && acc < accelerations[0])
		{
			return;
		}
//		double altitude = status.getRocketPosition().z;
//		if (time >= START_TIME || altitude >= altitudes[0]) {
////			startFins = true;
//		}
//		else
//		{
////			prevTime = time;
//			return;
//		}
		status.getFlightData().setValue(IDEAL_VALUE, accelerations[timeStep]);
		
		if (timeStep != 0 && time - prevTime < 0.07) 
		{
//			startFins = false;
			status.getFlightData().setValue(FIN_THICKNESS_TYPE, lastThickness);
			status.getFlightData().setValue(ERROR, lastError);
			return;
		}
		prevTime = time;
		
		// Find the fin set named CONTROL
		FinSet finset = null;
		for (RocketComponent c : status.getConfiguration()) {
			if ((c instanceof FinSet) && (c.getName().equals(FIN_NAME))) {
				finset = (FinSet) c;
				break;
			}
		}
		if (finset == null) {
			throw new SimulationException("A fin set with name '" + FIN_NAME + "' was not found");
		}
//		finset.setThickness(0.005);
		// Determine time step
//		double deltaT = status.getSimulationTime() - prevTime;
//		prevTime = status.getSimulationTime();
		
		
		Coordinate velocity = status.getRocketVelocity();
		
		// PID controller
//		double error = altitude - altitudes[timeStep];
//		if (acc == 0) throw new SimulationException("Acceleration 0");
		acc = (velocity.z - lastVelocity)/dt;
		lastVelocity = velocity.z;
		double error = acc - accelerations[timeStep];
		timeStep++;
		
		double p = pGain * error;
//		errorIntegral += error * deltaT;
//		double i = iGain * errorIntegral;
//		
//		
//		double d = deltaT == 0 ? 0 : -dGain * ((error - lastError)/deltaT);
		lastError = error;
		double output = p;
		
		if (output > 0.007) output = 0.007;
		else if (output < 0) output = 0;
		
		double thickness = 0.003 + output;
		lastThickness = thickness;
		finset.setThickness(thickness);

		status.getFlightData().setValue(FIN_THICKNESS_TYPE, finset.getThickness());
		status.getFlightData().setValue(ERROR, error);
		
	}
}