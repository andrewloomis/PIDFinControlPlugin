package net.sf.openrocket.pidfincontrol;

import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtension;

/**
 * The actual simulation extension.  A new instance is created for
 * each simulation it is attached to.
 * 
 * This class contains the configuration and is called before the
 * simulation is run.  It can do changes to the simulation, such
 * as add simulation listeners.
 * 
 * All configuration should be stored in the config variable, so that
 * file storage will work automatically.
 */
public class PIDFinControl extends AbstractSimulationExtension {
	
	@Override
	public String getName() {
		return "PID Fin Control";
	}
	
	@Override
	public String getDescription() {
		// This description is shown when the user clicks the info-button on the extension
		return "This extension controls Fin thickness using PID.";
	}
	
	@Override
	public void initialize(SimulationConditions conditions) throws SimulationException {
		conditions.getSimulationListenerList().add(new PIDFinControlSimulationListener(getPGain(), getIGain(), getDGain()));
	}
	
//	public double getMultiplier() {
//		return config.getDouble("multiplier", 1.0);
//	}
	
	public double getPGain() {
		return config.getDouble("pGain", 0.0);
	}
	
	public double getIGain() {
		return config.getDouble("iGain", 0.0);
	}
	
	public double getDGain() {
		return config.getDouble("dGain", 0.0);
	}
	
	public void setPGain(double pGain) {
		config.put("pGain", pGain);
		fireChangeEvent();
	}
	
	public void setIGain(double iGain) {
		config.put("iGain", iGain);
		fireChangeEvent();
	}
	
	public void setDGain(double dGain) {
		config.put("dGain", dGain);
		fireChangeEvent();
	}
	
	
	
//	public void setMultiplier(double multiplier) {
//		config.put("multiplier", multiplier);
//		fireChangeEvent();
//	}
	
}