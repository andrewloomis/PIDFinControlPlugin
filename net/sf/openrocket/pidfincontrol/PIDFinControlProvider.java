package net.sf.openrocket.pidfincontrol;

import net.sf.openrocket.pidfincontrol.PIDFinControl;
import net.sf.openrocket.plugin.Plugin;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtensionProvider;

@Plugin
public class PIDFinControlProvider extends AbstractSimulationExtensionProvider {
	
	public PIDFinControlProvider() {
		super(PIDFinControl.class, "Flight", "PID Fin Controller");
	}
	
}