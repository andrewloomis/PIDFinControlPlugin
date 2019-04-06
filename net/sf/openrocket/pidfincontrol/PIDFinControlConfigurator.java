package net.sf.openrocket.pidfincontrol;

import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSpinner;
import java.awt.GridLayout;

import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.gui.SpinnerEditor;
import net.sf.openrocket.gui.adaptors.DoubleModel;
import net.sf.openrocket.gui.components.BasicSlider;
import net.sf.openrocket.gui.components.UnitSelector;
import net.sf.openrocket.plugin.Plugin;
import net.sf.openrocket.simulation.extension.AbstractSwingSimulationExtensionConfigurator;
import net.sf.openrocket.unit.UnitGroup;

/**
 * The Swing configuration dialog for the extension.
 * 
 * The abstract implementation provides a ready JPanel using MigLayout
 * to which you can build the dialog. 
 */
@Plugin
public class PIDFinControlConfigurator extends AbstractSwingSimulationExtensionConfigurator<PIDFinControl> {
	
	public PIDFinControlConfigurator() {
		super(PIDFinControl.class);
	}
	
	@Override
	protected JComponent getConfigurationComponent(PIDFinControl extension, Simulation simulation, JPanel panel) {
		panel.setLayout(new GridLayout(3,2));
		
		panel.add(new JLabel("P Gain:"));
		
		DoubleModel p = new DoubleModel(extension, "PGain", UnitGroup.UNITS_RELATIVE, 0);
		
		JSpinner pSpin = new JSpinner(p.getSpinnerModel());
		pSpin.setEditor(new SpinnerEditor(pSpin));
		panel.add(pSpin, "w 65lp!");
		
		panel.add(new JLabel("I Gain:"));
		
		DoubleModel i = new DoubleModel(extension, "IGain", UnitGroup.UNITS_RELATIVE, 0);
		
		JSpinner iSpin = new JSpinner(i.getSpinnerModel());
		iSpin.setEditor(new SpinnerEditor(iSpin));
		panel.add(iSpin, "w 65lp!");
		
		panel.add(new JLabel("D Gain:"));
		
		DoubleModel d = new DoubleModel(extension, "DGain", UnitGroup.UNITS_RELATIVE, 0);
		
		JSpinner dSpin = new JSpinner(d.getSpinnerModel());
		dSpin.setEditor(new SpinnerEditor(dSpin));
		panel.add(dSpin, "w 65lp!");
		
		return panel;
	}
	
}
