package frc.robot.subsystems.dualArms;

import frc.robot.devices.controller.OI;
import frc.robot.util.Controls;

public class DualArmControls extends Controls {
	private DualArm dualArm;
	
	public DualArmControls(DualArm dualArm) {
		this.dualArm = dualArm;
	}

	@Override
	public void update() {
		if (OI.DR.getAButtonPressed()) {
			dualArm.raise();
		}

		if (OI.DR.getBButtonPressed()) {
			dualArm.lower();
		}

		if (OI.DR.getXButtonPressed()) {
			dualArm.intake();
		}

		if (OI.DR.getYButtonPressed()) {
			dualArm.expel();
		}
	}
}
