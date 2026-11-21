package frc.robot.subsystems.dualArms;

import static frc.robot.subsystems.dualArms.DualArmConstants.LEFT_ARM_ANGLE;
import static frc.robot.subsystems.dualArms.DualArmConstants.LEFT_ARM_LENGTH;
import static frc.robot.subsystems.dualArms.DualArmConstants.LEFT_LINE_WIDTH;
import static frc.robot.subsystems.dualArms.DualArmConstants.RIGHT_ARM_ANGLE;
import static frc.robot.subsystems.dualArms.DualArmConstants.RIGHT_ARM_LENGTH;
import static frc.robot.subsystems.dualArms.DualArmConstants.RIGHT_LINE_WIDTH;
import static frc.robot.subsystems.dualArms.DualArmConstants.VISUALIZER_HEIGHT;
import static frc.robot.subsystems.dualArms.DualArmConstants.VISUALIZER_WIDTH;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class DualArm2d {
	private final String name;
	private final LoggedMechanism2d mech;
	private final LoggedMechanismLigament2d leftArm;
	private final LoggedMechanismLigament2d rightArm;

	public DualArm2d(String name, Color8Bit color) {
		this.name = name;

		this.mech = new LoggedMechanism2d(VISUALIZER_WIDTH, VISUALIZER_HEIGHT);

		LoggedMechanismRoot2d root = mech.getRoot("root", 0, 0);

		leftArm = root.append(new LoggedMechanismLigament2d("leftArm", LEFT_ARM_LENGTH, LEFT_ARM_ANGLE, LEFT_LINE_WIDTH, color));
		rightArm = root.append(new LoggedMechanismLigament2d("rightArm", RIGHT_ARM_LENGTH, RIGHT_ARM_ANGLE, RIGHT_LINE_WIDTH, color));
	}

	public void setLeftArmAngle(double angle) {
		leftArm.setAngle(angle);
	}

	public void setLeftRollerAngle(double angle) {

	}

	public void setRightArmAngle(double angle) {
		rightArm.setAngle(angle);
	}

	public void setRightRollerAngle(double angle) {

	}

	public void periodic() {
		SmartDashboard.putData(name, mech);
		Logger.recordOutput(name, mech);
	}
}
