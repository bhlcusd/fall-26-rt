package frc.robot.subsystems.dualArms;

import static frc.robot.subsystems.dualArms.DualArmConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorConfig;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.util.Util;

// TODO: Add rollers to arms - add two more motors called leftRoller and rightRoller; add to DualArm2d as well. Have seperate keybind X and Y to roll rollers forwards and backwards, respectively
public class DualArm extends SubsystemBase<DualArm.Command> {
	public enum Command {
		DISABLED,
		IDLE,
		RAISE,
		LOWER
	}

	public enum Travel {
		PHASE_1,
		PHASE_2
	}

	private final DualArm2d dualArm2d = new DualArm2d("DualArm2d", new Color8Bit(255, 0, 0));

	private final Motor leftMotor;
	private final Motor rightMotor;

	private double leftAngle_deg;
	private double rightAngle_deg;

	private static DualArm instance;

	private DualArm() {
		super("DualArm");

		MotorConfig leftConfig = new MotorConfig(LEFT_MOTOR_ID)
			.withCanbus(CANBUS_LEFT)
			.withBrake(BRAKE_LEFT)
			.withFoc(FOC_LEFT)
			.withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO_LEFT)
			.withPIDGains(kPLeft, kILeft, kDLeft, GRAVITY_LEFT)
			.withMotionMagic(CRUISE_VELOCITY_LEFT_degps, ACCELERATION_LEFT_degps2, JERK_LEFT_degp3)
			.withSim(DC_MOTOR_LEFT, GEARING_LEFT, SIM_MOI_LEFT_kgm2);

		this.leftMotor = new Motor("leftMotor", leftConfig);

		MotorConfig rightConfig = new MotorConfig(RIGHT_MOTOR_ID)
			.withCanbus(CANBUS_RIGHT)
			.withBrake(BRAKE_RIGHT)
			.withFoc(FOC_RIGHT)
			.withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO_RIGHT)
			.withPIDGains(kPRight, kIRight, kDRight, GRAVITY_RIGHT)
			.withMotionMagic(CRUISE_VELOCITY_RIGHT_degps, ACCELERATION_RIGHT_degps2, JERK_RIGHT_degp3)
			.withSim(DC_MOTOR_RIGHT, GEARING_RIGHT, SIM_MOI_RIGHT_kgm2);

		this.rightMotor = new Motor("rightMotor", rightConfig);
	}

	public static DualArm getInstance() {
		if (instance == null) {
			instance = new DualArm();
		}

		return instance;
	}

	@Override
	protected void inputPeriodic() {
		leftMotor.readInputs();
		rightMotor.readInputs();
	}

	@Override
	protected void handle() {
		switch (getCommand()) {
			case DISABLED:
			case IDLE:
				leftMotor.stop();
				rightMotor.stop();
				break;
			case RAISE:
				if (firstLoop()) {
					setSubstate(Travel.PHASE_1);
				}

				switch ((Travel) getSubstate()) {
					case PHASE_1:
						if (Util.inRange(RAISE_PHASE_2_THRESHOLD_deg, RAISE_TOLERANCE_deg)) {
							setSubstate(Travel.PHASE_2);
							break;
						}
						leftMotor.setMotionMagic(RAISE_PHASE_2_THRESHOLD_deg);
						break;
					case PHASE_2:
						if (!Util.inRange(RAISE_PHASE_2_THRESHOLD_deg, RAISE_TOLERANCE_deg)) {
							setSubstate(Travel.PHASE_1);
							return;
						}

						if (Util.inRange(RAISE_FINAL_THRESHOLD_deg, RAISE_TOLERANCE_deg)) {
							setCommand(Command.IDLE);
						}
						rightMotor.setMotionMagic(RAISE_FINAL_THRESHOLD_deg);
						break;
					default:
						throw new IllegalArgumentException("DualArm: Unknown substate: " + getSubstate());
				}
				break;
			case LOWER:
				if (firstLoop()) {
					setSubstate(Travel.PHASE_1);
				}

				switch ((Travel) getSubstate()) {
					case PHASE_1:
						if (Util.inRange(LOWER_PHASE_2_THRESHOLD_deg, LOWER_TOLERANCE_deg)) {
							setSubstate(Travel.PHASE_2);
							break;
						}
						leftMotor.setMotionMagic(LOWER_PHASE_2_THRESHOLD_deg);
						break;
					case PHASE_2:
						if (!Util.inRange(LOWER_PHASE_2_THRESHOLD_deg, LOWER_TOLERANCE_deg)) {
							setSubstate(Travel.PHASE_1);
							return;
						}

						if (Util.inRange(LOWER_FINAL_THRESHOLD_deg, LOWER_TOLERANCE_deg)) {
							setCommand(Command.IDLE);
						}
						rightMotor.setMotionMagic(LOWER_FINAL_THRESHOLD_deg);
						break;
					default:
						throw new IllegalArgumentException("DualArm: Unknown substate: " + getSubstate());
				}
				break;
			default:
				throw new IllegalArgumentException("DualArm: handling invalid state: " + getCommand());
		}
	}

	@Override
	protected void outputPeriodic() {
		dualArm2d.setLeftArmAngle(leftAngle_deg);
		dualArm2d.setRightArmAngle(rightAngle_deg);
		dualArm2d.periodic();

		Logger.recordOutput("DualArm/LeftAngle_deg", leftAngle_deg);
		Logger.recordOutput("DualArm/RightAngle_deg", rightAngle_deg);
	}

	public void disable() {
		setCommand(Command.DISABLED);
	}

	public void idle() {
		setCommand(Command.IDLE);
	}

	public void raise() {
		setCommand(Command.RAISE);
	}

	public void lower() {
		setCommand(Command.LOWER);
	}
}
