package frc.robot.subsystems.dualArms;

import static frc.robot.subsystems.dualArms.DualArmConstants.*;

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
    private final LoggedMechanismLigament2d leftRoller;
    private final LoggedMechanismLigament2d rightRoller;

    public DualArm2d(String name, Color8Bit color) {
        this.name = name;

        this.mech = new LoggedMechanism2d(VISUALIZER_WIDTH, VISUALIZER_HEIGHT);

        LoggedMechanismRoot2d root = mech.getRoot("root", VISUALIZER_WIDTH / 2.0, VISUALIZER_HEIGHT / 2.0);

        leftArm = root.append(new LoggedMechanismLigament2d("leftArm", LEFT_ARM_LENGTH, LEFT_ARM_VISUAL_OFFSET_deg - LEFT_ARM_ANGLE, LEFT_LINE_WIDTH, color));
        rightArm = root.append(new LoggedMechanismLigament2d("rightArm", RIGHT_ARM_LENGTH, RIGHT_ARM_VISUAL_OFFSET_deg + RIGHT_ARM_ANGLE, RIGHT_LINE_WIDTH, color));

        leftRoller = leftArm.append(new LoggedMechanismLigament2d("leftRoller", LEFT_ROLLER_LENGTH, LEFT_ROLLER_ANGLE_OFFSET_deg, LEFT_ROLLER_LINE_WIDTH, color));
        rightRoller = rightArm.append(new LoggedMechanismLigament2d("rightRoller", RIGHT_ROLLER_LENGTH, RIGHT_ROLLER_ANGLE_OFFSET_deg, RIGHT_ROLLER_LINE_WIDTH, color));
    }

    public void setLeftArmAngle(double angle) {
        leftArm.setAngle(LEFT_ARM_VISUAL_OFFSET_deg - angle);
    }

    public void setLeftRollerAngle(double angle) {
        leftRoller.setAngle(LEFT_ROLLER_ANGLE_OFFSET_deg + angle);
    }

    public void setRightArmAngle(double angle) {
        rightArm.setAngle(RIGHT_ARM_VISUAL_OFFSET_deg + angle);
    }

    public void setRightRollerAngle(double angle) {
        rightRoller.setAngle(RIGHT_ROLLER_ANGLE_OFFSET_deg + angle);
    }

    public void periodic() {
        SmartDashboard.putData(name, mech);
        Logger.recordOutput(name, mech);
    }
}
