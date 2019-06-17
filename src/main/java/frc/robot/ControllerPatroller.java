package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class will create joysticks for all 6 slots and allow you to retrieve them based on their display name.
 */
public class ControllerPatroller {

    private static ControllerPatroller patroller;

    /**
     * Retrieve the singleton instance of the ControllerPatroller
     * 
     * @return the instance of the ControllerPatroller
     */
    public static ControllerPatroller getPatroller() {
        if (patroller == null) {
            patroller = new ControllerPatroller();
        }

        return patroller;
    }

    // The ordered list of user defined joysticks
    private List<Joystick> controllers = new ArrayList<>();

    /**
     * @param controllerNames the list of controller names in the order the user would like to access them
     */
    private ControllerPatroller() {
        createJoysticks();
    }

    // Create joysticks for all six slots so we can see what their names are
    private void createJoysticks() {
        for (int i = 0; i < 6; i++) {
            controllers.add(new Joystick(i));
        }
    }

    /**
     * @param index the index of the controller to access
     */
    public Joystick get(String name) {
        return controllers.stream().filter(c -> c.getName().contains(name)).findFirst().get();
    }
}
