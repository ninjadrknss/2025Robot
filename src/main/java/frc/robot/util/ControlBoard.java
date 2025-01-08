package frc.robot.util;

import frc.robot.subsystems.Superstructure;

public class ControlBoard {
    private static ControlBoard instance = null;

    /* Controllers */
    private final PS5Controller driver = new PS5Controller(0);
//    private final PS5Controller operator = new PS5Controller(1);

    /* Subsystems */
    private final Superstructure superstructure;

    private ControlBoard() {
        superstructure = Superstructure.getInstance();

        configureDriverBindings();
        configureOperatorBindings();
    }

    public static ControlBoard getInstance() {
        if (instance == null) instance = new ControlBoard();
        return instance;
    }

    private void configureDriverBindings() {

    }

    private void configureOperatorBindings() {

    }
}
