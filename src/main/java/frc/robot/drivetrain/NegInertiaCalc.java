package frc.robot.drivetrain;

public class NegInertiaCalc {
    
    private double negInertiaScalar;
    private double negInertiaAccumulator;
    private double oldTurn;
    
    public NegInertiaCalc(double negInertiaScalar) {
        this.negInertiaScalar = negInertiaScalar;
    }
    
    /**
     * Calculate the counter turn power to cancel out the negative inertia
     * @param turn the current turn value
     * @return the value to counter the inertia
     */
    public double calculate(double turn) {
        double newTurn = turn;
        double negInertia = newTurn - oldTurn;
        oldTurn = newTurn;
        
        double negInertiaPower = negInertia * negInertiaScalar;
        negInertiaAccumulator += negInertiaPower;
        newTurn += negInertiaAccumulator;
        
        if (negInertiaAccumulator > 1) {
            negInertiaAccumulator -= 1;
        } else if (negInertiaAccumulator < -1) {
            negInertiaAccumulator += 1;
        } else {
            negInertiaAccumulator = 0;
        }
        
        return newTurn;
    }
    
    public void reset() {
        negInertiaAccumulator = 0;
        oldTurn = 0;
    }
}
