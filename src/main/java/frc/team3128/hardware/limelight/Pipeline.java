package frc.team3128.hardware.limelight;

public enum Pipeline {
    RED (0),
    BLUE (2);

    private int pipeline;
    private Pipeline(int pipeline) {
        this.pipeline = pipeline;
    }

    public int getPipeline() {
        return pipeline;
    }
}