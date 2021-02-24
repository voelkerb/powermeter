
// Internal state machine for sampling
enum class SampleState{IDLE, SAMPLE};

// Available measures are VOLTAGE+CURRENT, ACTIVE+REACTIVE Power or both
enum class Measures{VI, PQ, VIPQ, VI_RMS};
enum class StreamType{USB, TCP, UDP, MQTT};