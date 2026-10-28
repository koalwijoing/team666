#pragma once

#include <opencv2/core.hpp>
#include <cstdint>
#include <string>
#include <vector>
#include <array>

namespace p3_crosswalk {

enum class State { Unavailable, NotDetected, Candidate, Confirmed, Stale, Held, Predicted };
enum class Relation { Unknown, NoCrosswalk, Outside, Boundary, Inside };

struct Config {
    int analysis_width = 320;
    int min_stripes = 4;
    double ttl_ms = 400.0;
    std::uint64_t max_age_frames = 2;
    int confirm_observations = 2;
    bool prediction_enabled = true;
    double prediction_max_ms = 400.0;
    std::uint64_t prediction_max_frames = 6;
};

struct Observation {
    std::uint64_t delivery_id = 0;
    double source_ms = 0.0;  // host capture-return, never infer-complete
    double now_ms = 0.0;
    cv::Mat left_bgr;
    cv::Mat twin_mask;       // optional; only the same successful source image
};

struct Result {
    State state = State::Unavailable;
    std::uint64_t delivery_id = 0;
    double source_ms = 0.0;
    int width = 0, height = 0;
    int confirmations = 0;
    double geometry_score = 0.0;  // heuristic evidence, not calibrated confidence
    double twin_support = 0.0;
    double elapsed_ms = 0.0;
    std::vector<cv::Point> region;
    std::vector<std::vector<cv::Point>> stripes;
    double projection_ms = 0.0;  // image time being projected; source_ms stays measured
    double prediction_age_ms = 0.0;
    double prediction_sigma_px = 0.0; // model sigma; not calibrated probability
};

class Detector {
public:
    explicit Detector(Config config = {});
    const Result& update(const Observation& input);
    Result view(std::uint64_t display_id, double now_ms) const;
    void reset();
private:
    Config config_;
    Result result_;
};

// Four independent position/velocity Kalman filters in normalized image space.
// The original geometry detector remains independently usable and unchanged.
class PredictiveDetector {
public:
    explicit PredictiveDetector(Config config = {});
    const Result& update(const Observation& input);
    Result view(std::uint64_t display_id, double now_ms, double image_source_ms) const;
    void reset();
private:
    struct Axis { double value=0,velocity=0,p00=0,p01=0,p11=0; };
    static Axis advance(Axis axis,double dt);
    Result project(std::uint64_t id,double source,double now) const;
    void seed(const Result& measurement);
    Config config_;
    Detector detector_;
    std::array<Axis,4> axes_{};
    Result measurement_,result_;
    bool tracking_=false;
    double filter_ms_=0,last_input_ms_=0;
    std::uint64_t last_input_id_=0;
    cv::Mat previous_scene_;
};

struct Person {
    cv::Rect2f box;
    float confidence = 0.0f;
    std::uint64_t run_id = 0;
    double source_ms = 0.0;
    int age_frames = 0;
    bool inferred_now = true;
};

struct PersonAssociation {
    Person person;
    cv::Point2f foot;
    Relation relation = Relation::Unknown;
};

std::vector<PersonAssociation> associate(const Result& crosswalk,
    const std::vector<Person>& persons, double now_ms, double ttl_ms = 400.0);
void draw(cv::Mat& left_bgr, const Result& crosswalk,
          const std::vector<PersonAssociation>& persons, bool text = true);
const char* state_name(State value);
const char* relation_name(Relation value);

}  // namespace p3_crosswalk
