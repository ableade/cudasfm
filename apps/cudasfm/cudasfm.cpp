#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <set>
#include <fstream>
#include <utility>
#include <map>
#include "shomatcher.hpp"
#include "shotracking.h"
#include "reconstructor.h"
#include "reconstructabilityscores.h"
#include <gflags/gflags.h>
#include <chrono>
#include <memory>

using std::pair;
using std::cerr;
using std::vector;
using std::string;
using std::map;
using std::make_shared;

DEFINE_bool(
    resize,
    false,
    "Choose whether to resize images before feature extraction, Default is False"
);
DEFINE_int32(
    max_image_size,
    FEATURE_PROCESS_SIZE,
    "If resizing, this is the max width to use for resizing, Defaults to 2500"
);
DEFINE_string(
    feature_type,
    "HAHOG",
    "Feature detection algorithm to use. Choose from SIFT, SURF, ORB, HAHOG, \
    the default is HAHOG"
);
DEFINE_string(
    reconstruction_score,
    "matchescount",
    "Scoring metric to sort image pairs"
);
DEFINE_double(
    knn_ratio,
    0.45,
    "Choose a percentage of the dataset for the knn range. Default is 0.25"
);
DEFINE_string(
    image_1,
    "",
    "First image to use in bootsrapping reconstruction"
);
DEFINE_string(
    image_2,
    "",
    "Second image to use in bootsrapping reconstruction"
);
DEFINE_bool(
    showInitialCameras,
    false,
    "Choose whether to plot and show initial camera positions before full bundle adjustment process"
);
DEFINE_double(
    outlier_threshold,
    BUNDLE_OUTLIER_THRESHOLD,
    "File containing candidate matches"
);

int main(int argc, char* argv[])
{
    map<string, RobustMatcher::Feature> featureMatchingAlgorithms{
        {"ORB", RobustMatcher::Feature::orb},
        {"SIFT", RobustMatcher::Feature::sift},
        {"SURF", RobustMatcher::Feature::surf},
        {"HAHOG", RobustMatcher::Feature::hahog}
    };
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    //string solverFlags = "-logtostderr";
    //google::InitGoogleLogging(solverFlags.c_str());

    string cameraCalibrationFile;
    if (argc < 2)
    {
        cerr << "Program usage: <flight session directory> optional -- <camera calibration file>. \
            Use the --help option to show additional flags \n";
        exit(1);
    }
    FlightSession flight;
    (argc > 2) ? flight = FlightSession(argv[1], argv[2]) : flight = FlightSession(argv[1]);


    RobustMatcher::Feature featureType = RobustMatcher::Feature::orb;
    try {
        featureType = featureMatchingAlgorithms.at(FLAGS_feature_type);
    }
    catch (std::out_of_range& e) {
        featureType = RobustMatcher::Feature::orb;
    }

    auto extractionSize = -1;
    if (FLAGS_resize) {
        extractionSize = FLAGS_max_image_size;
    }
    ShoMatcher shoMatcher(make_shared<FlightSession>(flight), extractionSize, featureType);


    if (FLAGS_showInitialCameras) {
        std::cerr << "Initial camera positions is not implemented yet" << std::endl;
    }
    
    //**** Begin Matching Pipeline ******
    if (argc > 3) {
        //A candidate file was provided 
        const auto candidateFile = argv[3];
        cerr << "Using candidate file " << candidateFile << std::endl;
        shoMatcher.getCandidateMatchesFromFile(candidateFile);
    }
    else {
        int k = FLAGS_knn_ratio * flight.getImageSet().size();
        shoMatcher.getCandidateMatchesUsingKNNSearch(k);
    }
    auto featureExtractionStartTime = std::chrono::high_resolution_clock::now();
    shoMatcher.extractFeatures();
    auto featureExtractionEndTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(featureExtractionStartTime - featureExtractionEndTime);
    std::cout << "Time taken for feature extraction: " << duration.count() << " milliseconds" << std::endl;

    auto robustMatchStartTime = std::chrono::high_resolution_clock::now();
    shoMatcher.runRobustFeatureMatching();
    auto robustMatchEndTime = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(robustMatchEndTime - robustMatchStartTime);
    std::cout << "Time taken for robust feature matching: " << duration.count() << " milliseconds" << std::endl;
    //******End matching pipeline******

    //***Begin tracking pipeline *****
    ShoTracker tracker(make_shared<FlightSession>(flight), shoMatcher.getCandidateImages());
    auto tracksGraph = tracker.buildTracksGraph();
    cerr << "Created tracks graph " << "\n";
    //*****End tracking pipeline *********

    auto bootstrapPair = make_pair(FLAGS_image_1, FLAGS_image_2);
    Reconstructor<RotationOnlyReconstructabilityScore> rotationOnlyReconstructor{make_shared<FlightSession>(flight), tracksGraph, bootstrapPair, FLAGS_outlier_threshold};
    Reconstructor<SnavelyReconstructionabilityScore> snavelyReconstructor{make_shared<FlightSession>(flight), tracksGraph, bootstrapPair, FLAGS_outlier_threshold};
    Reconstructor<MatchesCountReconstructabilityScore> matchesCountReconstructor{make_shared<FlightSession>(flight), tracksGraph, bootstrapPair, FLAGS_outlier_threshold};


    if (FLAGS_reconstruction_score == "matchescount") {
        matchesCountReconstructor.runIncrementalReconstruction(tracker);
    }
    else if (FLAGS_reconstruction_score == "snavely") {
        snavelyReconstructor.runIncrementalReconstruction(tracker);
    }
    else {
        rotationOnlyReconstructor.runIncrementalReconstruction(tracker);
    }

    cerr << "Finished incremental runIncrementalReconstructionreconstruction \n\n";
}