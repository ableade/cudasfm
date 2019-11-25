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

using std::pair;
using std::cerr;
using std::vector;
using std::string;
using std::map;

DEFINE_bool(resize, false, "Choose whether to resize images before feature extraction");
DEFINE_int32(max_image_size, FEATURE_PROCESS_SIZE, "If resizing, this is the max width to use for resizing");
DEFINE_string(feature_type, "SURF", "Feature detection algorithm to use. Choose from SIFT, SURF, ORB, HAHOG, the default is ORB");
DEFINE_string(reconstruction_score, "matchescount", "Scoring metric to sort image pairs");



int main(int argc, char* argv[])
{
    map<string, RobustMatcher::Feature> featureMatchingAlgorithms {
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
            Specify the --help option to show additional flags \n";
        exit(1);
    }
    FlightSession flight;
    (argc > 2) ? flight = FlightSession(argv[1], argv[2]) : flight = FlightSession(argv[1]);

   
    RobustMatcher::Feature featureType = RobustMatcher::Feature::orb;
    try {
        featureType = featureMatchingAlgorithms.at(FLAGS_feature_type);
    }
    catch (std::out_of_range &e) {
        featureType = RobustMatcher::Feature::orb;
    }
    
    auto extractionSize = -1;
    if (FLAGS_resize) {
        extractionSize = FLAGS_max_image_size;
    }
    ShoMatcher shoMatcher(flight, extractionSize, featureType);

    //**** Begin Matching Pipeline ******
    if (argc > 3) {
        //A candidate file was provided 
        const auto candidateFile = argv[3];
        cerr << "Using candidate file " << candidateFile << std::endl;
        shoMatcher.getCandidateMatchesFromFile(candidateFile);
    }
    else {
        double range = 0.00359;
        shoMatcher.getCandidateMatchesUsingSpatialSearch(range);
    }
    shoMatcher.extractFeatures();
    shoMatcher.runRobustFeatureMatching();
    //******End matching pipeline******

    //***Begin tracking pipeline *****
    ShoTracker tracker(flight, shoMatcher.getCandidateImages());
    auto tracksGraph = tracker.buildTracksGraph();
    cerr << "Created tracks graph " << "\n";
    //*****End tracking pipeline *********

    Reconstructor<RotationOnlyReconstructabilityScore> rotationOnlyReconstructor{ flight, tracksGraph };
    Reconstructor<SnavelyReconstructionabilityScore> snavelyReconstructor{ flight, tracksGraph };
    Reconstructor<MatchesCountReconstructabilityScore> matchesCountReconstructor{ flight, tracksGraph };

  
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