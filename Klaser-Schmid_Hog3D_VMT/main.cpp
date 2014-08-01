#include <QCoreApplication>

// std libs
#include <cassert>
#include <iostream>
#include <cmath>
#include <fstream>
#include <ctime>
#include <cstdlib>

// boost libs
#include <boost/program_options.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/typeof/typeof.hpp>
#include <boost/scoped_ptr.hpp>

// my stuff
//#include "functions.hpp"
#include <opencv/functions.h>
//#include "FastVideoGradientComputer.h"
#include "pclgradientcomputer.h" //ED 20140731
#include "FastHog3DComputer.h"
#include "vmtcalculator.h" //ED 20140731
//#include <opencv/Video.h>
#include <opencv/vmt.h>
#include <numeric/functions.hpp>

// std namespaces
using std::string;
using std::list;
using std::vector;
using std::cout;
using std::cerr;
using std::endl;

// boost namespaces
namespace po = boost::program_options;
namespace fs = boost::filesystem;

const string VERSION = "1.3.0";

// declerations
struct Position3D {
    double x;
    double y;
    double t;
    double sigma;
    double tau;
    boost::optional<Box3D> box3D;

    Position3D(double x_, double y_, double t_, double sigma_, double tau_)
        : x(x_), y(y_), t(t_), sigma(sigma_), tau(tau_)
    { }

    Position3D(double x0_, double y0_, double t0_, double width_, double height_, double length_)
        : x(x0_ + 0.5 * width_), y(y0_ + 0.5 * height_), t(t0_ + 0.5 * length_),
          sigma(0.5 * (width_ + height_)), tau(length_),
          box3D(Box3D(x0_, y0_, t0_, width_, height_, length_))
    { }
};

struct Position3DTLess {
    bool operator()(const Position3D& p1, const Position3D& p2) {
        return p1.t < p2.t;
    }
};
template<typename Vector, typename Stream>
void outputVector(Vector& v, Stream& out);
void printLicense();


int main(int argc, char *argv[])
{
    //QCoreApplication a(argc, argv);

    try {
        std::size_t xyNCells, tNCells, nPix;
        double normThreshold, cutZero;
        double sigmaSupport, tauSupport;
        std::string quantTypeStr;
        std::size_t polarBinsXY, polarBinsT;
        bool fullOrientation, gaussWeight, overlapCells, normGlobal, l1Norm;

        std::size_t xyNStride;
        std::size_t tNStride;
        double scaleOverlap;
        std::size_t bufferLength;
        double imgScaleFactor;
        std::size_t seed;
        std::size_t nSubsample;

        po::options_description argOpt("command line arguments");
        argOpt.add_options()
                ("video-file", po::value<string>(), "video file to process"); //FIXME

        po::options_description generalOpt("general options");
        generalOpt.add_options()
                ("help,h", "produce help message")
                ("license", "print license information")
                ("verbose,v", "verbose mode, the 3D boxes for which the descriptor is computed will be printed to stderr")
                ("position-file,p", po::value<string>(), "position file for sampling features; each line represents one feature position and consists of 5 elements (they may be floating points) which are: '<x-position> <y-position> <frame-number> <xy-scale> <t-scale>'; lines beginning with '#' are ignored")
                ("position-file2,q", po::value<string>(), "similar to --position-file, however with a different format; each line consists of 6 elements and describes a spatio-temporal cuboid for which the descriptor is computed: '<x> <y> <frame-start> <width> <height> <length>'; lines beginning with '#' are ignored")
                ("track-file,t", po::value<string>(), "track file for sampling features; each line represents a bounding box at a given frame; a line consists of 5 elements (they may be floating points) which are: '<frame-number> <top-left-x-position> <top-left-y-position> <width> <height>'; lines beginning with '#' are ignored; for a given position file, features with a center point that lies outside the set of given bounding boxes are ignored; for dense sampling, the --xy-nstride and --t-nstride options will be relative to the track (length and bounding boxes)")
                ("loose-track", "denotes that the track file is a collection of bounding boxes; for dense sampling, descriptors will be computed for each given bounding box in the track file")
                //                ("shot-file", po::value<string>(), "simple file with shot boundaries (i.e., frame number of beginning of a new shot) separate by any whitespace character; features crossing shot boundaries will be suppressed")
                ("force,f", "force computation of features, no suppression if features do not fit completely in the full video/buffer")
                ("n-subsample", po::value<std::size_t>(&nSubsample)->default_value(1), "subsample every n-th feautre (in average)")
                ("seed", po::value<std::size_t>(&seed)->default_value(time(0), "timestamp"), "seed for subsampling (default current time)")
                //            ("dump-frame", po::value<std::size_t>(), "this option lets you double check whether the video is read correctly, give it a frame number and it will save this frame from the video as 'frame<x>.png' in the curent working directory, then it will quit")
                ;

        po::options_description descriptorOpt("descriptor options");
        descriptorOpt.add_options()
                ("kth-optimized", "by default, the parameter setting optimized on the Hollywood2 training dataset (see explanations at the end) is employed; if this flag is set, the parameter settings optimized on the KTH training dataset are being used.")
                ("xy-ncells", po::value<std::size_t>(&xyNCells)->default_value(2), "number of HOG3D cells in x and y direction")
                ("t-ncells", po::value<std::size_t>(&tNCells)->default_value(5), "number of HOG3D cells in time")
                ("npix", po::value<std::size_t>(&nPix)->default_value(4), "number of hyper pixel support for each HOG3D cell, i.e., the histogram of a cell is computed for SxSxS subblocks")
                ("norm-threshold,n", po::value<double>(&normThreshold)->default_value(0.1, "0.1"), "suppress features with a descriptor L2-norm lower than the given threshold")
                ("cut-zero", po::value<double>(&cutZero)->default_value(0.25, "0.25"), "descriptor vector is normalized, its values are limited to given value, and the vector is renormalized again")
                ("sigma-support", po::value<double>(&sigmaSupport)->default_value(24), "on the original scale, sigma determines the size of the region around a sampling point for which the descriptor is computed; for a different scale, this size is given by: <characteristic-scale>*<sigma-support>; sigma is the support in x- and y-direction")
                ("tau-support", po::value<double>(&tauSupport)->default_value(12), "similar to 'sigma-support'; tau is the support in time")
                ("quantization-type,P", po::value<string>(&quantTypeStr)->default_value("polar"), "method that shall be taken for 3D gradient quantization: 'icosahedron' (20 faces=bins), 'dodecahedron' (12 faces=bins), 'polar' (binning on polar coordinates, you can specify the binning)")
                ("polar-bins-xy", po::value<std::size_t>(&polarBinsXY)->default_value(5), "number of bins for the XY-plane orientation using polar coordinate quantization (has either full or half orientation)")
                ("polar-bins-t", po::value<std::size_t>(&polarBinsT)->default_value(3), "number of bins for the XT-plane orientation using polar coordinate quantization (has always half orientation")
                ("full-orientation,F", po::value<bool>(&fullOrientation)->default_value(false), "By default, the half orientation is used (thus resulting in half the number of bins); if this flag is set, only the full sphere is used for quantization, thus doubling the number of bins")
                ("gauss-weight,G", po::value<bool>(&gaussWeight)->default_value(false), "By default, each (hyper) pixel has a weight = 1; this flag enables Gaussian weighting similar to the SIFT descriptor")
                ("overlap-cells,O", po::value<bool>(&overlapCells)->default_value(false), "Given this flag, cells in the descriptor will be 50% overlapping")
                ("norm-global,N", po::value<bool>(&normGlobal)->default_value(false), "By default, each cell in the descriptor is normalized; given this flag, normalization is carried out on the complete descriptor")
                ("l1-norm", po::value<bool>(&l1Norm)->default_value(false), "Given this flag, the each cell desriptor (or the full descriptor if given '--norm-global') will be normalized with L1 norm; by default normalization is done using L2-norm");

        po::options_description denseOpt("dense sampling options");
        denseOpt.add_options()
                ("xy-nstride", po::value<std::size_t>(&xyNStride), "how many features are sampled in x/y direction on the smallest scale (specify either xy-nstride or xy-stride)")
                ("xy-stride", po::value<double>(), "specifies the stride in x/y direction (in pixel) on the smallest scale (specify either xy-nstride or xy-stride)")
                ("xy-max-stride", po::value<double>(), "specifies the maximum stride (and indirectly its scale) for x/y")
                //                ("xy-max-scale", po::value<double>(), "specifies the maximum scale for x/y")
                //                ("xy-scale", po::value<double>(&xyScaleFactor)->default_value(sqrt(2), "sqrt(2)"), "scale factor for different scales in x/y direction")
                ("t-nstride", po::value<std::size_t>(&tNStride), "how many features are sampled in time on the smallest scale (specify either t-nstride or t-stride)")
                ("t-stride", po::value<double>(), "specifies the stride in t direction (in frames) on the smalles scale (specify either t-nstride or t-stride)")
                ("t-max-stride", po::value<double>(), "specifies the maximum stride (and indirectly its scale) for t")
                //                ("t-max-scale", po::value<double>(), "specifies the maximum scale for t")
                //                ("t-scale", po::value<double>(&tScaleFactor)->default_value(sqrt(2), "sqrt(2)"), "scale factor for different scales in time")
                //                ("scale-overlap", po::value<double>(&scaleOverlap)->default_value(2, "2"), "controls overlap of adjacent features, scales size of 3D box; for a factor of 1, features will be adjacent, any factor greater than 1 will cause overlapping features; a factor of 2 will double the size of the box (along each dimension) and thus will result in an overlap of 50%")
                ;

        po::options_description videoOpt("video options");
        videoOpt.add_options()
                //            ("start-frame,S", po::value<std::size_t>(), "if given, feature extraction starts at given frame")
                //            ("end-frame,E", po::value<std::size_t>(), "if given, feature extraction ends at given frame (including this frame)")
                //            ("start-time,s", po::value<double>(), "if given, feature extraction starts at the first frame at or after the given time (in seconds)")
                //            ("end-time,e", po::value<double>(), "if given, feature extraction ends at the last frame before or at the given time (in seconds)")
                ("buffer-length,b", po::value<std::size_t>(&bufferLength)->default_value(100), "length of the internal video buffer .. controls the memory usage as well as the maximal scale in time for features")
                ("simg", po::value<double>(&imgScaleFactor)->default_value(1), "scale the input video by this given factor");

        po::options_description desc;
        desc.add(argOpt).add(generalOpt).add(descriptorOpt).add(denseOpt).add(videoOpt);
        po::positional_options_description p;
        p.add("video-file", 1);

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
        po::notify(vm);

        //
        // check the command line arguments
        //

        if (vm.count("license")) {
            printLicense();
            return EXIT_SUCCESS;
        }

        if (vm.count("help")) {
            cout << endl;
            cout << "usage:" << endl;
            cout << "    extractFeatures [options] <video-file>" << endl;
            cout << "or (for testing):" << endl;
            cout << "    extractFeatures --dump-frame <frame-number> <video-file>" << endl << endl;
            cout << "output format is:" << endl;
            cout << "    <x> <y> <frame> <x-normalized> <y-norm.> <t-norm.> <xy-scale> <t-scale> <descriptor>" << endl << endl;
            cout << "version: " << VERSION << endl << endl;
            cout << "For license information use the option '--license'." << endl << endl;
            cout << desc << endl;
            cout << "descriptor parameters:" << endl;
            cout << "  By default, descriptor parameters are employed that have been learned on the" << endl;
            cout << "  training set of the Hollywood2 actions database. The setting is as follows:" << endl;
            cout << "    xy-ncells=2" << endl
                 << "    t-ncells=5" << endl
                 << "    npix=4" << endl
                 << "    sigma-support=24" << endl
                 << "    tau-support=12" << endl
                 << "    quantization-type=polar" << endl
                 << "    polar-bins-xy=5" << endl
                 << "    polar-bins-t=3" << endl
                 << "    half-orientation" << endl
                 << "    normalization of cell histograms independently with L2 norm" << endl << endl;
            cout << "  Optionally, a parameter setting learned on the KTH training set can be " << endl;
            cout << "  chosen by setting the option '--kth-optimized'. The parameters are then:" << endl;
            cout << "    xy-ncells=5"  << endl
                 << "    t-ncells=4" << endl
                 << "    npix=4" << endl
                 << "    sigma-support=16" << endl
                 << "    tau-support=4" << endl
                 << "    quantization-type=icosahedron" << endl
                 << "    half-orientation" << endl
                 << "    normalization of cell histograms independently with L2 norm" << endl << endl;
            cout << "  More information can be obtained in my PhD thesis: " << endl;
            cout << "    Alexander Klaeser, Learning Human Actions in Video, July 2010" << endl << endl;
            return EXIT_SUCCESS;
        }

        // check whether video file has been given
        if (!vm.count("video-file")) {
            cout << endl << "You need to specify a video file!" << endl << endl; //FIXME
            return EXIT_FAILURE;
        }

        std::string videoFileName = vm["video-file"].as<string>();

        // check whether all necessary
        if (!(vm.count("position-file") || vm.count("position-file2")) && !((vm.count("xy-nstride") && vm.count("t-nstride")) || (vm.count("xy-stride") || vm.count("t-stride")))) {
            cerr << endl << "Please enter either a position-file or parameters for dense samplng (either --xy-nstride and --t-nstride or --xy-stride and --t-stride)!" << endl << endl;
            return EXIT_FAILURE;
        }

        // check whether KTH options have been requested
        if (vm.count("kth-optimized")) {
            xyNCells = 5;
            tNCells = 4;
            nPix = 4;
            sigmaSupport = 16;
            tauSupport = 4;
            quantTypeStr = "icosahedron";
        }

        // get the correct type of orientation quantization
        FastHog3DComputer::QuantizationType quantizationType = FastHog3DComputer::Icosahedron;
        if (boost::algorithm::iequals(quantTypeStr, "icosahedron"))
            quantizationType = FastHog3DComputer::Icosahedron;
        else if (boost::algorithm::iequals(quantTypeStr, "dodecahedron"))
            quantizationType = FastHog3DComputer::Dodecahedron;
        else if (boost::algorithm::iequals(quantTypeStr, "polar"))
            quantizationType = FastHog3DComputer::PolarBinning;
        else
            throw std::runtime_error("unsupported platonic solid: " + quantTypeStr);


        boost::scoped_ptr<VmtCalculator> vmtCalculator(new VmtCalculator()); //FIXME!

        // init random seed
        srand(seed);

        //----------------------------------------------------------------------------------------
        // TRACK FILE IS READ HERE
        //----------------------------------------------------------------------------------------
        std::string trackFile = "";

        // read in the track file if it is given
        std::multimap<int, Box<double> > track;
        if (vm.count("track-file")) {
            // try to open the track file
            trackFile = vm["track-file"].as<string>();
            if (!fs::exists(trackFile))
                throw std::runtime_error("Track file does not exist: " + trackFile);
            std::ifstream file(trackFile.c_str());
            if (!file)
                throw std::runtime_error("Error opening track file: " + trackFile);

            // read in line by line
            std::string line;
            std::vector<double> tmpVec(5);
            while (file) {
                // get the next line
                std::getline(file, line);
                boost::algorithm::trim(line);
                if (line.empty() || boost::algorithm::starts_with(line, "#"))
                    continue;

                // parse line into vector
                std::istringstream iss(line);
                std::size_t i(0);
                for (i = 0; i < tmpVec.size() && iss >> tmpVec[i]; ++i);
                if (i < tmpVec.size() || !iss.eof())
                    throw std::runtime_error("Format error for tracks in line: " + line);
                int frame = static_cast<int>(round(tmpVec[0]));
                track.insert(std::pair<int, Box<double> >(frame, Box<double>(tmpVec[1], tmpVec[2], tmpVec[3], tmpVec[4])));
            }

        }

        //----------------------------------------------------------------------------------------
        //  TODO: CALCULATE VMT HERE
        //----------------------------------------------------------------------------------------
        cout << "Calculating VMT...";
        Vmt resultingVmt = vmtCalculator->calculateVmt(videoFileName, trackFile);
        cout << "..Done.\n";

        //----------------------------------------------------------------------------------------
        //  TODO: CALCULATE GRADIENT VECTOR HERE
        //----------------------------------------------------------------------------------------
        boost::scoped_ptr<PclGradientComputer> gradComputer(new PclGradientComputer(&resultingVmt));


        //init for HoG3D computer:
        FastHog3DComputer hogComputer(gradComputer.get(), quantizationType, polarBinsXY, polarBinsT,
                                      xyNCells, tNCells, nPix, normThreshold, cutZero, fullOrientation,
                                      gaussWeight, overlapCells, !normGlobal, l1Norm);

        // get dimensions of the video to process
        int width = gradComputer->getWidth();
        int height = gradComputer->getHeight();
        int depth = gradComputer->getDepth();

        cerr << "# video size: " << width << "x" << height << "x" << depth << endl;


        // do dense sampling ('position file' case is removed. ED 20140801 )
        bool isVerbose = vm.count("verbose");
        // if we have no positions file, we do dense sampling with given strides
        double xyStride = vm.count("xy-nstride") ? double(width) / xyNStride : vm["xy-stride"].as<double>();
        double zStride = vm.count("t-nstride") ? double(depth) / tNStride : vm["t-stride"].as<double>();

        // compute positions
        FastHog3DComputer::VectorType vec;
        std::list<Box<double> > bboxes; // list will contain all bounding boxes of a frame
        bboxes.push_back(Box<double>(0, 0, width, height)); // without a track, the full image is the only bounding box

        double zAdd = vm.count("loose-track") ? 1 : zStride;

        // spatial sampling
        for (double z = 0; z < depth; z += zAdd)
        {
            for (double x = 0; x <= width; x += xyStride)
            {
                for (double y = 0; y <= height; y += xyStride)
                {
                    // compute the feature position and size .. ensure that boxes are large enough
                    //cerr << "#   " << x << " " << y << " " << t << " " << xyScale << " " << tScale << " " << endl;
                    Box3D box;
                    box.width = std::max(hogComputer.getMinWidth(), xyStride);
                    box.height = std::max(hogComputer.getMinHeight(), xyStride);
                    box.depth = std::max(hogComputer.getMinLength(), zStride);
                    box.x = x/* - box.width * 0.5*/;
                    box.y = y/* - box.height * 0.5*/;
                    box.z = z/* - box.depth * 0.5*/;

//                    double xNormalized = std::min(0.999, (x - iBox->getLeft()) / iBox->getWidth());
//                    double yNormalized = std::min(0.999, (y - iBox->getTop()) / iBox->getHeight());

                    // check whether box is inside video
                    if (!gradComputer->isInBuffer(box) && !vm.count("force"))
                        continue;

                    // check for sampling
                    if (nSubsample > 1 && (rand() % nSubsample) > 0)
                        continue;

                    // compute feature vector
                    if (true/*isVerbose*/)
                        cerr << "# desc: " << box.x << ", " << box.y << ", " << box.z << ", "
                             << box.width << "x" << box.height << "x" << box.depth << endl;

                    continue; //FIXME remove me ED 20140801

                    //----------------------------------------------------------------------------------------
                    //  HOG3D DESCRIPTOR CALCULATED HERE
                    //----------------------------------------------------------------------------------------
                    //TODO:
                    //vec = hogComputer.getHog3D(box); UNCOMMENT ME!!! ED 20140801

                    // print out positions (original and normalized) and vector
                    if (vec.size() > 0)
                    {
                        //FIXME : I put 0 instead of xNormalized and yNormalized. ED 20140801
                        //FIXME : I put 1 instead of scale factors
                        cout << x << " " << y << " " << z << " " << 0 /*xNormalized*/ << " " << 0 /*yNormalized*/ << " " << (z / depth) << " " << 1 /*xyScale*/ << " " << 1 /*zScale*/ << " ";
                        outputVector(vec, cout);
                    }
                }
            }
        }
        //cerr << "# " << width << " " << height << " " << length << endl;
        //cerr << "# " << startFrame << " " << endFrame << endl;
        //cerr << "# " << xyStride << " " << xyStride << " " << tStride << " " << maxXyScale << " " << maxTScale << endl;
    }
    catch(boost::bad_lexical_cast &e) {
        cerr << endl << "Problem parsing command line arguments: " << e.what() << endl << endl;
        return EXIT_FAILURE;
    }
    catch(std::exception& e) {
        cerr << endl << "Error during execution: " << e.what() << endl << endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

// definitions
template<typename Vector, typename Stream>
void outputVector(Vector& v, Stream& out)
{
    std::size_t iNext(0);
    for (typename Vector::iterator i = v.begin(); i != v.end(); ++i, ++iNext) {
        assert(i.index() >= iNext);
        for (; iNext < i.index(); ++iNext) {
            out << "0";
            if (iNext < v.size() - 1)
                out << " ";
        }
        out << *i;
        if (i.index() < v.size() - 1)
            out << " ";
    }
    for (; iNext < v.size(); ++iNext) {
        out << "0";
        if (iNext < v.size() - 1)
            out << " ";
    }
    out << std::endl;
}

void printLicense()
{
    cout << endl
         << "LICENSE CONDITIONS FOR SOFTWARE" << endl << endl
         << "This software package for computing HOG3D descriptors in video is being made" << endl
         << "available for research purposes only. Any commercial use or resale of this" << endl
         << "software is prohibited without agreement of the author and INRIA Rhone-Alpes." << endl
         << "For further details, contact Alexander Klaeser (alexander.klaser@inrialpes.fr)." << endl << endl
         << "THE AUTHOR MAKES NO REPRESENTATIONS OR WARRANTIES OF ANY KIND CONCERNING THIS" << endl
         << "SOFTWARE." << endl << endl
         << "Copyright (2009), INRIA Rhone-Alpes - LEAR" << endl << endl;
}
