#include <QCoreApplication>
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QTime>

using namespace std;

int main(/*int argc, char *argv[]*/)
{
    const int totNumberOfFeats = 9459442;
    const int dimOfFeats = 88;
    const int randomFeatSize = 500000;
    const int nrOfUnwantedFeats = 8;

    cv::Mat featMat(randomFeatSize, dimOfFeats-nrOfUnwantedFeats, CV_32FC1);
    cv::Mat labels;
    cv::Mat centers;

    int k = 1000;

    //GENERATE RANDOM INDEXES
    cv::Mat randomIndexes(randomFeatSize, 1, CV_32SC1);
    cv::RNG rng;
    rng.fill(randomIndexes, cv::RNG::UNIFORM, 0, totNumberOfFeats);
    //for(int i=0; i<indexes.rows; i++) cout << indexes.at<int>(i, 0) << "\n";

    QList<int> sortedIndexes;
    for (int r=0; r<randomIndexes.rows; r++)
        sortedIndexes.push_back(randomIndexes.at<int>(r, 0));

    randomIndexes.release();

    qSort(sortedIndexes);


    QFile file("/home/emredog/LIRIS-data/training-validation_features/AllFeatures.feat");
    if (!file.open(QIODevice::ReadOnly))
    {
        std::cerr << "CANT OPEN FILE!!!";
        return -1;
    }

    QTextStream in(&file);
    QString line;
    QStringList fields;
    int featMatRowCounter = 0;
    int errorCounter = 0;
    int lineCounter = -1;
    int wantedLine = sortedIndexes.first();
    sortedIndexes.removeFirst();

    bool ok = true;

    while (!in.atEnd())
    {
        lineCounter++;

        //if its not the line we want, skip it
        if (lineCounter != wantedLine)
            continue;

        //if its the line we want, get the next line index we want
        wantedLine = sortedIndexes.first();
        sortedIndexes.removeFirst(); //remove it from list

        //if all indexes are finished
        if (sortedIndexes.isEmpty())
            break; //stop gathering features

        //and process the current line:
        line = in.readLine();
        fields = line.split(" ");

        for (int i = nrOfUnwantedFeats; i<fields.count(); i++)
        {
            float dummy = fields[i].toFloat(&ok);
            featMat.at<float>(featMatRowCounter, i-nrOfUnwantedFeats) = dummy;
            if (!ok) errorCounter++;
        }

        fields.clear();
        line.clear();
        //cout << ".";
        featMatRowCounter++;
    }
    in.reset();

    cout << endl << "Encountered " << errorCounter << " errors." << endl;

    cout << "Starting kmeans..." << endl;

    QTime timer;
    timer.start();
    double compactness = cv::kmeans(featMat, k, labels,
                                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 150, 0.1),
                                    100, cv::KMEANS_PP_CENTERS, centers);

    int elapsed = timer.elapsed();
    cout << "K-means completed in " << (double)elapsed/1000.0 << " with compactness: " << compactness << endl;

    QFile output("K-Means_s500K_k1000_150_01.out");
    if (!output.open(QIODevice::Append))
        return -1;

    in.setDevice(&output);

    for (int row = 0; row<centers.rows; row++)
    {
        for (int col = 0; col<centers.cols; col++)
            in << centers.at<float>(row, col) << " ";

         in << endl;
    }

    cout << "Wrote the file.";

}
