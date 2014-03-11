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

    const int termCrit_Count = 100;
    const double termCrit_Epsilon = 0.1;

    int k = 4000;

    cv::Mat featMat(randomFeatSize, dimOfFeats-nrOfUnwantedFeats, CV_32FC1);
    cv::Mat labels;
    cv::Mat centers;



    //GENERATE RANDOM INDEXES
    cv::Mat randomIndexes(randomFeatSize, 1, CV_32SC1);
    cv::RNG rng(static_cast<uint64>(QTime::currentTime().msec()));
    rng.fill(randomIndexes, cv::RNG::UNIFORM, 0, totNumberOfFeats);
    //for(int i=0; i<indexes.rows; i++) cout << indexes.at<int>(i, 0) << "\n";

    QList<int> sortedIndexes;
    for (int r=0; r<randomIndexes.rows; r++)
        sortedIndexes.push_back(randomIndexes.at<int>(r, 0));

    randomIndexes.release();

    qSort(sortedIndexes);


    QFile file("/home/emredog/LIRIS-data/training-validation_features_p20140310/AllFeaturesInSingleFile.features");
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
    int warninCounter = 0;
    int lineCounter = -1;
    int wantedLine = sortedIndexes.first();
    sortedIndexes.removeFirst();

    bool ok = true;

    while (!in.atEnd() && !sortedIndexes.isEmpty())
    {
        lineCounter++;

        if (lineCounter > wantedLine)
        {
            lineCounter -= 2;
            warninCounter++;
            continue;
        }

        //if its not the line we want, skip it
        if (lineCounter != wantedLine)
            continue;

        //if its the line we want, get the next line index we want
        wantedLine = sortedIndexes.first();
        sortedIndexes.removeFirst(); //remove it from list


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

    cout << endl << "Encountered " << errorCounter << " error(s) and " << warninCounter << " warning(s)." << endl;

    cout << "Starting kmeans with TermCriteria:Count: " << termCrit_Count << ", TermCriteria:Epsilon: "
         << termCrit_Epsilon << endl;

    QTime timer;
    timer.start();
    double compactness = cv::kmeans(featMat, k, labels,
                                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                                                     termCrit_Count, termCrit_Epsilon),
                                    100, cv::KMEANS_PP_CENTERS, centers);

    int elapsed = timer.elapsed();
    cout << "K-means completed in " << (double)elapsed/1000.0 << "seconds with compactness: " << compactness << endl;

    QFile output(QString("K-Means_s%1K_k%2_C%3_e%4.out").arg(randomFeatSize/1000).arg(k).arg(termCrit_Count).arg(termCrit_Epsilon));
    if (!output.open(QIODevice::Append))
        return -1;

    in.setDevice(&output);

    for (int row = 0; row<centers.rows; row++)
    {
        for (int col = 0; col<centers.cols; col++)
            in << centers.at<float>(row, col) << " ";

         in << endl;
    }

    cout << "Wrote the file." << endl;

}
