#include <QCoreApplication>
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <QFile>
#include <QTextStream>
#include <QStringList>

using namespace std;

int main(/*int argc, char *argv[]*/)
{
    const int totNumberOfFeats = 1926; //9459442;
    const int dimOfFeats = 88;

    cv::Mat featMat(totNumberOfFeats, dimOfFeats, CV_32FC1), labels;
    cv::Mat centers;

    int k = 1000;

    QFile file("/home/emredog/LIRIS-data/training-validation_features/vid0004_5_enter-leave.out");
    if (!file.open(QIODevice::ReadOnly))
    {
        std::cerr << "CANT OPEN FILE!!!";
        return -1;
    }

    QTextStream in(&file);
    QString line;
    QStringList fields;
    long errorCounter = 0;
    long lineCounter = 0;
    bool ok = true;

    while (!in.atEnd())
    {
        line = in.readLine();
        fields = line.split(" ");

        for (int i = 0; i<fields.count(); i++)
        {
            featMat.at<float>(lineCounter, i) = fields[i].toFloat(&ok);
            if (!ok) errorCounter++;
        }

        fields.clear();
        line.clear();
        cout << ".";
        lineCounter++;
    }
    in.reset();

    cout << endl << "Encountered " << errorCounter << " errors." << endl;

    cout << "Starting kmeans..." << endl;

    double compactness = cv::kmeans(featMat, k, labels,
                                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 5.0),
                                    100, cv::KMEANS_PP_CENTERS, centers);

    cout << "K-means completed with compactness: " << compactness << endl;

    QFile output("K-Means_001.out");
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
