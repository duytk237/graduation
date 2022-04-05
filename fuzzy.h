#ifndef FUZZY_H
#define FUZZY_H

#include <QObject>
#include "define_parameter.h"
class fuzzy: public QObject
{
    Q_OBJECT

private:
    std::vector<double> input_vector;
    std::vector<double> output_vector;
public:
    fuzzy();
    void fuzzy_set_data(std::vector<double> input, std::vector<double> output);
    void output_fuzzy(double input_value, double& output_value);
};
extern fuzzy fuzzy_control;
#endif // FUZZY_H
