#include "fuzzy.h"

fuzzy::fuzzy()
{

}

void fuzzy::fuzzy_set_data(std::vector<double> input, std::vector<double> output)
{
    input_vector.clear();
    output_vector.clear();
    if(input.size() == 0){
        log_console("input vector is empty");
        return;
    }
    if(output.size() == 0){
        log_console("output vector is empty");
        return;
    }
    if(input.size() != output.size()){
        log_console("output and input vector is in different sizes");
        return;
    }
    for(auto i: input){
        input_vector.push_back(i);
    }
    for(auto i: output){
        output_vector.push_back(i);
    }
}

void fuzzy::output_fuzzy(double input_value, double& output_value)
{
    std::vector<double> sub_vector;
    for(auto i: input_vector){
        sub_vector.push_back(fabs(input_value - i));
    }
    int min1_index = std::min_element(sub_vector.begin(), sub_vector.end()) - sub_vector.begin();
    sub_vector[min1_index] = 1000000;
    int min2_index = std::min_element(sub_vector.begin(), sub_vector.end()) - sub_vector.begin();

    double min_latch, max_latch;
    int min_index, max_index;
    if(input_vector.at(min1_index) > input_vector.at(min2_index)){
        min_latch = input_vector.at(min2_index); min_index = min2_index;
        max_latch = input_vector.at(min1_index); max_index = min1_index;
    }else{
        min_latch = input_vector.at(min1_index); min_index = min1_index;
        max_latch = input_vector.at(min2_index); max_index = min2_index;
    }

    double amin = (max_latch - input_value)/(max_latch - min_latch);
    double amax = (input_value - min_latch)/(max_latch - min_latch);
    output_value = amin*output_vector.at(min_index) + amax*output_vector.at(max_index);

}
fuzzy fuzzy_control;
