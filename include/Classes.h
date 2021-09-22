#ifndef DANIQUE_CLASSES_H
#define DANIQUE_CLASSES_H

class Segment {
public:
    std::vector<double> p1;
    std::vector<double> p2;
    double sigma;
    std::vector<double> dv;
};

class Line {
public:
    std::vector<double> p1;
    std::vector<double> p2;
};

class Distance {
public:
    std::vector<double> p1;
    std::vector<double> p2;
    std::vector<double> dv;
};

#endif //DANIQUE_CLASSES_H
