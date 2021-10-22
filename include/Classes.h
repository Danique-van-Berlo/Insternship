#ifndef DANIQUE_CLASSES_H
#define DANIQUE_CLASSES_H

class Segment {
public:
    std::vector<double> p1;
    std::vector<double> p2;
    std::vector<double> dv;
};

class Segment2 {
public:
    std::vector<double> p1;
    std::vector<double> p2;
    std::vector<double> dv;
    int index;
    int cat; //category: whole segment, first part segment, second part segment.
    int number;
};

class Line {
public:
    std::vector<double> p1;
    std::vector<double> p2;
};



#endif //DANIQUE_CLASSES_H
