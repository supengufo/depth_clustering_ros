//
// Created by alex on 2020/5/5.
//

#ifndef SRC_POINTINFO_H
#define SRC_POINTINFO_H
struct PointInfo {
public:
    int index_;
    float depth_;
    PointInfo(int index,float depth) : index_(index), depth_(depth){};
};
#endif //SRC_POINTINFO_H
